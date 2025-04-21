#include "position_control.h"

PositionControl::PositionControl(ros::NodeHandle& nh) : nh_(nh) {
    // 初始化传感器和机械臂初始位置
    nh_.param("sensor_x", x1_, 0.0);
    nh_.param("sensor_y", y1_, 0.0);
    nh_.param("sensor_z", z1_, 0.0);
    nh_.param("range_z", range_z, 0.0);
    nh_.param("arm_y0", y0_, 0.0);
    nh_.param("arm_z0", z0_, 0.0);

    // 订阅相机反馈数据
    sensor_data_sub_ = nh_.subscribe("/transform/sensor_data", 10, &PositionControl::sensorDataCallback, this);

    // 订阅机械臂末端位置
    endEffectorSub_ = nh_.subscribe("/naruto_pinocchio/fk_cart", 1, &PositionControl::fkCartCallback, this);
    endEffectorPub_ = nh_.advertise<geometry_msgs::Pose>("/naruto_position_control/ee", 1);
    // sub uav local position
    uavLocalPosSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &PositionControl::uavLocalposCallback, this);
    // 发布无人机目标位置
    position_pub_ = nh_.advertise<geometry_msgs::Pose>("/naruto_position_control/uav", 10);
}

void PositionControl::sensorDataCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // justify the data is valid? and transform to the body frame
    if (std::isnan(msg->position.y) || std::isnan(msg->position.z)) {
        measure.is_valid = false;
        ROS_WARN_STREAM_THROTTLE(5, "Invalid measurement data: " << msg->position.y << ", " << msg->position.z);
        return;
    } else {
        measure.is_valid = true;
        // double x = msg->position.x - x1_;
        double y = msg->position.y - y1_;
        double z = msg->position.z - z1_ + range_z;
        if(is_convert_mm_to_m_) {
            y = y / 1000.0;
            z = z / 1000.0;
        }
        measure.x = 0;
        measure.y = y;
        measure.z = z;
        ROS_INFO_STREAM_THROTTLE(1, "Measurement success: y = " << measure.y << ", z = " << measure.z);
    }
}

void PositionControl::fkCartCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // 更新当前机械臂末端位置
    current_fk_cart_ = *msg;
}

void PositionControl::uavLocalposCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // 更新当前无人机位姿
    current_uav_local_pos_ = *msg;
}
void PositionControl::uavPositionControl() {
    // 发布无人机目标位置
    geometry_msgs::Pose uavPos;
    uavPos.position.x = current_uav_local_pos_.position.x + measure.x;
    uavPos.position.y = current_uav_local_pos_.position.y + measure.y - y0_;
    uavPos.position.z = current_uav_local_pos_.position.z + measure.z - z0_;
    uavPos.orientation = current_uav_local_pos_.orientation;
    position_pub_.publish(uavPos);
}
void PositionControl::manipulatorPositionControl() {
    geometry_msgs::Pose target_ee;
    // 计算机械臂末端位置误差
    double error_y = measure.y - current_fk_cart_.position.y;
    double error_z = measure.z - current_fk_cart_.position.z;

    // 计算目标位置
    target_ee.position.x = current_fk_cart_.position.x;
    target_ee.position.y = current_fk_cart_.position.y + error_y;
    target_ee.position.z = current_fk_cart_.position.z + error_z;
    target_ee.orientation = current_fk_cart_.orientation;
    // 发布机械臂目标位置
    endEffectorPub_.publish(target_ee);
}

void PositionControl::updateControl() {
    uavPositionControl();
    manipulatorPositionControl();
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    PositionControl position_control(nh);
    // 试一下是否需要开启并行，rostopic一下发出来的target_ee,看看频率
    // 如果要开，把ros::spinOnce();注释掉
    // ros::AsyncSpinner spinner(2);
    // spinner.start();

    ros::Rate rate(30); // 控制频率
    while (ros::ok()) {
        ros::spinOnce();
        position_control.updateControl();
        rate.sleep();
    }

    return 0;
}