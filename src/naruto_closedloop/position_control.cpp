#include "position_control.h"

PositionControl::PositionControl(ros::NodeHandle& nh) : nh_(nh), measure{false, false} {
    // 初始化传感器和机械臂初始位置
    nh_.param("sensor_x", sensor_install_body_vec_[0], 0.0);
    nh_.param("sensor_y", sensor_install_body_vec_[1], 0.0);
    nh_.param("sensor_z", sensor_install_body_vec_[2], 0.0);
    nh_.param("range_z", range_z, 0.0);
    nh_.param("arm_x0", arm_ee_body_vec_[0], 0.0);
    nh_.param("arm_y0", arm_ee_body_vec_[1], 0.0);
    nh_.param("arm_z0", arm_ee_body_vec_[2], 0.0);
    nh_.param("compensate_max", max_compensate_, 0.05);  // 最大5cm补偿量
    nh_.param("alpha_filter", alpha_, 0.8); // 低通滤波系数
    nh_.param("test_indoor", test_indoor_, false); // 是否在室内测试
    // 订阅相机反馈数据
    // sensor_data_sub_ = nh_.subscribe("/transform/sensor_data", 10, &PositionControl::sensorDataCallback, this);
    sensor_data_sub_ = nh_.subscribe("sensor/distance", 10, &PositionControl::sensorDataCallback, this);
    // 订阅机械臂末端位置
    endEffectorSub_ = nh_.subscribe("/naruto_pinocchio/fk_cart", 1, &PositionControl::fkCartCallback, this);
    endEffectorPub_ = nh_.advertise<geometry_msgs::Pose>("/naruto_position_control/ee", 1);
    if(test_indoor_) {
        // if no gps, sub imu
        uavLocalPosSub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &PositionControl::uavLocalposCallback, this);
    } else {
        // 订阅无人机位姿
        uavLocalPosSub_ = nh_.subscribe<geometry_msgs::Pose>("/mavros/local_position/pose", 1, &PositionControl::uavLocalposCallback, this);
    }
    // 发布无人机目标位置
    position_pub_ = nh_.advertise<geometry_msgs::Pose>("/naruto_position_control/uav", 10);
}

void PositionControl::sensorDataCallback(const std_msgs::UInt16& msg) {
    // justify the data is valid? and transform to the body frame
    // Check if the message fields are NaN (assuming msg is std_msgs::UInt16, which has a 'data' field)
    if (std::isnan(static_cast<double>(msg.data))) {
        measure.is_valid = false;
        ROS_WARN_STREAM_THROTTLE(5, "Invalid measurement data: " << msg.data);
        return;
    }
    measure.is_valid = true;
    // double x = msg->position.x - x1_;
    double distance = static_cast<double>(msg.data);
    // convert cm to m
    distance = distance / 100.0;
    double y = distance + sensor_install_body_vec_[1];
    // 低通滤波
    if (!measure.is_valid_prev) {
        measure.y = y; // 初始化滤波器状态
    } else {
        measure.y = alpha_ * measure.y + (1 - alpha_) * y;
    }
    measure.is_valid_prev = true;
    measure.x = 0;
    measure.z = 0;
    ROS_INFO_STREAM_THROTTLE(1, "Measurement success: y = " << measure.y);
}

void PositionControl::sensorDataCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // justify the data is valid? and transform to the body frame
    if (std::isnan(msg->position.y) || std::isnan(msg->position.z)) {
        measure.is_valid = false;
        ROS_WARN_STREAM_THROTTLE(5, "Invalid measurement data: " << msg->position.y << ", " << msg->position.z);
        return;
    }
    measure.is_valid = true;
    // double x = msg->position.x - x1_;
    double y = msg->position.y - sensor_install_body_vec_[1];
    double z = msg->position.z - sensor_install_body_vec_[2] + range_z;
    if(is_convert_mm_to_m_) {
        y = y / 1000.0;
        z = z / 1000.0;
    }
    // 低通滤波
    measure.y = alpha_ * measure.y + (1 - alpha_) * y;
    measure.z = alpha_ * measure.z + (1 - alpha_) * z;
    measure.x = 0;

    ROS_INFO_STREAM_THROTTLE(1, "Measurement success: y = " << measure.y << ", z = " << measure.z);
}

void PositionControl::fkCartCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // 更新当前机械臂末端位置
    current_fk_cart_ = *msg;
    manipulatorPositionControl();
}

void PositionControl::uavLocalposCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // 更新当前无人机位姿
    current_uav_local_pos_ = *msg;
}
void PositionControl::uavLocalposCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 更新当前无人机位姿
    current_uav_local_pos_.orientation = msg->orientation;
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
    // 计算当前无人机位姿的旋转矩阵
    tf2::Quaternion q(current_uav_local_pos_.orientation.x,
                    current_uav_local_pos_.orientation.y,
                    current_uav_local_pos_.orientation.z,
                    current_uav_local_pos_.orientation.w);
    tf2::Matrix3x3 current_rotation(q);
    if (!initialized_)
    {
        initial_rotation_ = current_rotation;
        initialized_ = true;
    }
    // 姿态补偿
    tf2::Matrix3x3 delta_rotation = initial_rotation_.inverse() * current_rotation;
    tf2::Vector3 fk_cart_pos_vec(current_fk_cart_.position.x, current_fk_cart_.position.y, current_fk_cart_.position.z);
    tf2::Vector3 compensate_vector_attitude = delta_rotation * fk_cart_pos_vec - fk_cart_pos_vec;
    if (compensate_vector_attitude.length() > max_compensate_) {
        compensate_vector_attitude = compensate_vector_attitude.normalized() * max_compensate_;
    }
    // 距离补偿（基于传感器）
    double distance_to_pole = measure.y;  // 已滤波
    // 设测距方向为 body 坐标系 y 轴（前方）
    tf2::Vector3 forward_dir_body(0, 1, 0);  // 假设前方是 +Y
    tf2::Vector3 distance_vector_body = distance_to_pole * forward_dir_body;
    tf2::Vector3 distance_vector_world = current_rotation * distance_vector_body;
    // 计算新的末端目标点
    geometry_msgs::Pose ee_target_local;
    ee_target_local.position.x = current_fk_cart_.position.x;
    ee_target_local.position.y = distance_vector_world[1] - compensate_vector_attitude[1];
    ee_target_local.position.z = current_fk_cart_.position.z - compensate_vector_attitude[2];
    ee_target_local.orientation = current_fk_cart_.orientation;

    // 发布机械臂目标位置
    endEffectorPub_.publish(ee_target_local);
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

    ros::spin();

    return 0;
}