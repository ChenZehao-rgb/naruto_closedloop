#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/UInt16.h>
class PositionControl {
public:
    PositionControl(ros::NodeHandle& nh);
    void updateControl();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sensor_data_sub_;
    ros::Subscriber endEffectorSub_;
    ros::Subscriber uavLocalPosSub_;
    ros::Publisher position_pub_;
    ros::Publisher endEffectorPub_;

    geometry_msgs::Pose current_fk_cart_; // 当前机械臂末端位置
    geometry_msgs::Pose current_uav_local_pos_; // 当前无人机位姿
    geometry_msgs::Pose sensor_data_; // 相机坐标系下的目标位置

    struct Measure {
        bool is_valid;
        bool is_valid_prev;
        double x;
        double y;
        double z;
    } measure;

    double x1_, y1_, z1_, range_z; // 传感器安装位置
    tf2::Vector3 sensor_install_body_vec_; // 传感器安装位置向量
    tf2::Vector3 arm_ee_body_vec_; // 机械臂末端位置向量
    tf2::Matrix3x3 initial_rotation_; // 初始旋转矩阵
    bool initialized_ = false; // 是否已初始化
    double y0_, z0_;      // 机械臂初始末端位置
    bool is_convert_mm_to_m_ = true; // 是否需要转换单位
    bool test_indoor_ = false; // 是否为室内测试
    double max_compensate_; // 最大补偿量
    double alpha_; // 低通滤波系数
    void sensorDataCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void sensorDataCallback(const std_msgs::UInt16& msg);
    void fkCartCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void uavLocalposCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void uavLocalposCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void uavPositionControl();
    void manipulatorPositionControl();
};

#endif // POSITION_CONTROL_H