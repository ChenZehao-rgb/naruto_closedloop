#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/PositionTarget.h>
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
        double x;
        double y;
        double z;
    } measure;

    double x1_, y1_, z1_, range_z; // 传感器安装位置
    double y0_, z0_;      // 机械臂初始末端位置
    bool is_convert_mm_to_m_ = true; // 是否需要转换单位
    void sensorDataCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void fkCartCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void uavLocalposCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void uavPositionControl();
    void manipulatorPositionControl();
};

#endif // POSITION_CONTROL_H