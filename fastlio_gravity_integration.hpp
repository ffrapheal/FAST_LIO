#ifndef FASTLIO_GRAVITY_INTEGRATION_HPP
#define FASTLIO_GRAVITY_INTEGRATION_HPP

#include "gravity_alignment.hpp"
#include "use-ikfom.hpp"
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>

/**
 * @brief Fast-LIO2重力信息集成类
 * 用于从Fast-LIO2系统中提取重力方向并进行坐标对齐
 */
class FastLIOGravityIntegration
{
private:
    GravityAlignment gravity_aligner_;
    ros::Publisher gravity_pub_;
    ros::Publisher transform_pub_;
    bool gravity_published_;
    
public:
    FastLIOGravityIntegration(ros::NodeHandle& nh) : gravity_published_(false) {
        gravity_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/gravity_direction", 10);
        transform_pub_ = nh.advertise<geometry_msgs::TransformStamped>("/gravity_alignment_transform", 10);
    }
    
    /**
     * @brief 从Fast-LIO2状态更新重力方向
     * @param state Fast-LIO2的状态
     * @param timestamp 时间戳
     */
    void updateGravityFromFastLIO(const state_ikfom& state, double timestamp) {
        // 提取重力向量
        Vector3d gravity_vector;
        gravity_vector << state.grav[0], state.grav[1], state.grav[2];
        
        // 更新重力对齐器
        gravity_aligner_.setGravityFromFastLIO(gravity_vector);
        
        // 发布重力方向信息
        publishGravityDirection(gravity_vector, timestamp);
        
        // 发布变换信息
        publishAlignmentTransform(timestamp);
        
        if (!gravity_published_) {
            ROS_INFO("Gravity direction updated from Fast-LIO2:");
            ROS_INFO("  Gravity vector: [%.4f, %.4f, %.4f]", 
                     gravity_vector.x(), gravity_vector.y(), gravity_vector.z());
            ROS_INFO("  Gravity magnitude: %.4f m/s²", gravity_vector.norm());
            gravity_published_ = true;
        }
    }
    
    /**
     * @brief 获取重力对齐器的引用
     */
    GravityAlignment& getGravityAligner() {
        return gravity_aligner_;
    }
    
    /**
     * @brief 变换Fast-LIO2的位姿到重力对齐坐标系
     * @param fastlio_pos Fast-LIO2位置
     * @param fastlio_rot Fast-LIO2姿态
     * @param aligned_pos 输出对齐后位置
     * @param aligned_rot 输出对齐后姿态
     */
    void transformFastLIOPose(const Vector3d& fastlio_pos, const Matrix3d& fastlio_rot,
                             Vector3d& aligned_pos, Matrix3d& aligned_rot) {
        gravity_aligner_.transformPose(fastlio_pos, fastlio_rot, aligned_pos, aligned_rot);
    }
    
    /**
     * @brief 变换点云到重力对齐坐标系
     * @param input_cloud 输入点云
     * @param output_cloud 输出对齐后点云
     */
    template<typename PointType>
    void transformPointCloud(const std::vector<PointType>& input_cloud,
                           std::vector<PointType>& output_cloud) {
        gravity_aligner_.transformPoints(input_cloud, output_cloud);
    }
    
private:
    void publishGravityDirection(const Vector3d& gravity, double timestamp) {
        geometry_msgs::Vector3Stamped gravity_msg;
        gravity_msg.header.stamp = ros::Time(timestamp);
        gravity_msg.header.frame_id = "world";
        
        Vector3d gravity_normalized = gravity.normalized();
        gravity_msg.vector.x = gravity_normalized.x();
        gravity_msg.vector.y = gravity_normalized.y();
        gravity_msg.vector.z = gravity_normalized.z();
        
        gravity_pub_.publish(gravity_msg);
    }
    
    void publishAlignmentTransform(double timestamp) {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = ros::Time(timestamp);
        transform_msg.header.frame_id = "world";
        transform_msg.child_frame_id = "gravity_aligned_world";
        
        Matrix3d rotation = gravity_aligner_.getGravityAlignmentRotation();
        Quaterniond quat(rotation);
        
        transform_msg.transform.translation.x = 0.0;
        transform_msg.transform.translation.y = 0.0;
        transform_msg.transform.translation.z = 0.0;
        
        transform_msg.transform.rotation.w = quat.w();
        transform_msg.transform.rotation.x = quat.x();
        transform_msg.transform.rotation.y = quat.y();
        transform_msg.transform.rotation.z = quat.z();
        
        transform_pub_.publish(transform_msg);
    }
};

// 在laserMapping.cpp中集成重力对齐的修改建议
/*
在laserMapping.cpp的main函数中添加以下代码：

1. 在变量定义部分添加：
FastLIOGravityIntegration gravity_integration(nh);

2. 在主循环中，在状态更新后添加：
if (flg_EKF_inited) {
    // 更新重力方向
    gravity_integration.updateGravityFromFastLIO(state_point, Measures.lidar_beg_time);
    
    // 可选：发布对齐后的里程计
    nav_msgs::Odometry aligned_odom = odomAftMapped;
    Vector3d aligned_pos;
    Matrix3d aligned_rot;
    
    gravity_integration.transformFastLIOPose(
        Vector3d(state_point.pos(0), state_point.pos(1), state_point.pos(2)),
        state_point.rot.toRotationMatrix(),
        aligned_pos, aligned_rot
    );
    
    aligned_odom.pose.pose.position.x = aligned_pos.x();
    aligned_odom.pose.pose.position.y = aligned_pos.y();
    aligned_odom.pose.pose.position.z = aligned_pos.z();
    
    Quaterniond aligned_quat(aligned_rot);
    aligned_odom.pose.pose.orientation.w = aligned_quat.w();
    aligned_odom.pose.pose.orientation.x = aligned_quat.x();
    aligned_odom.pose.pose.orientation.y = aligned_quat.y();
    aligned_odom.pose.pose.orientation.z = aligned_quat.z();
    
    aligned_odom.header.frame_id = "gravity_aligned_world";
    aligned_odom.child_frame_id = "gravity_aligned_body";
    
    // 发布对齐后的里程计（需要创建相应的publisher）
    // pubAlignedOdom.publish(aligned_odom);
}

3. 在publisher定义部分添加：
ros::Publisher pubAlignedOdom = nh.advertise<nav_msgs::Odometry>("/aligned_odometry", 100000);
ros::Publisher pubAlignedCloud = nh.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 100000);
*/

#endif // FASTLIO_GRAVITY_INTEGRATION_HPP