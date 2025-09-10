#include "gravity_alignment.hpp"
#include "IMU_Processing.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class GravityAlignedMapping
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher aligned_odom_pub_;
    ros::Publisher aligned_cloud_pub_;
    
    GravityAlignment gravity_aligner_;
    bool gravity_initialized_;
    bool use_fastlio_gravity_;
    
public:
    GravityAlignedMapping() : gravity_initialized_(false), use_fastlio_gravity_(true) {
        // 订阅IMU数据用于重力估计
        imu_sub_ = nh_.subscribe("/livox/imu", 1000, &GravityAlignedMapping::imuCallback, this);
        
        // 订阅Fast-LIO2的里程计输出
        odom_sub_ = nh_.subscribe("/Odometry", 100, &GravityAlignedMapping::odomCallback, this);
        
        // 发布对齐后的里程计和点云
        aligned_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/aligned_odometry", 100);
        aligned_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 100);
        
        ROS_INFO("Gravity Aligned Mapping initialized");
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!gravity_initialized_ && !use_fastlio_gravity_) {
            // 方法1: 从原始IMU数据估计重力方向
            gravity_aligner_.addIMUSample(msg);
            
            // 尝试估计重力方向
            if (gravity_aligner_.estimateGravityDirection(100)) {
                gravity_initialized_ = true;
                gravity_aligner_.printInfo();
                ROS_INFO("Gravity direction estimated from IMU data");
            }
        }
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (use_fastlio_gravity_ && !gravity_initialized_) {
            // 方法2: 从Fast-LIO2的状态中获取重力方向
            // 注意：这需要修改Fast-LIO2代码来发布重力信息，或者从日志文件读取
            // 这里假设我们有办法获取到Fast-LIO2的重力估计
            // Vector3d fastlio_gravity = getFastLIOGravity(); // 需要实现这个函数
            
            // 示例：假设我们知道重力方向（实际使用中需要从Fast-LIO2获取）
            Vector3d example_gravity(0.1, 0.2, -9.8); // 示例重力向量
            gravity_aligner_.setGravityFromFastLIO(example_gravity);
            gravity_initialized_ = true;
            ROS_INFO("Gravity direction set from Fast-LIO2");
        }
        
        if (gravity_initialized_) {
            // 变换里程计数据
            publishAlignedOdometry(msg);
        }
    }
    
    void publishAlignedOdometry(const nav_msgs::Odometry::ConstPtr& original_odom) {
        nav_msgs::Odometry aligned_odom = *original_odom;
        
        // 提取原始位置和姿态
        Vector3d position(original_odom->pose.pose.position.x,
                         original_odom->pose.pose.position.y,
                         original_odom->pose.pose.position.z);
        
        Quaterniond quat(original_odom->pose.pose.orientation.w,
                        original_odom->pose.pose.orientation.x,
                        original_odom->pose.pose.orientation.y,
                        original_odom->pose.pose.orientation.z);
        Matrix3d orientation = quat.toRotationMatrix();
        
        // 应用重力对齐变换
        Vector3d aligned_position;
        Matrix3d aligned_orientation;
        gravity_aligner_.transformPose(position, orientation, aligned_position, aligned_orientation);
        
        // 更新对齐后的里程计
        aligned_odom.pose.pose.position.x = aligned_position.x();
        aligned_odom.pose.pose.position.y = aligned_position.y();
        aligned_odom.pose.pose.position.z = aligned_position.z();
        
        Quaterniond aligned_quat(aligned_orientation);
        aligned_odom.pose.pose.orientation.w = aligned_quat.w();
        aligned_odom.pose.pose.orientation.x = aligned_quat.x();
        aligned_odom.pose.pose.orientation.y = aligned_quat.y();
        aligned_odom.pose.pose.orientation.z = aligned_quat.z();
        
        // 变换速度
        Vector3d velocity(original_odom->twist.twist.linear.x,
                         original_odom->twist.twist.linear.y,
                         original_odom->twist.twist.linear.z);
        Vector3d aligned_velocity = gravity_aligner_.transformPoint(velocity);
        
        aligned_odom.twist.twist.linear.x = aligned_velocity.x();
        aligned_odom.twist.twist.linear.y = aligned_velocity.y();
        aligned_odom.twist.twist.linear.z = aligned_velocity.z();
        
        // 更新坐标系名称
        aligned_odom.header.frame_id = "gravity_aligned_world";
        aligned_odom.child_frame_id = "gravity_aligned_body";
        
        aligned_odom_pub_.publish(aligned_odom);
    }
    
    // 示例：变换点云数据
    void transformPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        if (!gravity_initialized_) return;
        
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // 应用重力对齐变换
        std::vector<pcl::PointXYZI> transformed_points;
        gravity_aligner_.transformPoints(cloud->points, transformed_points);
        
        // 创建变换后的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        aligned_cloud->points = transformed_points;
        aligned_cloud->width = transformed_points.size();
        aligned_cloud->height = 1;
        aligned_cloud->is_dense = true;
        
        // 发布对齐后的点云
        sensor_msgs::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned_cloud, aligned_msg);
        aligned_msg.header = cloud_msg->header;
        aligned_msg.header.frame_id = "gravity_aligned_world";
        
        aligned_cloud_pub_.publish(aligned_msg);
    }
};

// 独立使用示例
void standaloneExample() {
    std::cout << "=== Standalone Gravity Alignment Example ===" << std::endl;
    
    GravityAlignment aligner;
    
    // 方法1: 模拟IMU数据进行重力估计
    std::cout << "Method 1: Estimating gravity from simulated IMU data..." << std::endl;
    
    // 模拟静止状态下的IMU数据（重力向下，带少量噪声）
    for (int i = 0; i < 100; ++i) {
        sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
        
        // 添加少量随机噪声
        double noise_x = (rand() % 100 - 50) / 1000.0; // ±0.05 m/s²
        double noise_y = (rand() % 100 - 50) / 1000.0;
        double noise_z = (rand() % 100 - 50) / 1000.0;
        
        imu_msg->linear_acceleration.x = 0.1 + noise_x;   // 轻微倾斜
        imu_msg->linear_acceleration.y = 0.2 + noise_y;   // 轻微倾斜
        imu_msg->linear_acceleration.z = -9.81 + noise_z; // 主要重力分量
        
        aligner.addIMUSample(imu_msg);
    }
    
    if (aligner.estimateGravityDirection()) {
        aligner.printInfo();
        
        // 测试坐标变换
        std::cout << "\nTesting coordinate transformation..." << std::endl;
        
        // 原始点（世界坐标系）
        Vector3d original_point(1.0, 2.0, 3.0);
        std::cout << "Original point: " << original_point.transpose() << std::endl;
        
        // 变换后的点（重力对齐坐标系）
        Vector3d transformed_point = aligner.transformPoint(original_point);
        std::cout << "Transformed point: " << transformed_point.transpose() << std::endl;
        
        // 获取变换矩阵
        Matrix3d rotation = aligner.getGravityAlignmentRotation();
        std::cout << "\nGravity alignment rotation matrix:" << std::endl;
        std::cout << rotation << std::endl;
        
        // 验证z轴对齐
        Vector3d new_z_axis = rotation * Vector3d(0, 0, 1);
        Vector3d gravity_opposite = -aligner.getGravityDirection();
        std::cout << "\nNew Z-axis: " << new_z_axis.transpose() << std::endl;
        std::cout << "Gravity opposite: " << gravity_opposite.transpose() << std::endl;
        std::cout << "Alignment error: " << (new_z_axis - gravity_opposite).norm() << std::endl;
    }
    
    std::cout << "\n=== Example completed ===" << std::endl;
}

// 从Fast-LIO2状态获取重力的示例函数
Vector3d getFastLIOGravityFromState(const state_ikfom& state) {
    // Fast-LIO2中重力存储为S2流形
    // 这里假设我们能访问到state.grav
    Vector3d gravity_vector;
    gravity_vector << state.grav[0], state.grav[1], state.grav[2];
    return gravity_vector;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gravity_aligned_mapping");
    
    // 运行独立示例
    standaloneExample();
    
    // 启动ROS节点
    GravityAlignedMapping mapper;
    
    ROS_INFO("Gravity aligned mapping node started. Waiting for data...");
    ros::spin();
    
    return 0;
}