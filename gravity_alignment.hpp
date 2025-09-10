#ifndef GRAVITY_ALIGNMENT_HPP
#define GRAVITY_ALIGNMENT_HPP

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <iostream>

using namespace Eigen;

class GravityAlignment
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GravityAlignment() : initialized_(false), sample_count_(0) {
        gravity_samples_.clear();
        gravity_direction_ = Vector3d(0, 0, -1); // 默认重力方向向下
        G_magnitude_ = 9.81; // 重力加速度大小
    }
    
    ~GravityAlignment() {}
    
    /**
     * @brief 添加IMU数据样本用于重力方向估计
     * @param imu_msg IMU消息
     * @return 是否成功添加样本
     */
    bool addIMUSample(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        Vector3d acc(imu_msg->linear_acceleration.x, 
                    imu_msg->linear_acceleration.y, 
                    imu_msg->linear_acceleration.z);
        
        gravity_samples_.push_back(acc);
        sample_count_++;
        
        // 保持样本数量在合理范围内
        if (gravity_samples_.size() > max_samples_) {
            gravity_samples_.pop_front();
        }
        
        return true;
    }
    
    /**
     * @brief 从收集的IMU样本估计重力方向
     * @param min_samples 最少需要的样本数
     * @return 是否成功估计重力方向
     */
    bool estimateGravityDirection(int min_samples = 50) {
        if (gravity_samples_.size() < min_samples) {
            std::cout << "Not enough samples for gravity estimation. Current: " 
                     << gravity_samples_.size() << ", Required: " << min_samples << std::endl;
            return false;
        }
        
        // 计算加速度均值
        Vector3d mean_acc = Vector3d::Zero();
        for (const auto& acc : gravity_samples_) {
            mean_acc += acc;
        }
        mean_acc /= gravity_samples_.size();
        
        // 计算方差以检查IMU是否足够静止
        Vector3d variance = Vector3d::Zero();
        for (const auto& acc : gravity_samples_) {
            Vector3d diff = acc - mean_acc;
            variance += diff.cwiseProduct(diff);
        }
        variance /= gravity_samples_.size();
        
        double total_variance = variance.norm();
        std::cout << "IMU variance: " << total_variance << std::endl;
        
        if (total_variance > max_variance_threshold_) {
            std::cout << "IMU motion detected (variance too high). Please keep IMU static during initialization." << std::endl;
            return false;
        }
        
        // 重力方向是平均加速度的反方向（因为IMU测量的是重力的反作用力）
        gravity_direction_ = -mean_acc.normalized();
        G_magnitude_ = mean_acc.norm();
        
        std::cout << "Estimated gravity direction: " << gravity_direction_.transpose() << std::endl;
        std::cout << "Estimated gravity magnitude: " << G_magnitude_ << " m/s²" << std::endl;
        
        initialized_ = true;
        return true;
    }
    
    /**
     * @brief 直接设置重力方向（如果已知）
     * @param gravity_dir 重力方向向量（不需要归一化）
     */
    void setGravityDirection(const Vector3d& gravity_dir) {
        gravity_direction_ = gravity_dir.normalized();
        initialized_ = true;
    }
    
    /**
     * @brief 从Fast-LIO2的状态中获取重力方向
     * @param grav_s2 Fast-LIO2中的S2重力表示
     */
    void setGravityFromFastLIO(const Vector3d& grav_s2) {
        // Fast-LIO2中的重力向量已经是世界坐标系下的重力方向
        gravity_direction_ = grav_s2.normalized();
        G_magnitude_ = grav_s2.norm();
        initialized_ = true;
        std::cout << "Gravity direction from Fast-LIO2: " << gravity_direction_.transpose() << std::endl;
    }
    
    /**
     * @brief 计算将当前坐标系z轴对齐到重力反方向的旋转矩阵
     * @return 旋转矩阵
     */
    Matrix3d getGravityAlignmentRotation() const {
        if (!initialized_) {
            std::cerr << "Gravity direction not initialized!" << std::endl;
            return Matrix3d::Identity();
        }
        
        // 目标方向：z轴指向重力反方向（向上）
        Vector3d target_z = -gravity_direction_;
        Vector3d current_z(0, 0, 1);
        
        // 如果已经对齐，返回单位矩阵
        if (target_z.dot(current_z) > 0.999) {
            return Matrix3d::Identity();
        }
        
        // 如果完全相反，需要特殊处理
        if (target_z.dot(current_z) < -0.999) {
            // 选择一个垂直向量作为旋转轴
            Vector3d axis = current_z.cross(Vector3d(1, 0, 0));
            if (axis.norm() < 0.1) {
                axis = current_z.cross(Vector3d(0, 1, 0));
            }
            axis.normalize();
            return AngleAxisd(M_PI, axis).toRotationMatrix();
        }
        
        // 计算旋转轴和旋转角
        Vector3d rotation_axis = current_z.cross(target_z).normalized();
        double rotation_angle = acos(current_z.dot(target_z));
        
        // 构造旋转矩阵
        return AngleAxisd(rotation_angle, rotation_axis).toRotationMatrix();
    }
    
    /**
     * @brief 获取完整的变换矩阵（4x4）
     * @param translation 可选的平移向量
     * @return 4x4变换矩阵
     */
    Matrix4d getGravityAlignmentTransform(const Vector3d& translation = Vector3d::Zero()) const {
        Matrix4d transform = Matrix4d::Identity();
        transform.block<3,3>(0,0) = getGravityAlignmentRotation();
        transform.block<3,1>(0,3) = translation;
        return transform;
    }
    
    /**
     * @brief 变换点云或坐标点
     * @param points 输入点集
     * @param transformed_points 输出变换后的点集
     */
    template<typename PointType>
    void transformPoints(const std::vector<PointType>& points, 
                        std::vector<PointType>& transformed_points) const {
        if (!initialized_) {
            std::cerr << "Gravity direction not initialized!" << std::endl;
            return;
        }
        
        Matrix3d rotation = getGravityAlignmentRotation();
        transformed_points.resize(points.size());
        
        for (size_t i = 0; i < points.size(); ++i) {
            Vector3d point(points[i].x, points[i].y, points[i].z);
            Vector3d transformed = rotation * point;
            
            transformed_points[i].x = transformed.x();
            transformed_points[i].y = transformed.y();
            transformed_points[i].z = transformed.z();
            
            // 保留其他属性
            if constexpr (std::is_same_v<PointType, pcl::PointXYZI>) {
                transformed_points[i].intensity = points[i].intensity;
            }
        }
    }
    
    /**
     * @brief 变换单个点
     * @param point 输入点
     * @return 变换后的点
     */
    Vector3d transformPoint(const Vector3d& point) const {
        if (!initialized_) {
            std::cerr << "Gravity direction not initialized!" << std::endl;
            return point;
        }
        return getGravityAlignmentRotation() * point;
    }
    
    /**
     * @brief 变换位姿（位置+姿态）
     * @param position 输入位置
     * @param orientation 输入姿态（旋转矩阵）
     * @param transformed_position 输出变换后位置
     * @param transformed_orientation 输出变换后姿态
     */
    void transformPose(const Vector3d& position, const Matrix3d& orientation,
                      Vector3d& transformed_position, Matrix3d& transformed_orientation) const {
        if (!initialized_) {
            std::cerr << "Gravity direction not initialized!" << std::endl;
            transformed_position = position;
            transformed_orientation = orientation;
            return;
        }
        
        Matrix3d gravity_rotation = getGravityAlignmentRotation();
        transformed_position = gravity_rotation * position;
        transformed_orientation = gravity_rotation * orientation;
    }
    
    // Getter函数
    Vector3d getGravityDirection() const { return gravity_direction_; }
    double getGravityMagnitude() const { return G_magnitude_; }
    bool isInitialized() const { return initialized_; }
    
    // 设置参数
    void setMaxSamples(int max_samples) { max_samples_ = max_samples; }
    void setVarianceThreshold(double threshold) { max_variance_threshold_ = threshold; }
    
    /**
     * @brief 打印当前重力对齐信息
     */
    void printInfo() const {
        std::cout << "=== Gravity Alignment Info ===" << std::endl;
        std::cout << "Initialized: " << (initialized_ ? "Yes" : "No") << std::endl;
        if (initialized_) {
            std::cout << "Gravity direction: " << gravity_direction_.transpose() << std::endl;
            std::cout << "Gravity magnitude: " << G_magnitude_ << " m/s²" << std::endl;
            
            Matrix3d rotation = getGravityAlignmentRotation();
            Vector3d euler = rotation.eulerAngles(0, 1, 2) * 180.0 / M_PI;
            std::cout << "Alignment rotation (Roll, Pitch, Yaw): " << euler.transpose() << " degrees" << std::endl;
        }
        std::cout << "Sample count: " << gravity_samples_.size() << std::endl;
        std::cout << "===============================" << std::endl;
    }

private:
    bool initialized_;
    int sample_count_;
    Vector3d gravity_direction_;
    double G_magnitude_;
    
    std::deque<Vector3d> gravity_samples_;
    int max_samples_ = 200;
    double max_variance_threshold_ = 2.0; // 方差阈值，用于检测IMU是否静止
};

#endif // GRAVITY_ALIGNMENT_HPP