#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>

using namespace Eigen;

class SimpleGravityAlignment
{
private:
    bool initialized_;
    Vector3d gravity_direction_;
    double G_magnitude_;
    
public:
    SimpleGravityAlignment() : initialized_(false), G_magnitude_(9.81) {
        gravity_direction_ = Vector3d(0, 0, -1);
    }
    
    void setGravityDirection(const Vector3d& gravity_dir) {
        gravity_direction_ = gravity_dir.normalized();
        G_magnitude_ = gravity_dir.norm();
        initialized_ = true;
        
        std::cout << "Gravity direction set to: " << gravity_direction_.transpose() << std::endl;
        std::cout << "Gravity magnitude: " << G_magnitude_ << " m/s²" << std::endl;
    }
    
    Matrix3d getGravityAlignmentRotation() const {
        if (!initialized_) {
            std::cerr << "Gravity direction not initialized!" << std::endl;
            return Matrix3d::Identity();
        }
        
        Vector3d target_z = -gravity_direction_; // Z轴指向重力反方向
        Vector3d current_z(0, 0, 1);
        
        if (target_z.dot(current_z) > 0.999) {
            return Matrix3d::Identity();
        }
        
        if (target_z.dot(current_z) < -0.999) {
            Vector3d axis = current_z.cross(Vector3d(1, 0, 0));
            if (axis.norm() < 0.1) {
                axis = current_z.cross(Vector3d(0, 1, 0));
            }
            axis.normalize();
            return AngleAxisd(M_PI, axis).toRotationMatrix();
        }
        
        Vector3d rotation_axis = current_z.cross(target_z).normalized();
        double rotation_angle = acos(current_z.dot(target_z));
        
        return AngleAxisd(rotation_angle, rotation_axis).toRotationMatrix();
    }
    
    Vector3d transformPoint(const Vector3d& point) const {
        if (!initialized_) {
            std::cerr << "Gravity direction not initialized!" << std::endl;
            return point;
        }
        return getGravityAlignmentRotation() * point;
    }
    
    Vector3d getGravityDirection() const { return gravity_direction_; }
    double getGravityMagnitude() const { return G_magnitude_; }
    bool isInitialized() const { return initialized_; }
    
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
        std::cout << "===============================" << std::endl;
    }
};

void testGravityAlignment() {
    std::cout << "=== Testing Simple Gravity Alignment ===" << std::endl;
    
    SimpleGravityAlignment aligner;
    
    // 测试1: 水平重力（标准情况）
    std::cout << "\nTest 1: Standard horizontal gravity" << std::endl;
    Vector3d horizontal_gravity(0, 0, -9.81);
    aligner.setGravityDirection(horizontal_gravity);
    aligner.printInfo();
    
    // 验证Z轴对齐
    Matrix3d rotation = aligner.getGravityAlignmentRotation();
    Vector3d original_z(0, 0, 1);
    Vector3d transformed_z = rotation * original_z;
    Vector3d expected_z = -aligner.getGravityDirection();
    
    std::cout << "Original Z-axis: " << original_z.transpose() << std::endl;
    std::cout << "Transformed Z-axis: " << transformed_z.transpose() << std::endl;
    std::cout << "Expected direction: " << expected_z.transpose() << std::endl;
    std::cout << "Alignment error: " << (transformed_z - expected_z).norm() << std::endl;
    
    // 测试2: 倾斜重力
    std::cout << "\nTest 2: Tilted gravity (30 degrees around X-axis)" << std::endl;
    double tilt_angle = 30.0 * M_PI / 180.0;
    Vector3d tilted_gravity(0, 9.81 * sin(tilt_angle), -9.81 * cos(tilt_angle));
    
    SimpleGravityAlignment tilted_aligner;
    tilted_aligner.setGravityDirection(tilted_gravity);
    tilted_aligner.printInfo();
    
    // 验证倾斜情况下的对齐
    Matrix3d tilted_rotation = tilted_aligner.getGravityAlignmentRotation();
    Vector3d tilted_transformed_z = tilted_rotation * Vector3d(0, 0, 1);
    Vector3d tilted_expected_z = -tilted_aligner.getGravityDirection();
    
    std::cout << "Tilted transformed Z-axis: " << tilted_transformed_z.transpose() << std::endl;
    std::cout << "Tilted expected direction: " << tilted_expected_z.transpose() << std::endl;
    std::cout << "Tilted alignment error: " << (tilted_transformed_z - tilted_expected_z).norm() << std::endl;
    
    // 测试3: 点变换
    std::cout << "\nTest 3: Point transformation" << std::endl;
    std::vector<Vector3d> test_points = {
        Vector3d(1.0, 0.0, 0.0),  // X轴单位向量
        Vector3d(0.0, 1.0, 0.0),  // Y轴单位向量
        Vector3d(0.0, 0.0, 1.0),  // Z轴单位向量
        Vector3d(1.0, 2.0, 3.0),  // 任意点
    };
    
    std::cout << "Point transformations with tilted gravity:" << std::endl;
    for (const auto& point : test_points) {
        Vector3d transformed = tilted_aligner.transformPoint(point);
        std::cout << "[" << point.transpose() << "] -> [" << transformed.transpose() << "]" << std::endl;
    }
    
    // 测试4: 旋转矩阵属性验证
    std::cout << "\nTest 4: Rotation matrix properties" << std::endl;
    std::cout << "Determinant (should be 1): " << tilted_rotation.determinant() << std::endl;
    Matrix3d orthogonality_check = tilted_rotation * tilted_rotation.transpose() - Matrix3d::Identity();
    std::cout << "Max orthogonality error: " << orthogonality_check.cwiseAbs().maxCoeff() << std::endl;
    
    std::cout << "\n=== All tests completed ===" << std::endl;
}

void simulateIMUGravityEstimation() {
    std::cout << "\n=== Simulating IMU Gravity Estimation ===" << std::endl;
    
    // 模拟IMU数据收集和重力估计过程
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 0.05); // 高斯噪声
    
    // 场景1: 水平放置的IMU
    std::cout << "\nScenario 1: Horizontal IMU" << std::endl;
    Vector3d true_gravity(0, 0, -9.81);
    Vector3d estimated_gravity = Vector3d::Zero();
    int sample_count = 0;
    
    // 模拟100个IMU样本
    for (int i = 0; i < 100; ++i) {
        Vector3d imu_measurement = true_gravity + Vector3d(noise(gen), noise(gen), noise(gen));
        
        // 增量平均计算（类似Fast-LIO2中的方法）
        sample_count++;
        estimated_gravity += (imu_measurement - estimated_gravity) / sample_count;
    }
    
    // 重力方向是加速度测量的反方向
    Vector3d estimated_gravity_direction = -estimated_gravity.normalized();
    
    std::cout << "True gravity direction: " << (-true_gravity.normalized()).transpose() << std::endl;
    std::cout << "Estimated gravity direction: " << estimated_gravity_direction.transpose() << std::endl;
    std::cout << "Estimation error: " << (estimated_gravity_direction - (-true_gravity.normalized())).norm() << std::endl;
    
    // 应用重力对齐
    SimpleGravityAlignment aligner;
    aligner.setGravityDirection(-estimated_gravity); // 注意符号
    
    std::cout << "Applying gravity alignment..." << std::endl;
    aligner.printInfo();
    
    // 场景2: 倾斜IMU
    std::cout << "\nScenario 2: Tilted IMU (45 degrees)" << std::endl;
    double tilt = 45.0 * M_PI / 180.0;
    Vector3d tilted_true_gravity(0, 9.81 * sin(tilt), -9.81 * cos(tilt));
    Vector3d tilted_estimated = Vector3d::Zero();
    sample_count = 0;
    
    for (int i = 0; i < 100; ++i) {
        Vector3d imu_measurement = tilted_true_gravity + Vector3d(noise(gen), noise(gen), noise(gen));
        sample_count++;
        tilted_estimated += (imu_measurement - tilted_estimated) / sample_count;
    }
    
    Vector3d tilted_estimated_direction = -tilted_estimated.normalized();
    Vector3d tilted_true_direction = -tilted_true_gravity.normalized();
    
    std::cout << "Tilted true gravity direction: " << tilted_true_direction.transpose() << std::endl;
    std::cout << "Tilted estimated gravity direction: " << tilted_estimated_direction.transpose() << std::endl;
    std::cout << "Tilted estimation error: " << (tilted_estimated_direction - tilted_true_direction).norm() << std::endl;
    
    SimpleGravityAlignment tilted_aligner;
    tilted_aligner.setGravityDirection(-tilted_estimated);
    
    // 验证对齐效果
    Matrix3d alignment_rotation = tilted_aligner.getGravityAlignmentRotation();
    Vector3d aligned_z = alignment_rotation * Vector3d(0, 0, 1);
    std::cout << "After alignment, Z-axis points to: " << aligned_z.transpose() << std::endl;
    std::cout << "Should be close to [0, 0, 1]: error = " << (aligned_z - Vector3d(0, 0, 1)).norm() << std::endl;
}

int main() {
    std::cout << "Starting Simple Gravity Alignment Tests..." << std::endl;
    
    // 运行基本功能测试
    testGravityAlignment();
    
    // 模拟IMU重力估计过程
    simulateIMUGravityEstimation();
    
    std::cout << "\nAll tests completed successfully!" << std::endl;
    return 0;
}