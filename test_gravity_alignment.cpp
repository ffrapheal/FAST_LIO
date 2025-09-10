#include "gravity_alignment.hpp"
#include <iostream>
#include <vector>
#include <random>

void testGravityAlignment() {
    std::cout << "=== Testing Gravity Alignment Functionality ===" << std::endl;
    
    GravityAlignment aligner;
    
    // 测试1: 设置已知重力方向
    std::cout << "\nTest 1: Setting known gravity direction" << std::endl;
    Vector3d known_gravity(0.1, 0.2, -9.8); // 轻微倾斜的重力
    aligner.setGravityDirection(known_gravity);
    aligner.printInfo();
    
    // 测试2: 验证坐标变换
    std::cout << "\nTest 2: Coordinate transformation validation" << std::endl;
    
    // 原始坐标系的一些测试点
    std::vector<Vector3d> test_points = {
        Vector3d(1.0, 0.0, 0.0),  // X轴单位向量
        Vector3d(0.0, 1.0, 0.0),  // Y轴单位向量
        Vector3d(0.0, 0.0, 1.0),  // Z轴单位向量
        Vector3d(1.0, 2.0, 3.0),  // 任意点
        Vector3d(-1.0, -2.0, -3.0) // 对称点
    };
    
    std::cout << "Original -> Transformed points:" << std::endl;
    for (const auto& point : test_points) {
        Vector3d transformed = aligner.transformPoint(point);
        std::cout << "[" << point.transpose() << "] -> [" << transformed.transpose() << "]" << std::endl;
    }
    
    // 测试3: 验证Z轴对齐
    std::cout << "\nTest 3: Z-axis alignment verification" << std::endl;
    Matrix3d rotation = aligner.getGravityAlignmentRotation();
    Vector3d original_z(0, 0, 1);
    Vector3d transformed_z = rotation * original_z;
    Vector3d expected_z = -aligner.getGravityDirection(); // Z轴应该指向重力反方向
    
    std::cout << "Original Z-axis: " << original_z.transpose() << std::endl;
    std::cout << "Transformed Z-axis: " << transformed_z.transpose() << std::endl;
    std::cout << "Expected direction (opposite to gravity): " << expected_z.transpose() << std::endl;
    std::cout << "Alignment error: " << (transformed_z - expected_z).norm() << std::endl;
    
    // 测试4: 旋转矩阵属性验证
    std::cout << "\nTest 4: Rotation matrix properties" << std::endl;
    std::cout << "Determinant (should be 1): " << rotation.determinant() << std::endl;
    std::cout << "Orthogonality check (R * R^T - I, should be near zero):" << std::endl;
    Matrix3d orthogonality_check = rotation * rotation.transpose() - Matrix3d::Identity();
    std::cout << orthogonality_check << std::endl;
    std::cout << "Max orthogonality error: " << orthogonality_check.cwiseAbs().maxCoeff() << std::endl;
    
    // 测试5: 变换矩阵
    std::cout << "\nTest 5: 4x4 transformation matrix" << std::endl;
    Vector3d translation(10.0, 20.0, 30.0);
    Matrix4d transform = aligner.getGravityAlignmentTransform(translation);
    std::cout << "4x4 Transformation matrix with translation [10, 20, 30]:" << std::endl;
    std::cout << transform << std::endl;
    
    // 测试6: 位姿变换
    std::cout << "\nTest 6: Pose transformation" << std::endl;
    Vector3d original_position(5.0, 10.0, 15.0);
    Matrix3d original_orientation = Matrix3d::Identity(); // 单位姿态
    
    Vector3d transformed_position;
    Matrix3d transformed_orientation;
    
    aligner.transformPose(original_position, original_orientation, 
                         transformed_position, transformed_orientation);
    
    std::cout << "Original position: " << original_position.transpose() << std::endl;
    std::cout << "Transformed position: " << transformed_position.transpose() << std::endl;
    std::cout << "Original orientation (Identity):" << std::endl << original_orientation << std::endl;
    std::cout << "Transformed orientation:" << std::endl << transformed_orientation << std::endl;
    
    std::cout << "\n=== All tests completed ===" << std::endl;
}

void testIMUGravityEstimation() {
    std::cout << "\n=== Testing IMU Gravity Estimation ===" << std::endl;
    
    GravityAlignment aligner;
    
    // 模拟不同场景的IMU数据
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 0.05); // 高斯噪声，标准差0.05 m/s²
    
    // 场景1: 水平放置的IMU (重力向下)
    std::cout << "\nScenario 1: Horizontal IMU (gravity downward)" << std::endl;
    aligner = GravityAlignment(); // 重置
    
    for (int i = 0; i < 150; ++i) {
        sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
        imu_msg->linear_acceleration.x = noise(gen);
        imu_msg->linear_acceleration.y = noise(gen);
        imu_msg->linear_acceleration.z = -9.81 + noise(gen);
        
        aligner.addIMUSample(imu_msg);
    }
    
    if (aligner.estimateGravityDirection()) {
        aligner.printInfo();
        Vector3d gravity_dir = aligner.getGravityDirection();
        Vector3d expected_dir(0, 0, 1); // 期望重力向下
        std::cout << "Expected gravity direction: " << expected_dir.transpose() << std::endl;
        std::cout << "Estimation error: " << (gravity_dir - expected_dir).norm() << std::endl;
    }
    
    // 场景2: 倾斜放置的IMU
    std::cout << "\nScenario 2: Tilted IMU" << std::endl;
    aligner = GravityAlignment(); // 重置
    
    // 假设IMU绕X轴旋转30度
    double tilt_angle = 30.0 * M_PI / 180.0;
    Vector3d tilted_gravity(0, 9.81 * sin(tilt_angle), -9.81 * cos(tilt_angle));
    
    for (int i = 0; i < 150; ++i) {
        sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
        imu_msg->linear_acceleration.x = tilted_gravity.x() + noise(gen);
        imu_msg->linear_acceleration.y = tilted_gravity.y() + noise(gen);
        imu_msg->linear_acceleration.z = tilted_gravity.z() + noise(gen);
        
        aligner.addIMUSample(imu_msg);
    }
    
    if (aligner.estimateGravityDirection()) {
        aligner.printInfo();
        Vector3d gravity_dir = aligner.getGravityDirection();
        Vector3d expected_dir = tilted_gravity.normalized();
        std::cout << "Expected gravity direction: " << expected_dir.transpose() << std::endl;
        std::cout << "Estimation error: " << (gravity_dir - expected_dir).norm() << std::endl;
        
        // 测试对齐后的Z轴
        Matrix3d rotation = aligner.getGravityAlignmentRotation();
        Vector3d aligned_z = rotation * Vector3d(0, 0, 1);
        Vector3d expected_aligned_z = -expected_dir;
        std::cout << "Aligned Z-axis: " << aligned_z.transpose() << std::endl;
        std::cout << "Expected aligned Z-axis: " << expected_aligned_z.transpose() << std::endl;
        std::cout << "Z-axis alignment error: " << (aligned_z - expected_aligned_z).norm() << std::endl;
    }
    
    std::cout << "\n=== IMU estimation tests completed ===" << std::endl;
}

int main() {
    std::cout << "Starting Gravity Alignment Tests..." << std::endl;
    
    // 运行基本功能测试
    testGravityAlignment();
    
    // 运行IMU估计测试
    testIMUGravityEstimation();
    
    std::cout << "\nAll tests completed successfully!" << std::endl;
    return 0;
}