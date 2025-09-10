# Gravity Alignment for Fast-LIO2

这个模块提供了从IMU数据中提取重力方向并进行坐标系对齐的功能，使得z轴指向重力的反方向（与底面平行）。

## 功能概述

- **重力方向估计**: 从静态IMU数据估计重力方向
- **坐标系对齐**: 将当前坐标系转换为重力对齐坐标系
- **Fast-LIO2集成**: 直接从Fast-LIO2的状态中获取重力信息
- **实时变换**: 对位姿、点云等数据进行实时坐标变换

## 文件结构

```
├── gravity_alignment.hpp              # 核心重力对齐类
├── fastlio_gravity_integration.hpp    # Fast-LIO2集成接口
├── gravity_alignment_example.cpp      # ROS节点使用示例
├── test_gravity_alignment.cpp         # 功能测试程序
└── GRAVITY_ALIGNMENT_README.md        # 使用说明
```

## 核心类: GravityAlignment

### 主要功能

1. **重力方向估计**
   ```cpp
   GravityAlignment aligner;
   
   // 方法1: 从IMU数据估计
   for (auto& imu_msg : imu_data) {
       aligner.addIMUSample(imu_msg);
   }
   bool success = aligner.estimateGravityDirection(100); // 最少100个样本
   
   // 方法2: 直接设置已知重力方向
   Vector3d gravity_dir(0.1, 0.2, -9.8);
   aligner.setGravityDirection(gravity_dir);
   
   // 方法3: 从Fast-LIO2状态设置
   aligner.setGravityFromFastLIO(fastlio_gravity_vector);
   ```

2. **坐标变换**
   ```cpp
   // 获取旋转矩阵
   Matrix3d rotation = aligner.getGravityAlignmentRotation();
   
   // 变换单个点
   Vector3d transformed_point = aligner.transformPoint(original_point);
   
   // 变换位姿
   Vector3d aligned_pos, aligned_rot;
   aligner.transformPose(original_pos, original_rot, aligned_pos, aligned_rot);
   
   // 变换点云
   std::vector<PointType> aligned_cloud;
   aligner.transformPoints(original_cloud, aligned_cloud);
   ```

## 与Fast-LIO2集成

### 方法1: 修改laserMapping.cpp

在`laserMapping.cpp`中添加以下代码：

```cpp
#include "fastlio_gravity_integration.hpp"

// 在main函数中添加
FastLIOGravityIntegration gravity_integration(nh);

// 在主循环中，状态更新后添加
if (flg_EKF_inited) {
    gravity_integration.updateGravityFromFastLIO(state_point, Measures.lidar_beg_time);
    
    // 获取对齐后的位姿
    Vector3d aligned_pos;
    Matrix3d aligned_rot;
    gravity_integration.transformFastLIOPose(
        Vector3d(state_point.pos(0), state_point.pos(1), state_point.pos(2)),
        state_point.rot.toRotationMatrix(),
        aligned_pos, aligned_rot
    );
    
    // 发布对齐后的里程计...
}
```

### 方法2: 独立ROS节点

运行独立的重力对齐节点：

```bash
rosrun fast_lio gravity_alignment_example
```

该节点会：
- 订阅Fast-LIO2的里程计输出 (`/Odometry`)
- 发布重力对齐后的里程计 (`/aligned_odometry`)
- 发布重力方向信息 (`/gravity_direction`)

## 编译说明

### 作为Fast-LIO2的一部分编译

1. 将头文件复制到Fast-LIO2项目目录
2. 在Fast-LIO2的`CMakeLists.txt`中添加包含路径
3. 修改`laserMapping.cpp`集成重力对齐功能

### 独立编译测试

```bash
# 编译测试程序
g++ -std=c++17 -O3 test_gravity_alignment.cpp -o test_gravity_alignment \
    -I/usr/include/eigen3 -lm

# 运行测试
./test_gravity_alignment
```

### ROS环境编译

```bash
# 在catkin工作空间中
catkin_make

# 运行示例节点
rosrun fast_lio gravity_alignment_example
```

## 使用示例

### 基本使用

```cpp
#include "gravity_alignment.hpp"

int main() {
    GravityAlignment aligner;
    
    // 设置重力方向（例如从Fast-LIO2获取）
    Vector3d gravity(0.1, 0.2, -9.8);
    aligner.setGravityFromFastLIO(gravity);
    
    // 变换点
    Vector3d original_point(1.0, 2.0, 3.0);
    Vector3d aligned_point = aligner.transformPoint(original_point);
    
    std::cout << "Original: " << original_point.transpose() << std::endl;
    std::cout << "Aligned:  " << aligned_point.transpose() << std::endl;
    
    return 0;
}
```

### 点云变换示例

```cpp
// 假设有PCL点云
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
std::vector<pcl::PointXYZI> aligned_points;

// 应用重力对齐变换
aligner.transformPoints(cloud->points, aligned_points);

// 创建对齐后的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
aligned_cloud->points = aligned_points;
aligned_cloud->width = aligned_points.size();
aligned_cloud->height = 1;
aligned_cloud->is_dense = true;
```

## 重要参数

### GravityAlignment类参数

- `max_samples_`: 最大IMU样本数量 (默认: 200)
- `max_variance_threshold_`: IMU静止检测的方差阈值 (默认: 2.0)
- `G_magnitude_`: 重力加速度大小 (默认: 9.81 m/s²)

### 调整参数

```cpp
aligner.setMaxSamples(300);           // 增加样本数量
aligner.setVarianceThreshold(1.0);    // 更严格的静止检测
```

## 坐标系说明

### 变换前后的坐标系

- **变换前**: Fast-LIO2的世界坐标系（重力方向任意）
- **变换后**: 重力对齐坐标系（Z轴指向重力反方向，即向上）

### 变换矩阵

变换矩阵将原始坐标系的Z轴旋转到重力的反方向：

```
P_aligned = R_gravity * P_original
```

其中`R_gravity`是重力对齐旋转矩阵。

## 注意事项

1. **IMU静止假设**: 重力估计需要IMU在初始化期间保持相对静止
2. **坐标系一致性**: 确保Fast-LIO2和重力对齐使用相同的坐标系定义
3. **精度考虑**: 重力估计的精度取决于IMU的噪声水平和样本数量
4. **实时性**: 坐标变换计算量较小，可以实时进行

## 故障排除

### 常见问题

1. **重力估计失败**
   - 检查IMU是否静止（方差过大）
   - 增加样本数量
   - 检查IMU数据是否正常

2. **变换结果异常**
   - 验证重力方向是否正确估计
   - 检查坐标系定义是否一致
   - 确认变换矩阵的正交性

3. **ROS集成问题**
   - 检查话题名称是否正确
   - 确认消息类型匹配
   - 验证时间戳同步

### 调试信息

启用详细输出：
```cpp
aligner.printInfo();  // 打印重力对齐信息
```

## 性能优化

- 使用Eigen的SIMD优化
- 预分配内存避免动态分配
- 合理设置样本数量平衡精度和计算量

## 扩展功能

可以基于此模块扩展的功能：
- 自动水平校正
- 倾斜角度计算
- 重力异常检测
- 多传感器融合校正