#pragma once

#include <memory>

#include "common/SlamResult.h"
#include "sensor/Imu.h"
#include "sensor/LaserScan.h"
#include "sensor/WheelOdometry.h"

namespace lidar_slam {

/**
 * SLAM对外接口
 */
class SlamInterface
{
public:
    SlamInterface();

    // LaserScan的频率计算Pose
    SlamResult GetPose(const std::shared_ptr<LaserScan>& laser_scan, const std::shared_ptr<Imu>& imu,
                       const std::shared_ptr<WheelOdometry>& wheel_odometry);

    // 通过轮式里程计、imu插值填补，计算Pose
    SlamResult GetPoseInterpolated(const std::shared_ptr<Imu>& imu,
                                   const std::shared_ptr<WheelOdometry>& wheel_odometry);

    // 通过imu插值填补，计算Pose
    SlamResult GetPoseInterpolatedByImu(const std::shared_ptr<Imu>& imu);

    // 通过轮式里程插值填补，计算Pose
    SlamResult GetPoseInterpolatedByWheelOdometry(const std::shared_ptr<WheelOdometry>& wheel_odometry);
};
}