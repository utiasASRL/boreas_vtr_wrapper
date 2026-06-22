#pragma once

#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

#include <lgmath/se3/TransformationWithCovariance.hpp>

namespace fs = std::filesystem;
using EdgeTransform = lgmath::se3::TransformationWithCovariance;

namespace vtr {
namespace testing {

inline Eigen::Matrix4d load_T_radar_applanix(const fs::path &path) {
    std::ifstream ifs1(path / "calib" / "T_applanix_lidar.txt", std::ios::in);
    std::ifstream ifs2(path / "calib" / "T_radar_lidar.txt", std::ios::in);

    if (!ifs1.is_open()) {
        CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_lidar.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_applanix_lidar.txt").string());
    }
    if (!ifs2.is_open()) {
        CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_radar_lidar.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_radar_lidar.txt").string());
    }

    Eigen::Matrix4d T_applanix_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_lidar_mat(row, col);

    Eigen::Matrix4d T_radar_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs2 >> T_radar_lidar_mat(row, col);

    return Eigen::Matrix4d(T_radar_lidar_mat * T_applanix_lidar_mat.inverse());
}

inline Eigen::Matrix4d load_T_wheel_applanix(const fs::path &path, bool aligned = false) {
    std::ifstream ifs(path / "calib" / "T_applanix_wheel.txt", std::ios::in);
    Eigen::Matrix4d T_applanix_wheel_mat;
    if (!ifs.is_open()) {
        CLOG(WARNING, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_wheel.txt. Loading default.";
        T_applanix_wheel_mat << 0.999560, 0.029665, 0.000000, -0.813993,
                               -0.029665, 0.999560, 0.000000, -0.455312,
                                0.000000, 0.000000, 1.000000, -1.610000,
                                0.000000, 0.000000, 0.000000, 1.000000;
    } else {
        for (size_t row = 0; row < 4; row++)
            for (size_t col = 0; col < 4; col++) ifs >> T_applanix_wheel_mat(row, col);
    }

    // This transform has y forward, x right, z up
    Eigen::Matrix4d T_wheel_applanix = T_applanix_wheel_mat.inverse();

    if (aligned) {
        // Rotate it so that x is forward to make it more intuitive
        Eigen::Matrix4d T_wheelfwd_wheel = Eigen::Matrix4d::Identity();
        T_wheelfwd_wheel.block<3, 3>(0, 0) << 0, 1, 0,
                                             -1, 0, 0,
                                              0, 0, 1;
        T_wheel_applanix = T_wheelfwd_wheel * T_wheel_applanix;
    }

    return T_wheel_applanix;
}

inline EdgeTransform load_T_robot_radar(const fs::path &path) {
    Eigen::Matrix4d T_radar_applanix = load_T_radar_applanix(path);

    // This transform has x forward, y left, z up
    Eigen::Matrix4d T_wheel_applanix = load_T_wheel_applanix(path, true);

    // Extrinsic from radar to rear wheel
    EdgeTransform T_robot_radar(Eigen::Matrix4d(T_wheel_applanix * T_radar_applanix.inverse()),
                        Eigen::Matrix<double, 6, 6>::Zero());

    return T_robot_radar;
}

inline EdgeTransform load_T_robot_lidar(const fs::path &path) {
    std::ifstream ifs(path / "calib" / "T_applanix_lidar.txt", std::ios::in);
    if (!ifs.is_open()) {
        CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_lidar.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_applanix_lidar.txt").string());
    }
    Eigen::Matrix4d T_applanix_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs >> T_applanix_lidar_mat(row, col);

    // This transform has x forward, y left, z up
    Eigen::Matrix4d T_wheel_applanix = load_T_wheel_applanix(path, true);

    // Extrinsic from lidar to rear wheel
    EdgeTransform T_robot_lidar(Eigen::Matrix4d(T_wheel_applanix * T_applanix_lidar_mat),
                                Eigen::Matrix<double, 6, 6>::Zero());

    return T_robot_lidar;
}

inline EdgeTransform load_T_robot_aeva(const fs::path &path) {
    std::ifstream ifs1(path / "calib" / "T_applanix_lidar.txt", std::ios::in);
    std::ifstream ifs2(path / "calib" / "T_aeva_lidar.txt", std::ios::in);

    Eigen::Matrix4d T_applanix_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_lidar_mat(row, col);

    Eigen::Matrix4d T_aeva_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs2 >> T_aeva_lidar_mat(row, col);

    Eigen::Matrix4d T_applanix_aeva = T_applanix_lidar_mat * T_aeva_lidar_mat.inverse();
    
    // Extrinsic from lidar to rear wheel
    // This transform has x forward, y left, z up
    Eigen::Matrix4d T_wheel_applanix = load_T_wheel_applanix(path, true);

    EdgeTransform T_robot_aeva(Eigen::Matrix4d(T_wheel_applanix * T_applanix_aeva),   // transform
                               Eigen::Matrix<double, 6, 6>::Zero());                  // covariance
    return T_robot_aeva;
}

inline EdgeTransform load_T_imu_robot(const fs::path &path, const std::string &imu_name) {
    // Extrinsic from applanix to rear wheel
    // This transform has x forward, y left, z up
    Eigen::Matrix4d T_wheel_applanix = load_T_wheel_applanix(path, true);

    EdgeTransform T_robot_imu;
    if (imu_name == "dmu") {
        std::ifstream ifs1(path / "calib" / "T_applanix_dmu.txt", std::ios::in);
        Eigen::Matrix4d T_applanix_dmu_mat;
    if (!ifs1.is_open()) {
        CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_dmu.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_applanix_dmu.txt").string());
    } else {
        for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_dmu_mat(row, col);
    }

    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_wheel_applanix * T_applanix_dmu_mat),
                                Eigen::Matrix<double, 6, 6>::Zero());
    } else if (imu_name == "aeva") {
    std::ifstream ifs1(path / "calib" / "T_applanix_lidar.txt", std::ios::in);
    Eigen::Matrix4d T_applanix_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_lidar_mat(row, col);

    std::ifstream ifs2(path / "calib" / "T_aeva_lidar.txt", std::ios::in);
    Eigen::Matrix4d T_aeva_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs2 >> T_aeva_lidar_mat(row, col);

    std::ifstream ifs3(path / "calib" / "T_imu_aeva.txt", std::ios::in);
    Eigen::Matrix4d T_imu_aeva_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs3 >> T_imu_aeva_mat(row, col);

    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_wheel_applanix * T_applanix_lidar_mat *
                                T_aeva_lidar_mat.inverse() * T_imu_aeva_mat.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());
    } else if (imu_name == "imu") {
    // Extrinsic from applanix to applanix IMU
    Eigen::Matrix4d T_imu_applanix;
    // Rotate applanix 90 degrees about z axis and then 180 degrees about y axis
    T_imu_applanix << 0, -1,  0,  0,
                     -1,  0,  0,  0,
                      0,  0, -1,  0,
                      0,  0,  0,  1;

    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_wheel_applanix * T_imu_applanix.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());
    } else {
    CLOG(ERROR, "boreas_wrapper") << "Unknown IMU name: " << imu_name;
    return EdgeTransform();
    }

    return T_robot_imu.inverse();
}

inline EdgeTransform load_T_wheel_robot(const fs::path &path) {
    Eigen::Matrix4d T_wheel_applanix = load_T_wheel_applanix(path, false);
    Eigen::Matrix4d T_robot_applanix = load_T_wheel_applanix(path, true);
    EdgeTransform T_wheel_robot(Eigen::Matrix4d(T_wheel_applanix * T_robot_applanix.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());
    return T_wheel_robot;
}

inline EdgeTransform load_T_enu_init(const fs::path &path, const bool &reverse, const std::string &sensor="lidar") {
    std::ifstream ifs(path / "applanix" / (sensor + "_poses.csv"), std::ios::in);

    if (!ifs.is_open()) {
        CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "applanix" / (sensor + "_poses.csv");
        throw std::invalid_argument("File not found: " + (path / "applanix" / (sensor + "_poses.csv")).string());
    }

    std::string header;
    std::getline(ifs, header);

    std::string first_pose;
    if (reverse) {
        std::string last_pose;
        // If reverse, we want to get last line
        while (std::getline(ifs, last_pose)) {
            first_pose = last_pose;
        }
    } else {
        std::getline(ifs, first_pose);
    }

    std::stringstream ss{first_pose};
    std::vector<double> gt;
    for (std::string str; std::getline(ss, str, ',');)
        gt.push_back(std::stod(str));

    Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
    T_mat.block<3, 3>(0, 0) = rpy2rot(gt[7], gt[8], gt[9]);
    T_mat.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];

    EdgeTransform T(T_mat);
    T.setZeroCovariance();

    return T;
}

} // namespace testing
} // namespace vtr