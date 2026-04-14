// Utils for data accessing

#pragma once

#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include "vtr_testing_common/utils.hpp"
#include "vtr_testing_common/math_utils.hpp"

namespace fs = std::filesystem;

namespace vtr {
namespace testing {

inline void load_lidar_groundtruth(const fs::path &path, std::vector<lgmath::se3::Transformation> &all_gt_poses, std::vector<Eigen::Vector<double, 6>> &all_gt_vels, const std::string &sensor="lidar") {
  std::ifstream ifs(path / "applanix" / (sensor + "_poses.csv"), std::ios::in);
  // Clear header line
  std::string line;
  std::getline(ifs, line);
  // Loop through all gt data
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::vector<double> gt;
    for (std::string str; std::getline(ss, str, ',');)
      gt.push_back(std::stod(str));

    // Store gt pose
    Eigen::Matrix4d T_ab_mat = Eigen::Matrix4d::Identity();
    T_ab_mat.block<3, 3>(0, 0) = rpy2rot(gt[7], gt[8], gt[9]);
    T_ab_mat.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];
    lgmath::se3::Transformation T_ab = lgmath::se3::Transformation(T_ab_mat);
    all_gt_poses.push_back(T_ab.inverse());

    // Store gt velocity
    Eigen::Vector<double, 3> vbar;
    vbar << gt[4], gt[5], gt[6];
    vbar = T_ab_mat.block<3, 3>(0, 0).transpose() * vbar;
    Eigen::Vector<double, 6> body_rate;
    body_rate << vbar[0], vbar[1], vbar[2], gt[12], gt[11], gt[10];
    all_gt_vels.push_back(-body_rate);
  }
}

inline void load_radar_groundtruth(const fs::path &path, std::vector<lgmath::se3::Transformation> &all_gt_poses, std::vector<Eigen::Vector<double, 6>> &all_gt_vels) {
  std::ifstream ifs(path / "applanix" / "radar_poses.csv", std::ios::in);
  // Clear header line
  std::string line;
  std::getline(ifs, line);
  // Loop through all gt data
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::vector<double> gt;
    for (std::string str; std::getline(ss, str, ',');)
      gt.push_back(std::stod(str));

    // Store gt pose
    Eigen::Matrix4d T_ab_mat = Eigen::Matrix4d::Identity();
    T_ab_mat.block<3, 3>(0, 0) = rpy2rot(roundToPi(gt[7]), roundToPi(gt[8]), gt[9]);
    T_ab_mat.block<3, 1>(0, 3) << gt[1], gt[2], 0.0;
    lgmath::se3::Transformation T_ab = lgmath::se3::Transformation(T_ab_mat);
    all_gt_poses.push_back(T_ab.inverse());

    // Store gt velocity
    Eigen::Vector<double, 3> vbar;
    vbar << gt[4], gt[5], gt[6];
    vbar = T_ab_mat.block<3, 3>(0, 0).transpose() * vbar;
    Eigen::Vector<double, 6> body_rate;
    body_rate << vbar[0], vbar[1], 0.0, 0.0, 0.0, gt[10];
    all_gt_vels.push_back(-body_rate);
  }
}

inline std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path) {
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  uint float_offset = 4;
  uint fields = 6;  // x, y, z, i, r, t
  uint point_step = float_offset * fields;
  uint N = floor(buffer.size() / point_step);
  Eigen::MatrixXd pc(Eigen::MatrixXd::Ones(N, fields));
  for (uint i = 0; i < N; ++i) {
    uint bufpos = i * point_step;
    for (uint j = 0; j < fields; ++j) {
      pc(i, j) =
          getFloatFromByteArray(buffer.data(), bufpos + j * float_offset);
    }
  }
  // Add offset to timestamps
  const auto timestamp = getStampFromPath(path);
  double t = double(timestamp / 1000) * 1.0e-6;
  pc.block(0, 5, N, 1).array() += t;

  return std::make_pair(timestamp, std::move(pc));
}

inline std::pair<int64_t, Eigen::MatrixXd> load_aeva(const std::string &path) {
  // load Aeries II pointcloud
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4; // float32
  // 9 point cloud fields, use 10 because point_flags is 64 bits
  unsigned fields = 10;  // x, y, z, radial velocity, intensity, signal quality, reflectivity, time, point_flags (beam_id, line_id, face_id)
  unsigned point_step = float_offset * fields;
  unsigned N = floor(buffer.size() / point_step);

  std::vector<Eigen::VectorXd> points; // Vector to store valid points dynamically

  const auto timestamp = getStampFromPath(path);
  double t = double(timestamp);

  for (uint i = 0; i < N; i++) {
    int bufpos = i * point_step;
    int offset = 0;

    // Temporary variables
    double x, y, z, radial_velocity, intensity, time_temp;
    uint64_t point_flags;
    int line_id, face_id;

    // x, y, z
    x = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    y = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    z = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);

    ++offset;
    // Radial velocity
    radial_velocity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // Intensity
    intensity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // Skip signal quality
    ++offset;
    // Skip reflectivity
    ++offset;
    // Timestamp
    time_temp = (int64_t)(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset)); // nanosec
    time_temp += t;
    ++offset;
    // Point Flags (64 bit flag, only need first 32 bits)
    point_flags = int(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset));

    // Extract flags
    line_id = ((point_flags >> 8) & 0xFF);
    face_id = ((point_flags >> 22) & 0xF);

    // Error checks
    if (line_id < 0 || line_id >= 64) continue;
    if (face_id < 0 || face_id > 5) continue;

    Eigen::VectorXd point(8);
    point << x, y, z, radial_velocity, intensity, time_temp, line_id, face_id;
    points.push_back(point);
  }

  // Convert vector to Eigen::MatrixXd
  Eigen::MatrixXd pc(points.size(), 8);
  for (size_t k = 0; k < points.size(); ++k) {
    pc.row(k) = points[k];
  }
  return std::make_pair(timestamp, pc);
}

struct IMUMeasurement {
  // custom to prevent precision loss
  int64_t timestamp_ns;
  long double angvel_x;
  long double angvel_y;
  long double angvel_z;
};

inline void load_all_imu_meas(const fs::path &imu_meas_file, std::vector<IMUMeasurement> &all_imu_meas, fs::path imu_file_name) {
  // Confirm file exists
  if (!fs::exists(imu_meas_file)) {
    CLOG(ERROR, "boreas_wrapper") << "IMU measurement file does not exist: " << imu_meas_file;
    throw std::invalid_argument("File not found: " + imu_meas_file.string());
  }
  
  std::ifstream imu_stream(imu_meas_file, std::ios::in);

  // Get rid of header
  std::string header;
  std::getline(imu_stream, header);

  // Loop over all imu measurements
  std::string imu_meas;

  while (std::getline(imu_stream, imu_meas)) {
    std::stringstream ss(imu_meas);
    std::string token;

    // Load timestamp separately to preserve precision
    std::getline(ss, token, ',');  // token = timestamp string
    int64_t timestamp_ns = stringToNanoseconds(token);
    // Load rest of the IMU data
    std::vector<long double> imu;
    while (std::getline(ss, token, ',')) {
      imu.push_back(std::stod(token));
    }

    IMUMeasurement meas;
    if (imu_file_name == "imu.csv" || imu_file_name == "imu_raw.csv") {
      // [angvel_z, angvel_y, angvel_x, accelz, accely, accelx]
      meas.timestamp_ns = static_cast<int64_t>(timestamp_ns);
      meas.angvel_x = imu[2];
      meas.angvel_y = imu[1];
      meas.angvel_z = imu[0];
    } else if (imu_file_name == "dmu_imu.csv") {
      // [angvel_z, angvel_y, angvel_x, ..., ..., ..., ..., angvel_x, angvel_y, angvel_z]
      meas.timestamp_ns = static_cast<int64_t>(timestamp_ns);
      meas.angvel_x = imu[0];
      meas.angvel_y = imu[1];
      meas.angvel_z = imu[2];
    } else if (imu_file_name == "aeva_imu.csv") {
      // [angvel_x, angvel_y, angvel_z, ..., ..., ..., ..., angvel_x, angvel_y, angvel_z]
      meas.timestamp_ns = static_cast<int64_t>(timestamp_ns);
      meas.angvel_x = imu[0];
      meas.angvel_y = imu[1];
      meas.angvel_z = imu[2];
    } else {
      // Unknown IMU file name
      CLOG(ERROR, "boreas_wrapper") << "Unknown IMU file name: " << imu_file_name;
      break;
    }

    all_imu_meas.push_back(meas);
  }
}

inline void load_wheel_encoder_data(const fs::path &path, const int encoder_max, std::vector<std::pair<int64_t, int64_t>> &all_wheel_meas) {
  std::ifstream wheel_stream(path / "applanix" / "dmi.csv", std::ios::in);
  // Get rid of header (GPSTime,pulse_count)
  std::string header;
  std::getline(wheel_stream, header);
  // Loop over all wheel measurements
  std::string curr_wheel_meas;
  std::vector<int64_t> timestamps;
  std::vector<int64_t> pulse_counts;
  while (std::getline(wheel_stream, curr_wheel_meas)) {
    std::stringstream ss(curr_wheel_meas);
    std::string time_str, pulse_str;
    
    std::getline(ss, time_str, ',');
    std::getline(ss, pulse_str, ',');

    int64_t timestamp = stringToNanoseconds(time_str);
    int64_t pulse_count = std::stoll(pulse_str);

    timestamps.push_back(timestamp);
    pulse_counts.push_back(pulse_count);
  }

  // Handle encoder wrap-around
  int64_t offset = 0;
  std::vector<int64_t> adjusted_counts = pulse_counts;
  for (size_t i = 1; i < pulse_counts.size(); ++i) {
    int64_t diff_count = std::abs(pulse_counts[i] - pulse_counts[i - 1]);
    if (diff_count > (encoder_max / 2)) {
      offset += encoder_max; // forward roll over
    }
    adjusted_counts[i] += offset;
  }

  all_wheel_meas.reserve(timestamps.size());
  for (size_t i = 0; i < timestamps.size(); ++i) {
    all_wheel_meas.emplace_back(std::pair(timestamps[i], adjusted_counts[i]));
  }
}

} // namespace testing
} // namespace vtr