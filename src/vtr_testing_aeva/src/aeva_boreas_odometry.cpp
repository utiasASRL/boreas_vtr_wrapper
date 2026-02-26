#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/pipeline.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"

#include "vtr_testing_aeva/utils.hpp"

#include "yaml-cpp/yaml.h"
#include <iomanip>
#include <iostream>
#include <fstream>

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::testing;

int64_t stringToNanoseconds(const std::string &timestamp_str) {
  size_t dot_pos = timestamp_str.find('.');
  std::string sec_str = timestamp_str.substr(0, dot_pos);
  std::string frac_str = (dot_pos != std::string::npos) ? timestamp_str.substr(dot_pos + 1) : "0";

  if (dot_pos == std::string::npos) {
    switch (timestamp_str.length()) {
      case 16: // Aeva timestamp format
        return std::stoll(timestamp_str) * 1'000;
      case 19: // DMU timestamp format
        return std::stoll(timestamp_str);
      default:
        throw std::invalid_argument("Unexpected timestamp length: " + std::to_string(timestamp_str.length()));
    }
  }

  // Pad fractional part to exactly 9 digits (nanoseconds)
  while (frac_str.length() < 9) frac_str += '0';
  if (frac_str.length() > 9) frac_str = frac_str.substr(0, 9); // truncate

  int64_t seconds = std::stoll(sec_str);
  int64_t nanos = std::stoll(frac_str);
  return seconds * 1'000'000'000LL + nanos;
}

struct IMUMeasurement {
  // custom to prevent precision loss
  int64_t timestamp_ns;
  long double angvel_x;
  long double angvel_y;
  long double angvel_z;
};

int64_t getStampFromPath(const std::string &path) {
  std::vector<std::string> parts;
  boost::split(parts, path, boost::is_any_of("/"));
  std::string stem = parts[parts.size() - 1];
  boost::split(parts, stem, boost::is_any_of("."));
  int64_t time1 = std::stoll(parts[0]);
  return time1 * 1000;
}

std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path, double start_time, double end_time, int64_t filename) {
  // load Aeries II pointcloud
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4; // float32
  // 9 point cloud fields, use 10 because point_flags is 64 bits
  unsigned fields = 10;  // x, y, z, radial velocity, intensity, signal quality, reflectivity, time, point_flags (beam_id, line_id, face_id)
  unsigned point_step = float_offset * fields;
  unsigned N = floor(buffer.size() / point_step);

  std::vector<Eigen::VectorXd> points; // Vector to store valid points dynamically

  auto getFloatFromByteArray = [](char *byteArray, unsigned index) -> float {
    return *((float *)(byteArray + index));
  };

  for (unsigned i(0); i < N; i++) {
    int bufpos = i * point_step;
    int offset = 0;

    // Temporary variables
    double x, y, z, radial_velocity, intensity, time_temp;
    int64_t time_keep;
    uint64_t point_flags;
    int beam_id, line_id, face_id, sensor_id;

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
    double t = double(filename);
    time_keep = (int64_t)(time_temp + t);
    time_temp = time_temp * 1e-9 + start_time;
    ++offset;
    // Point Flags (64 bit flag, only need first 32 bits)
    point_flags = int(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset));

    // Extract flags
    line_id = ((point_flags >> 8) & 0xFF);
    beam_id = ((point_flags >> 16) & 0xF);
    face_id = ((point_flags >> 22) & 0xF);

    // Error checks
    if (line_id < 0 || line_id >= 64) continue;
    if (face_id < 0 || face_id > 5) continue;

    // Sensor id - only one sensor
    sensor_id = 0;

    // Include if within start and end time
    if (time_temp > start_time && time_temp <= end_time) {
        Eigen::VectorXd point(10);
        point << x, y, z, radial_velocity, intensity, time_keep, beam_id, line_id, face_id, sensor_id;
        points.push_back(point);
    }
  }

  // Convert vector to Eigen::MatrixXd
  Eigen::MatrixXd pc(points.size(), 10);
  for (size_t k = 0; k < points.size(); ++k) {
    pc.row(k) = points[k];
  }
  return std::make_pair(fields, pc);
}

EdgeTransform load_T_lidar_robot(const fs::path &path) {
  // Extrinsic from applanix to aeva
  std::ifstream ifs1(path / "calib" / "T_applanix_aeva.txt", std::ios::in);
  Eigen::Matrix4d T_applanix_aeva_mat;
  if (!ifs1.is_open()) {
    CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_aeva.txt. Loading preset.";
    T_applanix_aeva_mat << 
      0.0116474, -0.99998734, 0.0, -0.37043642,
      0.9999333,  0.0116284,  0.0,  0.39745466,
      0.0,        0.0,        1.0, -0.1032,
      0.0,        0.0,        0.0,  1.0;
  } else {
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_aeva_mat(row, col);
  }

  // Extrinsic from applanix to rear axel of vehicle
  std::ifstream ifs2(path / "calib" / "T_applanix_axel.txt", std::ios::in);
  Eigen::Matrix4d T_applanix_axel_mat;
  if (!ifs2.is_open()) {
    CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_axel.txt. Loading preset.";
    T_applanix_axel_mat << 
      0.0299955, -0.99955003, 0.0,  0.0,
      0.99955003, 0.0299955,  0.0, -0.51,
      0.0,        0.0,        1.0, -1.45,
      0.0,        0.0,        0.0,  1.0;
  } else {
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs2 >> T_applanix_axel_mat(row, col);
  }

  EdgeTransform T_lidar_robot(Eigen::Matrix4d((T_applanix_axel_mat.inverse() * T_applanix_aeva_mat).inverse()),   // transform
                              Eigen::Matrix<double, 6, 6>::Zero());                                               // covariance
  return T_lidar_robot;
}

bool filecomp (std::string file1, std::string file2) { 
  long long i = std::stoll(file1.substr(0, file1.find(".")));
  long long j = std::stoll(file2.substr(0, file2.find(".")));
  return (i<j); 
}

std::string getFirstFilename(const std::string& dir_path) {
    std::string first_filename;
    std::filesystem::directory_iterator dir_iter(dir_path);
    if (dir_iter != std::filesystem::directory_iterator()) {
        first_filename = dir_iter->path().filename().string();
    }
    return first_filename;
}

void load_all_imu_meas(const fs::path &imu_meas_file, std::vector<IMUMeasurement> &all_imu_meas, fs::path imu_file_name) {
  std::ifstream imu_stream(imu_meas_file, std::ios::in);
  // Get rid of header (GPSTime,angvel_z,angvel_y,angvel_x,accelz,accely,accelx)
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
      meas.timestamp_ns = static_cast<int64_t>(timestamp_ns);
      meas.angvel_x = imu[2];
      meas.angvel_y = imu[1];
      meas.angvel_z = imu[0];
    } else if (imu_file_name == "dmu_imu.csv") {
      meas.timestamp_ns = static_cast<int64_t>(timestamp_ns);
      meas.angvel_x = imu[0];
      meas.angvel_y = imu[1];
      meas.angvel_z = imu[2];
    } else if (imu_file_name == "aeva_imu.csv") {
      meas.timestamp_ns = static_cast<int64_t>(timestamp_ns);
      meas.angvel_x = imu[0];
      meas.angvel_y = imu[1];
      meas.angvel_z = imu[2];
    } else {
      CLOG(ERROR, "boreas_wrapper") << "Unknown IMU file name: " << imu_file_name;
      break;
    }

    all_imu_meas.push_back(meas);
  }
}

EdgeTransform load_T_imu_robot(const fs::path &path, const std::string &imu_name) {
  // Extrinsic from applanix to rear axel of the vehicle
  std::ifstream ifs_rob(path / "calib" / "T_applanix_axel.txt", std::ios::in);
  Eigen::Matrix4d T_applanix_axel_mat;
  if (!ifs_rob.is_open()) {
    CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_axel.txt. Loading preset.";
    T_applanix_axel_mat << 
      0.0299955, -0.99955003, 0.0,  0.0,
      0.99955003, 0.0299955,  0.0, -0.51,
      0.0,        0.0,        1.0, -1.45,
      0.0,        0.0,        0.0,  1.0;
  } else {
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs_rob >> T_applanix_axel_mat(row, col);
  }

  EdgeTransform T_robot_imu;
  if (imu_name == "dmu") {
    // Extrinsic from applanix to dmu
    std::ifstream ifs1(path / "calib" / "T_applanix_dmu.txt", std::ios::in);
    Eigen::Matrix4d T_applanix_dmu_mat;
    if (!ifs1.is_open()) {
      CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_dmu.txt. Loading preset.";
      T_applanix_dmu_mat << 
        1.0,  0.0,  0.0,  0.0,
        0.0, -1.0,  0.0,  0.0,
        0.0,  0.0, -1.0, -0.15,
        0.0,  0.0,  0.0,  1.0;
    } else {
      for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_dmu_mat(row, col);
    }
  
    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_applanix_axel_mat.inverse() * T_applanix_dmu_mat),
                                Eigen::Matrix<double, 6, 6>::Zero());

  } else if (imu_name == "aeva") {
    // Extrinsic from applanix to aeva
    std::ifstream ifs1(path / "calib" / "T_applanix_aeva.txt", std::ios::in);
    Eigen::Matrix4d T_applanix_aeva_mat;
    if (!ifs1.is_open()) {
      CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_aeva.txt. Loading preset.";
      T_applanix_aeva_mat << 
        0.0116474, -0.99998734, 0.0, -0.37043642,
        0.9999333,  0.0116284,  0.0,  0.39745466,
        0.0,        0.0,        1.0, -0.1032,
        0.0,        0.0,        0.0,  1.0;
    } else {
      for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_aeva_mat(row, col);
    }
    CLOG(WARNING, "boreas_wrapper") << "T_applanix_aeva_mat: " << T_applanix_aeva_mat;

    // Extrinsic from aeva imu to aeva
    std::ifstream ifs2(path / "calib" / "T_imu_aeva.txt", std::ios::in);
    Eigen::Matrix4d T_imu_aeva_mat;
    if (!ifs2.is_open()) {
      CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_imu_aeva.txt. Loading preset.";
      T_imu_aeva_mat << 
        1.0, 0.0, 0.0, -0.020,
        0.0, 1.0, 0.0, -0.023,
        0.0, 0.0, 1.0,  0.037,
        0.0, 0.0, 0.0,  1.0;
    } else {
      for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs2 >> T_imu_aeva_mat(row, col);
    }
  
    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_applanix_axel_mat.inverse() * T_applanix_aeva_mat * T_imu_aeva_mat.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());

  } else if (imu_name == "imu") {
    // Extrinsic from applanix to applanix IMU
    Eigen::Matrix4d T_imu_applanix;
    // Rotate applanix 90 degrees about z axis and then 180 degrees about y axis
    T_imu_applanix << 
       0.0, -1.0,  0.0, 0.0,
      -1.0,  0.0,  0.0, 0.0,
       0.0,  0.0, -1.0, 0.0,
       0.0,  0.0,  0.0, 1.0;

    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_applanix_axel_mat.inverse() * T_imu_applanix.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());
  } else {
    CLOG(ERROR, "boreas_wrapper") << "Unknown IMU name: " << imu_name;
    return EdgeTransform();
  }
  return T_robot_imu.inverse();
}

EdgeTransform load_T_wheel_robot(const fs::path &path) {
  // Extrinsic from applanix to wheel encoder (left rear wheel)
  std::ifstream ifs1(path / "calib" / "T_applanix_wheel.txt", std::ios::in);
  Eigen::Matrix4d T_applanix_wheel_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_wheel_mat(row, col);

  // Extrinsic from applanix to rear axel of the vehicle
  std::ifstream ifs2(path / "calib" / "T_applanix_axel.txt", std::ios::in);
  Eigen::Matrix4d T_applanix_axel_mat;
  if (!ifs2.is_open()) {
    CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_axel.txt. Loading preset.";
    T_applanix_axel_mat << 
      0.0299955, -0.99955003, 0.0,  0.0,
      0.99955003, 0.0299955,  0.0, -0.51,
      0.0,        0.0,        1.0, -1.45,
      0.0,        0.0,        0.0,  1.0;
  } else {
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs2 >> T_applanix_axel_mat(row, col);
  }  

  EdgeTransform T_wheel_robot(Eigen::Matrix4d((T_applanix_axel_mat.inverse() * T_applanix_wheel_mat).inverse()),
                              Eigen::Matrix<double, 6, 6>::Zero());
  
  return T_wheel_robot;
}

void load_wheel_encoder_data(const fs::path &path, const int encoder_max, std::vector<std::pair<int64_t, int64_t>> &all_wheel_meas) {
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

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  rclcpp::init(argc, argv);
  const std::string node_name = "boreas_odometry_" + random_string(10);
  auto node = rclcpp::Node::make_shared(node_name);

  // odometry sequence directory
  const auto odo_dir_str =
      node->declare_parameter<std::string>("odo_dir", "/tmp");
  fs::path odo_dir{utils::expand_user(utils::expand_env(odo_dir_str))};

  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Configure logging
  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled = node->declare_parameter<std::vector<std::string>>(
      "log_enabled", std::vector<std::string>{});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  CLOG(WARNING, "boreas_wrapper") << "Odometry Directory: " << odo_dir.string();
  CLOG(WARNING, "boreas_wrapper") << "Output Directory: " << data_dir.string();

  std::vector<std::string> parts;
  boost::split(parts, odo_dir_str, boost::is_any_of("/"));
  auto stem = parts.back();
  boost::replace_all(stem, "-", "_");
  CLOG(WARNING, "boreas_wrapper") << "Publishing status to topic: "
                        << ("aeva_" + stem + "_lidar_odometry");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      "aeva_" + stem + "_lidar_odometry", 1);

  // Load IMU data
  const auto use_imu = node->declare_parameter<bool>("boreas.imu.use_imu", false);
  const auto imu_name = node->declare_parameter<std::string>("boreas.imu.imu_name", "aeva");
  CLOG(WARNING, "boreas_wrapper") << "IMU enabled: " << use_imu;
  std::vector<IMUMeasurement> all_imu_meas;
  EdgeTransform T_imu_robot; 
  Eigen::Vector3d gyro_bias(0.0, 0.0, 0.0);
  if (use_imu) {
    // Check that imu name is one of "dmu", "aeva", "imu"
    CLOG(WARNING, "boreas_wrapper") << "IMU name: " << imu_name;
    if (imu_name != "dmu" && imu_name != "aeva" && imu_name != "imu") {
      CLOG(ERROR, "boreas_wrapper") << "Unknown IMU name: " << imu_name;
      return 1;
    }
    const auto imu_file_name = (imu_name == "imu") ? "imu_raw.csv" : (imu_name + "_imu.csv");
    const auto imu_path = odo_dir / "imu" / imu_file_name;
    load_all_imu_meas(imu_path, all_imu_meas, imu_file_name);
    T_imu_robot = load_T_imu_robot(odo_dir, imu_name);
    CLOG(WARNING, "boreas_wrapper") << "Loaded " << all_imu_meas.size() << " IMU measurements";
    CLOG(WARNING, "boreas_wrapper") << "Transform from IMU to robot has been set to:\n" << T_imu_robot;

    // Average the first 500 IMU messages to compute the gyro bias
    Eigen::Vector3d gyro_bias_sum(0.0, 0.0, 0.0);
    size_t imu_count = std::min(static_cast<size_t>(500), all_imu_meas.size());
    for (size_t i = 0; i < imu_count; ++i) {
      gyro_bias_sum(0) += all_imu_meas[i].angvel_x;
      gyro_bias_sum(1) += all_imu_meas[i].angvel_y;
      gyro_bias_sum(2) += all_imu_meas[i].angvel_z;
    }
    gyro_bias = gyro_bias_sum / imu_count;
    CLOG(WARNING, "boreas_wrapper") << "Computed gyro bias from first " << imu_count
                    << " IMU measurements: " << gyro_bias.transpose();
  }

  // Load wheel encoder data
  const auto use_wheel_encoder = node->declare_parameter<bool>("boreas.wheel_encoder.use_wheel_encoder", false);
  const auto encoder_max = node->declare_parameter<int>("boreas.wheel_encoder.encoder_max", 16777216);
  CLOG(WARNING, "boreas_wrapper") << "Wheel encoder enabled: " << use_wheel_encoder;
  std::vector<std::pair<int64_t, int64_t>> all_wheel_meas;
  double wheel_param = 0.0;
  EdgeTransform T_wheel_robot; 
  if (use_wheel_encoder) {
    load_wheel_encoder_data(odo_dir, encoder_max, all_wheel_meas);
    T_wheel_robot = load_T_wheel_robot(odo_dir);

    CLOG(WARNING, "boreas_wrapper") << "Loaded " << all_wheel_meas.size() << " wheel measurements";
    CLOG(WARNING, "boreas_wrapper") << "Transform from wheel to robot has been set to:\n" << T_wheel_robot;
  }

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), false);

  // Pipeline
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node);
  auto pipeline = pipeline_factory->get("pipeline");
  auto pipeline_output = pipeline->createOutputCache();
  // some modules require node for visualization
  pipeline_output->node = node;

  // Tactic Callback
  auto callback = std::make_shared<RvizTacticCallback>(node);

  // Tactic
  auto tactic =
      std::make_shared<Tactic>(Tactic::Config::fromROS(node), pipeline,
                               pipeline_output, graph, callback);
  tactic->setPipeline(PipelineMode::TeachBranch);
  tactic->addRun();

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "lidar";

  const auto T_lidar_robot = load_T_lidar_robot(odo_dir);
  CLOG(WARNING, "boreas_wrapper") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_lidar_robot;

  std::string dir_path_ = odo_dir.string() + "/aeva/";
  std::vector<std::string> filenames_;
  int64_t first_state_time;

  auto dir_iter = std::filesystem::directory_iterator(dir_path_);
  std::count_if(begin(dir_iter), end(dir_iter), [&filenames_](auto &entry) {
    if (entry.is_regular_file()) filenames_.emplace_back(entry.path().filename().string());
    return entry.is_regular_file();
  });
  std::sort(filenames_.begin(), filenames_.end(), filecomp);  // custom comparison

  first_state_time = std::stoll(filenames_[0].substr(0, filenames_[0].find("."))) * 1000; // convert to nanosec

  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg =
      tf2::eigenToTransform(Eigen::Affine3d(T_lidar_robot.inverse().matrix()));
  msg.header.frame_id = robot_frame;
  msg.child_frame_id = lidar_frame;
  tf_sbc->sendTransform(msg);

  const auto clock_publisher =
      node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // List of lidar data
  std::vector<fs::directory_entry> files;
  for (const auto &dir_entry : fs::directory_iterator{odo_dir / "aeva"})
    if (!fs::is_directory(dir_entry)) files.push_back(dir_entry);
  std::sort(files.begin(), files.end());
  CLOG(WARNING, "boreas_wrapper") << "Found " << files.size() << " lidar data";
  const auto start_frame = node->declare_parameter<int>("boreas.odometry.start_frame", 0);
  const auto end_frame = node->declare_parameter<int>("boreas.odometry.end_frame", -1);

  // thread handling variables
  TestControl test_control(node);

  // main loop
  int frame = 0;
  int imu_counter = 0;
  int wheel_counter = 0;
  auto it = files.begin();
  while (it != files.end()) {
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(test_control.delay()));

    if (frame < start_frame) {
      ++it;
      ++frame;
      continue;
    } else if (end_frame > 0 && frame > end_frame) {
      break;
    }

    const auto filename = getStampFromPath((it)->path().string());
    int64_t time_delta_micro = filename - first_state_time;
    double start_time = static_cast<double>(time_delta_micro) / 1e9;

    CLOG(WARNING, "boreas_wrapper") << "\033[95mLoading aeva frame " << frame
                                    << " with timestamp " << filename << "\033[0m";
    
    // Note: we peak into future data for the end timestamp for evaluation convenience. An online implementation
    // would need different logic, i.e., use the last timestamp of the pointcloud
    double end_time = start_time + 0.1;
    // Get the name of the next file
    int64_t next_state_time;
    if ((it + 1) != files.end()) {
      next_state_time = getStampFromPath((it + 1)->path().string());
      auto end_time = static_cast<double>(next_state_time - first_state_time) / 1e9;
    }
    int64_t start_name = filename;
    
    Eigen::MatrixXd points;
    // load Aeries II boreas pointcloud
    std::tie(std::ignore, points) = load_lidar(it->path().string(), start_time, end_time, start_name);

    if (points.rows() == 0) {
      CLOG(WARNING, "boreas_wrapper") << "No points found in frame " << frame;
      ++it;
      ++frame;
      continue;
    }

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(start_name);
    clock_publisher->publish(time_msg);

    // Feed in IMU data if available/desired
    std::vector<sensor_msgs::msg::Imu> gyro_msgs;
    if (use_imu) {
      int64_t timestamp_imu = all_imu_meas[imu_counter].timestamp_ns;
      int64_t start_timestamp = points(0, 5);
      int64_t end_timestamp = points(points.rows() - 1, 5);

      if (imu_counter == 0) {
        // Find IMU measurement right before lidar frame to initialize
        while (all_imu_meas[imu_counter].timestamp_ns < start_timestamp) {
          ++imu_counter;
        }
      }

      // Loop through all IMU measurements from previous one to end of current lidar frame
      // This captures IMU measurements that are between frames
      Eigen::Matrix<double, 4, 1> imu_meas;
      while (imu_counter < all_imu_meas.size() && all_imu_meas[imu_counter].timestamp_ns < end_timestamp) {
        auto gyro_msg = sensor_msgs::msg::Imu();
        gyro_msg.angular_velocity.x = all_imu_meas[imu_counter].angvel_x;
        gyro_msg.angular_velocity.y = all_imu_meas[imu_counter].angvel_y;
        gyro_msg.angular_velocity.z = all_imu_meas[imu_counter].angvel_z;
        gyro_msg.header.stamp = rclcpp::Time(all_imu_meas[imu_counter].timestamp_ns);
        gyro_msgs.push_back(gyro_msg);
        ++imu_counter;
      }
      CLOG(WARNING, "boreas_wrapper") << "Loaded " << gyro_msgs.size() << " IMU measurements";
    }

    // Feed in wheel encoder data if available/desired
    std::vector<std::pair<rclcpp::Time, double>> wheel_meas;
    if (use_wheel_encoder) {
      int64_t timestamp_wheel = all_wheel_meas[wheel_counter].first;
      int64_t start_timestamp = points(0, 5);
      int64_t end_timestamp = points(points.rows() - 1, 5);

      if (wheel_counter == 0) {
        // Find wheel measurement right before lidar frame to initialize
        while (all_wheel_meas[wheel_counter].first < start_timestamp) {
          ++wheel_counter;
        }
      }

      // Loop through all wheel measurements from previous one to end of current lidar frame
      // This captures wheel measurements that are between frames
      while (wheel_counter < all_wheel_meas.size() && all_wheel_meas[wheel_counter].first < end_timestamp) {
        std::pair<rclcpp::Time, double> wheel_msg = {
          rclcpp::Time(all_wheel_meas[wheel_counter].first),             // timestamp
          static_cast<double>(all_wheel_meas[wheel_counter].second)      // pulse count
        };
        wheel_meas.push_back(wheel_msg);
        ++wheel_counter;
      }
      CLOG(WARNING, "boreas_wrapper") << "Loaded " << wheel_meas.size() << " wheel measurements";
    }

    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<lidar::LidarQueryCache>();

    // some modules require node for visualization
    query_data->node = node;

    // set timestamp
    query_data->stamp.emplace(start_name);

    // make up some environment info (not important)
    tactic::EnvInfo env_info;
    env_info.terrain_type = 0;
    query_data->env_info.emplace(env_info);

    // set lidar frame
    query_data->points.emplace(std::move(points));

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_lidar_robot);

    // set gyro messages
    if (gyro_msgs.size() > 0) {
      query_data->T_s_r_gyro.emplace(T_imu_robot);
      query_data->gyro_msgs.emplace(gyro_msgs);
      // query_data->gyro_bias.emplace(gyro_bias);
    }

    // set wheel encoder messages
    if (wheel_meas.size() > 0) {
      query_data->T_s_r_wheel.emplace(T_wheel_robot);
      query_data->wheel_meas.emplace(wheel_meas);
    }

    // execute the pipeline
    tactic->input(query_data);

    std_msgs::msg::String status_msg;
    status_msg.data = "Finished processing lidar frame " +
                      std::to_string(frame) + " with timestamp " +
                      std::to_string(start_name);
    status_publisher->publish(status_msg);

    ++it;
    ++frame;
  }

  rclcpp::shutdown();

  tactic.reset();
  callback.reset();
  pipeline.reset();
  pipeline_factory.reset();
  CLOG(WARNING, "boreas_wrapper") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "boreas_wrapper") << "Saving pose graph and reset. - DONE!";
}