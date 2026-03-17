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

#include "yaml-cpp/yaml.h"
#include "vtr_testing_aeva/utils.hpp"
#include <iomanip>
#include <fstream>

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::testing;

float getFloatFromByteArray(char *byteArray, uint index) {
  return *((float *)(byteArray + index));
}

int64_t getStampFromPath(const std::string &path) {
  std::vector<std::string> parts;
  boost::split(parts, path, boost::is_any_of("/"));
  std::string stem = parts[parts.size() - 1];
  boost::split(parts, stem, boost::is_any_of("."));
  int64_t time1 = std::stoll(parts[0]);
  return time1 * 1000;
}

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

std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path) {
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

Eigen::Matrix4d load_T_wheel_applanix(const fs::path &path, bool aligned = false) {
  std::ifstream ifs(path / "calib" / "T_applanix_wheel.txt", std::ios::in);
  if (!ifs.is_open()) {
    CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_wheel.txt";
    throw std::invalid_argument("File not found: " + (path / "calib" / "T_applanix_wheel.txt").string());
  }
  Eigen::Matrix4d T_applanix_wheel_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs >> T_applanix_wheel_mat(row, col);
  

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

EdgeTransform load_T_robot_aeva(const fs::path &path) {
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
                              Eigen::Matrix<double, 6, 6>::Zero());                 // covariance
  return T_robot_aeva;
}

EdgeTransform load_T_imu_robot(const fs::path &path, const std::string &imu_name) {
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
    EdgeTransform T_robot_aeva = load_T_robot_aeva(path);

    std::ifstream ifs(path / "calib" / "T_imu_aeva.txt", std::ios::in);
    Eigen::Matrix4d T_imu_aeva_mat;
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs >> T_imu_aeva_mat(row, col);
  
    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_robot_aeva.matrix() * T_imu_aeva_mat.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());
  } else if (imu_name == "imu") {
    // Extrinsic from applanix to applanix IMU
    Eigen::Matrix4d T_imu_applanix;
    // Rotate applanix 90 degrees about z axis and then 180 degrees about y axis
    T_imu_applanix <<  0, -1,  0,  0,
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

EdgeTransform load_T_wheel_robot(const fs::path &path) {
  Eigen::Matrix4d T_wheel_applanix = load_T_wheel_applanix(path, false);
  Eigen::Matrix4d T_robot_applanix = load_T_wheel_applanix(path, true);
  EdgeTransform T_wheel_robot(Eigen::Matrix4d(T_wheel_applanix * T_robot_applanix.inverse()),
                               Eigen::Matrix<double, 6, 6>::Zero());
  CLOG(WARNING, "boreas_wrapper") << "T_wheel_robot has been set to" << T_wheel_robot;
  return T_wheel_robot;
}

Eigen::Matrix3d toRoll(const double &r) {
  Eigen::Matrix3d roll;
  roll << 1, 0, 0, 0, cos(r), sin(r), 0, -sin(r), cos(r);
  return roll;
}

Eigen::Matrix3d toPitch(const double &p) {
  Eigen::Matrix3d pitch;
  pitch << cos(p), 0, -sin(p), 0, 1, 0, sin(p), 0, cos(p);
  return pitch;
}

Eigen::Matrix3d toYaw(const double &y) {
  Eigen::Matrix3d yaw;
  yaw << cos(y), sin(y), 0, -sin(y), cos(y), 0, 0, 0, 1;
  return yaw;
}

Eigen::Matrix3d rpy2rot(const double &r, const double &p, const double &y) {
  return toRoll(r) * toPitch(p) * toYaw(y);
}

EdgeTransform load_T_enu_lidar_init(const fs::path &path, const bool &reverse) {
  std::ifstream ifs(path / "applanix" / "aeva_poses.csv", std::ios::in);

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

void load_groundtruth(const fs::path &path, std::vector<lgmath::se3::Transformation> &all_gt_poses, std::vector<Eigen::Vector<double, 6>> &all_gt_vels) {
  std::ifstream ifs(path / "applanix" / "aeva_poses.csv", std::ios::in);

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

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  rclcpp::init(argc, argv);
  const std::string node_name = "boreas_localization_" + random_string(10);
  auto node = rclcpp::Node::make_shared(node_name);

  // odometry sequence directory
  const auto odo_dir_str =
      node->declare_parameter<std::string>("odo_dir", "/tmp");
  fs::path odo_dir{utils::expand_user(utils::expand_env(odo_dir_str))};

  // localization sequence directory
  const auto loc_dir_str =
      node->declare_parameter<std::string>("loc_dir", "/tmp");
  fs::path loc_dir{utils::expand_user(utils::expand_env(loc_dir_str))};

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
  CLOG(WARNING, "boreas_wrapper") << "Localization Directory: " << loc_dir.string();
  CLOG(WARNING, "boreas_wrapper") << "Output Directory: " << data_dir.string();

  std::vector<std::string> parts;
  boost::split(parts, loc_dir_str, boost::is_any_of("/"));
  auto stem = parts.back();
  boost::replace_all(stem, "-", "_");
  CLOG(WARNING, "boreas_wrapper") << "Publishing status to topic: "
                        << ("aeva_" + stem + "_lidar_localization");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      "aeva_" + stem + "_lidar_localization", 1);

  // Load IMU data
  const auto use_imu = node->declare_parameter<bool>("boreas.imu.use_imu", false);
  const auto imu_name = node->declare_parameter<std::string>("boreas.imu.imu_name", "aeva");
  CLOG(WARNING, "boreas_wrapper") << "IMU enabled: " << use_imu;
  std::vector<IMUMeasurement> all_imu_meas;
  EdgeTransform T_imu_robot; 
  if (use_imu) {
    // Check that imu name is one of "dmu", "aeva", "imu"
    CLOG(WARNING, "boreas_wrapper") << "IMU name: " << imu_name;
    if (imu_name != "dmu" && imu_name != "aeva" && imu_name != "imu") {
      CLOG(ERROR, "boreas_wrapper") << "Unknown IMU name: " << imu_name;
      return 1;
    }
    const auto imu_file_name = (imu_name == "imu") ? "imu_raw.csv" : (imu_name + "_imu.csv");
    const auto imu_path = loc_dir / "imu" / imu_file_name;
    if (!fs::exists(imu_path)) {
      CLOG(ERROR, "boreas_wrapper") << "IMU file does not exist: " << imu_path.string();
      return 1;
    }
    load_all_imu_meas(imu_path, all_imu_meas, imu_file_name);
    T_imu_robot = load_T_imu_robot(loc_dir, imu_name);
    CLOG(WARNING, "boreas_wrapper") << "Loaded " << all_imu_meas.size() << " IMU measurements";
    CLOG(WARNING, "boreas_wrapper") << "Transform from IMU to robot has been set to:\n" << T_imu_robot;
  }

  // Load wheel encoder data
  const auto use_wheel_encoder = node->declare_parameter<bool>("boreas.wheel_encoder.use_wheel_encoder", false);
  const auto encoder_max = node->declare_parameter<int>("boreas.wheel_encoder.encoder_max", 16777216);
  CLOG(WARNING, "boreas_wrapper") << "Wheel encoder enabled: " << use_wheel_encoder;
  std::vector<std::pair<int64_t, int64_t>> all_wheel_meas;
  EdgeTransform T_wheel_robot; 
  if (use_wheel_encoder) {
    load_wheel_encoder_data(loc_dir, encoder_max, all_wheel_meas);
    T_wheel_robot = load_T_wheel_robot(loc_dir);

    CLOG(WARNING, "boreas_wrapper") << "Loaded " << all_wheel_meas.size() << " wheel measurements";
    CLOG(WARNING, "boreas_wrapper") << "Transform from wheel to robot has been set to:\n" << T_wheel_robot;
  }

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

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
  tactic->setPipeline(PipelineMode::RepeatFollow);
  tactic->addRun();

  // Get the path that we should repeat
  VertexId::Vector sequence;
  sequence.reserve(graph->numberOfVertices());
  CLOG(WARNING, "boreas_wrapper") << "Total number of vertices: "
                        << graph->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  using LocEvaluator = tactic::PrivilegedEvaluator<tactic::GraphBase>;
  auto evaluator = std::make_shared<LocEvaluator>(*graph);
  auto privileged_path = graph->getSubgraph(0ul, evaluator);
  std::stringstream ss;

  // Load parameter about whether to run localization in reverse
  const auto reverse = node->declare_parameter<bool>("boreas.localization.reverse", false);
  for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
       ++it) {
    ss << it->v()->id() << " ";
    if (reverse) {
      sequence.insert(sequence.begin(), it->v()->id());
    }
    else {
      sequence.push_back(it->v()->id());
    }
  }

  CLOG(WARNING, "boreas_wrapper")<< "Test vertices: " << ss.str();
  if (reverse) CLOG(WARNING, "boreas_wrapper") << "Running localization in reverse";

  /// NOTE: odometry is teach, localization is repeat
  auto T_loc_odo_init = [&]() {
    const auto T_robot_aeva_odo = load_T_robot_aeva(odo_dir);
    const auto T_enu_aeva_odo = load_T_enu_lidar_init(odo_dir, reverse);

    const auto T_robot_aeva_loc = load_T_robot_aeva(loc_dir);
    const auto T_enu_aeva_loc = load_T_enu_lidar_init(loc_dir, false);

    return T_robot_aeva_loc * T_enu_aeva_loc.inverse() * T_enu_aeva_odo *
           T_robot_aeva_odo.inverse();
  }();
  T_loc_odo_init.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());
  CLOG(WARNING, "boreas_wrapper")
      << "Transform from localization to odometry has been set to "
      << T_loc_odo_init.vec().transpose();

  tactic->setPath(sequence, /* trunk sid */ 0, T_loc_odo_init, true);

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "lidar";
  
  const auto T_robot_aeva = load_T_robot_aeva(loc_dir);
  const auto T_aeva_robot = T_robot_aeva.inverse();
  CLOG(WARNING, "boreas_wrapper") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_aeva_robot;

  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg =
      tf2::eigenToTransform(Eigen::Affine3d(T_aeva_robot.inverse().matrix()));
  msg.header.frame_id = robot_frame;
  msg.child_frame_id = lidar_frame;
  tf_sbc->sendTransform(msg);

  const auto clock_publisher =
      node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // List of lidar data
  std::vector<fs::directory_entry> files;
  for (const auto &dir_entry : fs::directory_iterator{loc_dir / "aeva"})
    if (!fs::is_directory(dir_entry)) files.push_back(dir_entry);
  std::sort(files.begin(), files.end());
  CLOG(WARNING, "boreas_wrapper") << "Found " << files.size() << " aeva lidar data";
  const auto start_frame = node->declare_parameter<int>("odometry.start_frame", 0);
  const auto end_frame = node->declare_parameter<int>("odometry.end_frame", -1);

  // Load in groundtruth data
  const auto load_gt = node->declare_parameter<bool>("boreas.load_gt", true);
  CLOG(WARNING, "boreas_wrapper") << "Load groundtruth: " << load_gt;
  std::vector<lgmath::se3::Transformation> T_lid_world_gt;
  std::vector<Eigen::Vector<double, 6>> v_lid_gt;
  // Reserve space
  T_lid_world_gt.reserve(files.size());
  v_lid_gt.reserve(files.size());
  if (load_gt) {
    load_groundtruth(odo_dir, T_lid_world_gt, v_lid_gt);
    CLOG(WARNING, "boreas_wrapper") << "Loaded groundtruth for " << T_lid_world_gt.size() << " frames";
  }

  // thread handling variables
  TestControl test_control(node);

  // Store current gyro and gyro timestamps for the next loop
  static Eigen::MatrixXd prev_gyro = Eigen::MatrixXd();
  static std::vector<int64_t> prev_gyro_timestamps = std::vector<int64_t>();

  // main loop
  int frame = 0;
  int imu_counter = 0;
  int wheel_counter = 0;
  auto it = files.begin();
  const int64_t offset_ns = 100000000LL; 
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

    const auto [timestamp, points] = load_lidar(it->path().string());
    CLOG(WARNING, "boreas_wrapper") << "\033[95mLoading aeva frame " << frame
                                    << " with timestamp " << timestamp << "\033[0m";

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(timestamp);
    clock_publisher->publish(time_msg);

    // Feed in IMU data if available/desired
    std::vector<sensor_msgs::msg::Imu> gyro_msgs;
    if (use_imu) {
      int64_t start_timestamp = points(0, 5);
      int64_t end_timestamp = std::min(static_cast<int64_t>(points(points.rows() - 1, 5)), 
                                       timestamp + offset_ns);

      if (imu_counter == 0) {
        // Find IMU measurement right before lidar frame to initialize
        while (all_imu_meas[imu_counter].timestamp_ns < start_timestamp) {
          ++imu_counter;
        }
      }

      // Loop through all IMU measurements from previous one to end of current lidar frame
      // This captures IMU measurements that are between frames
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
      int64_t start_timestamp = points(0, 5);
      int64_t end_timestamp = std::min(static_cast<int64_t>(points(points.rows() - 1, 5)), 
                                       timestamp + offset_ns);

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
    query_data->stamp.emplace(timestamp);

    // make up some environment info (not important)
    tactic::EnvInfo env_info;
    env_info.terrain_type = 0;
    query_data->env_info.emplace(env_info);

    // set lidar frame
    query_data->points.emplace(std::move(points));

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_aeva_robot);

    // set gyro messages
    if (gyro_msgs.size() > 0) {
      query_data->T_s_r_gyro.emplace(T_imu_robot);
      query_data->gyro_msgs.emplace(gyro_msgs);
    }

    // set wheel encoder messages
    if (wheel_meas.size() > 0) {
      query_data->T_s_r_wheel.emplace(T_wheel_robot);
      query_data->wheel_meas.emplace(wheel_meas);
    }

    // set groundtruth if loaded
    if (load_gt && frame < T_lid_world_gt.size()) {
      query_data->T_s_world_gt.emplace(T_lid_world_gt[frame]);
      query_data->v_s_gt.emplace(v_lid_gt[frame]);
    }

    // execute the pipeline
    tactic->input(query_data);

    std_msgs::msg::String status_msg;
    status_msg.data = "Finished processing lidar frame " +
                      std::to_string(frame) + " with timestamp " +
                      std::to_string(timestamp);
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
