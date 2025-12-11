#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_radar/pipeline.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"
#include <lgmath/se3/Transformation.hpp>

#include "vtr_testing_radar/utils.hpp"

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::testing;

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

Eigen::Matrix4d load_T_radar_applanix(const fs::path &path) {
  std::ifstream ifs1(path / "calib" / "T_applanix_lidar.txt", std::ios::in);
  std::ifstream ifs2(path / "calib" / "T_radar_lidar.txt", std::ios::in);

  Eigen::Matrix4d T_applanix_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_lidar_mat(row, col);

  Eigen::Matrix4d T_radar_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs2 >> T_radar_lidar_mat(row, col);

  return Eigen::Matrix4d(T_radar_lidar_mat * T_applanix_lidar_mat.inverse());
}

EdgeTransform load_T_robot_radar(const fs::path &path) {
  Eigen::Matrix4d identity_matrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> zero_cov = Eigen::Matrix<double, 6, 6>::Zero();
  EdgeTransform T_robot_radar(identity_matrix, zero_cov);

  return T_robot_radar;
}

EdgeTransform load_T_imu_robot(const fs::path &path, const std::string &imu_name) {
  EdgeTransform T_robot_imu;
  // Robot frame is just the radar frame
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

    Eigen::Matrix4d T_radar_applanix = load_T_radar_applanix(path);

    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_radar_applanix * T_applanix_dmu_mat),
                                Eigen::Matrix<double, 6, 6>::Zero());

  } else if (imu_name == "aeva") {
    std::ifstream ifs1(path / "calib" / "T_applanix_aeva.txt", std::ios::in);
    Eigen::Matrix4d T_applanix_aeva_mat;
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_aeva_mat(row, col);

    std::ifstream ifs2(path / "calib" / "T_imu_aeva.txt", std::ios::in);
    Eigen::Matrix4d T_imu_aeva_mat;
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs2 >> T_imu_aeva_mat(row, col);

    Eigen::Matrix4d T_radar_applanix = load_T_radar_applanix(path);
  
    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_radar_applanix * T_applanix_aeva_mat *
                                T_imu_aeva_mat.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());
  } else if (imu_name == "imu") {
    // Extrinsic from applanix to applanix IMU
    Eigen::Matrix4d T_imu_applanix;
    // Rotate applanix 90 degrees about z axis and then 180 degrees about y axis
    T_imu_applanix << 0, -1, 0, 0,
                      -1, 0, 0, 0,
                      0, 0, -1, 0,
                      0, 0, 0, 1;

    Eigen::Matrix4d T_radar_applanix = load_T_radar_applanix(path);

    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_radar_applanix * T_imu_applanix.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());
  } else {
    CLOG(ERROR, "boreas_wrapper") << "Unknown IMU name: " << imu_name;
    return EdgeTransform();
  }

  return T_robot_imu.inverse();
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

EdgeTransform load_T_wheel_robot(const fs::path &path) {
  std::ifstream ifs2(path / "calib" / "T_applanix_wheel.txt", std::ios::in);
  Eigen::Matrix4d T_applanix_wheel_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs2 >> T_applanix_wheel_mat(row, col);

  CLOG(WARNING, "boreas_wrapper") << "T_applanix_wheel_mat: " << T_applanix_wheel_mat;

  Eigen::Matrix4d T_radar_applanix = load_T_radar_applanix(path);       

  EdgeTransform T_wheel_robot(Eigen::Matrix4d((T_radar_applanix * T_applanix_wheel_mat).inverse()),
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

double roundToPi(double value) {
    return std::round(value / M_PI) * M_PI;
}

void load_groundtruth(const fs::path &path, std::vector<lgmath::se3::Transformation> &all_gt_poses, std::vector<Eigen::Vector<double, 6>> &all_gt_vels) {
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

void load_radar_time_span(const cv::Mat &raw_data, int64_t &start_time, int64_t &final_time) {
  const uint N = raw_data.rows;  
  start_time = *((int64_t *)(raw_data.ptr<uchar>(0))) * 1000;
  final_time = *((int64_t *)(raw_data.ptr<uchar>(N - 1))) * 1000;
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
                                  << (stem + "_radar_odometry");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      stem + "_radar_odometry", 1);

  // Load IMU data
  const auto use_imu = node->declare_parameter<bool>("boreas.imu.use_imu", false);
  const auto imu_name = node->declare_parameter<std::string>("boreas.imu.imu_name", "dmu");
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
    const auto imu_path = odo_dir / "imu" / imu_file_name;
    load_all_imu_meas(imu_path, all_imu_meas, imu_file_name);
    T_imu_robot = load_T_imu_robot(odo_dir, imu_name);
    CLOG(WARNING, "boreas_wrapper") << "Loaded " << all_imu_meas.size() << " IMU measurements";
    CLOG(WARNING, "boreas_wrapper") << "Transform from IMU to robot has been set to:\n" << T_imu_robot;
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
  std::string radar_frame = "radar";

  const auto T_robot_radar = load_T_robot_radar(odo_dir);
  const auto T_radar_robot = T_robot_radar.inverse();
  CLOG(WARNING, "boreas_wrapper") << "Transform from " << robot_frame << " to "
                                  << radar_frame << " has been set to" << T_radar_robot;

  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg =
      tf2::eigenToTransform(Eigen::Affine3d(T_radar_robot.inverse().matrix()));
  msg.header.frame_id = robot_frame;
  msg.child_frame_id = radar_frame;
  tf_sbc->sendTransform(msg);

  const auto clock_publisher =
      node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // List of radar data
  std::vector<fs::directory_entry> files;
  const auto radar_dir_name = node->declare_parameter<std::string>("boreas.radar_dir_name", "radar");
  for (const auto &dir_entry : fs::directory_iterator{odo_dir / radar_dir_name})
    if (dir_entry.path().extension() == ".png") files.push_back(dir_entry);
  std::sort(files.begin(), files.end());
  CLOG(WARNING, "boreas_wrapper") << "Found " << files.size() << " radar data";
  const auto start_frame = node->declare_parameter<int>("boreas.odometry.start_frame", 0);
  const auto end_frame = node->declare_parameter<int>("boreas.odometry.end_frame", -1);

  // Load in groundtruth data
  const auto load_gt = node->declare_parameter<bool>("boreas.load_gt", true);
  CLOG(WARNING, "boreas_wrapper") << "Load groundtruth: " << load_gt;
  std::vector<lgmath::se3::Transformation> T_rad_world_gt;
  std::vector<Eigen::Vector<double, 6>> v_rad_gt;
  // Reserve space
  T_rad_world_gt.reserve(files.size());
  v_rad_gt.reserve(files.size());
  if (load_gt) {
    load_groundtruth(odo_dir, T_rad_world_gt, v_rad_gt);
    CLOG(WARNING, "boreas_wrapper") << "Loaded groundtruth for " << T_rad_world_gt.size() << " frames";
  }

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

    ///
    const auto timestamp = getStampFromPath(it->path().string());
    const auto scan = cv::imread(it->path().string(), cv::IMREAD_GRAYSCALE);

    CLOG(WARNING, "boreas_wrapper") << "\033[95mLoading radar frame " << frame
                                    << " with timestamp " << timestamp << "\033[0m";

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(timestamp);
    clock_publisher->publish(time_msg);
    
    // Feed in IMU data if available/desired
    std::vector<sensor_msgs::msg::Imu> gyro_msgs;
    if (use_imu) {
      int64_t timestamp_imu = all_imu_meas[imu_counter].timestamp_ns;
      int64_t start_timestamp;
      int64_t end_timestamp;
      load_radar_time_span(scan, start_timestamp, end_timestamp);

      if (imu_counter == 0) {
        // Find IMU measurement right before radar frame to initialize
        while (all_imu_meas[imu_counter].timestamp_ns < start_timestamp) {
          ++imu_counter;
        }
      }

      // Loop through all IMU measurements from previous one to end of current radar frame
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
    }

    // Feed in wheel encoder data if available/desired
    std::vector<std::pair<rclcpp::Time, double>> wheel_meas;
    if (use_wheel_encoder) {
      int64_t timestamp_wheel = all_wheel_meas[wheel_counter].first;
      int64_t start_timestamp;
      int64_t end_timestamp;
      load_radar_time_span(scan, start_timestamp, end_timestamp);

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
    auto query_data = std::make_shared<radar::RadarQueryCache>();

    // some modules require node for visualization
    query_data->node = node;

    // make up some environment info (not important)
    tactic::EnvInfo env_info;
    env_info.terrain_type = 0;
    query_data->env_info.emplace(env_info);

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_radar_robot);

    // set timestamp
    query_data->stamp.emplace(timestamp);

    // set radar frame
    query_data->scan.emplace(scan);

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

    // Set groundtruth if loaded
    if (load_gt && frame < T_rad_world_gt.size()) {
      query_data->T_s_world_gt.emplace(T_rad_world_gt[frame]);
      query_data->v_s_gt.emplace(v_rad_gt[frame]);
    }

    // Add sequence name to query data
    query_data->seq_name = stem;

    // execute the pipeline
    tactic->input(query_data);

    std_msgs::msg::String status_msg;
    status_msg.data = "Finished processing radar frame " +
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