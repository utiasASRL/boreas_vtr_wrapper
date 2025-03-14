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

EdgeTransform load_T_robot_radar(const fs::path &path) {
  std::ifstream ifs1(path / "calib" / "T_applanix_lidar.txt", std::ios::in);
  std::ifstream ifs2(path / "calib" / "T_radar_lidar.txt", std::ios::in);

  Eigen::Matrix4d T_applanix_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_lidar_mat(row, col);

  Eigen::Matrix4d T_radar_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs2 >> T_radar_lidar_mat(row, col);

  // Extrinsic from applanix to rear axel
  Eigen::Matrix4d T_axel_applanix;
  // Want to estimate at rear axel
  T_axel_applanix << 0.0299955, 0.99955003, 0, 0.51,
                    -0.99955003, 0.0299955, 0, 0.0,
                    0, 0, 1, 1.45,
                    0, 0, 0, 1;

  EdgeTransform T_robot_radar(Eigen::Matrix4d(T_axel_applanix * T_applanix_lidar_mat *
                                              T_radar_lidar_mat.inverse()),
                              Eigen::Matrix<double, 6, 6>::Zero());

  return T_robot_radar;
}

EdgeTransform load_T_imu_robot(const fs::path &path, const std::string &imu_name) {
  EdgeTransform T_robot_imu;
  if (imu_name == "dmu") {
    std::ifstream ifs1(path / "calib" / "T_applanix_dmu.txt", std::ios::in);
    Eigen::Matrix4d T_applanix_dmu_mat;
    if (!ifs1.is_open()) {
      CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_dmu.txt. Loading preset.";
      T_applanix_dmu_mat << 1.0,  0.0,  0.0,  0.0,
                            0.0, -1.0,  0.0,  0.0,
                            0.0,  0.0, -1.0, -0.15,
                            0.0,  0.0,  0.0,  1.0;
    } else {
      for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_dmu_mat(row, col);
    }
    // Extrinsic from applanix to rear axel
    Eigen::Matrix4d T_axel_applanix;
    // Want to estimate at rear axel
    T_axel_applanix << 0.0299955, 0.99955003, 0, 0.51,
                      -0.99955003, 0.0299955, 0, 0.0,
                      0, 0, 1, 1.45,
                      0, 0, 0, 1;
  
    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_axel_applanix * T_applanix_dmu_mat),
                                Eigen::Matrix<double, 6, 6>::Zero());
  } else if (imu_name == "aeva") {
    std::ifstream ifs1(path / "calib" / "T_applanix_aeva.txt", std::ios::in);
    Eigen::Matrix4d T_applanix_aeva_mat;
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_aeva_mat(row, col);

    Eigen::Matrix4d T_imu_aeva_mat;
    T_imu_aeva_mat << 1.0, 0.0, 0.0, -0.020,
                        0.0, 1.0, 0.0, -0.023,
                        0.0, 0.0, 1.0, 0.037,
                        0.0, 0.0, 0.0, 1.0;

    // Extrinsic from applanix to rear axel
    Eigen::Matrix4d T_axel_applanix;
    // Want to estimate at rear axel
    T_axel_applanix << 0.0299955, 0.99955003, 0, 0.51,
                      -0.99955003, 0.0299955, 0, 0.0,
                      0, 0, 1, 1.45,
                      0, 0, 0, 1;
  
    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_axel_applanix * T_applanix_aeva_mat *
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

    // Extrinsic from applanix to rear axel
    Eigen::Matrix4d T_axel_applanix;
    // Want to estimate at rear axel
    T_axel_applanix << 0.0299955, 0.99955003, 0, 0.51,
                      -0.99955003, 0.0299955, 0, 0.0,
                      0, 0, 1, 1.45,
                      0, 0, 0, 1;
  
    T_robot_imu = EdgeTransform(Eigen::Matrix4d(T_axel_applanix * T_imu_applanix.inverse()),
                                Eigen::Matrix<double, 6, 6>::Zero());
  } else {
    CLOG(ERROR, "boreas_wrapper") << "Unknown IMU name: " << imu_name;
    return EdgeTransform();
  }

  return T_robot_imu.inverse();
}

void load_all_imu_meas(const fs::path &imu_meas_file, std::vector<Eigen::MatrixXd> &all_imu_meas, fs::path imu_file_name) {
  std::ifstream imu_stream(imu_meas_file, std::ios::in);
  // Get rid of header (GPSTime,angvel_z,angvel_y,angvel_x,accelz,accely,accelx)
  std::string header;
  std::getline(imu_stream, header);
  // Loop over all imu measurements
  std::string imu_meas;
  while (std::getline(imu_stream, imu_meas)) {
      std::stringstream ss(imu_meas);
      std::vector<long double> imu;
      for (std::string str; std::getline(ss, str, ',');)
              imu.push_back(std::stod(str));
      Eigen::MatrixXd imu_meas_mat = Eigen::MatrixXd(4, 1);
      if (imu_file_name == "imu.csv" || imu_file_name == "imu_raw.csv") {
        imu_meas_mat << imu[0], imu[3], imu[2], imu[1]; // timestamp, angvel_x, angvel_y, angvel_z
      } else if (imu_file_name == "dmu_imu.csv") {
        imu_meas_mat << imu[0], imu[7], imu[8], imu[9]; // timestamp, angvel_x, angvel_y, angvel_z
      } else {
        CLOG(ERROR, "boreas_wrapper") << "Unknown IMU file name: " << imu_file_name;
        break;
      }        
      all_imu_meas.push_back(imu_meas_mat);
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
  std::vector<Eigen::MatrixXd> all_imu_meas;
  EdgeTransform T_imu_robot; 
  if (use_imu) {
    // Check that imu name is one of "dmu", "aeva", "imu"
    CLOG(WARNING, "boreas_wrapper") << "IMU name: " << imu_name;
    if (imu_name != "dmu" && imu_name != "aeva" && imu_name != "imu") {
      CLOG(ERROR, "boreas_wrapper") << "Unknown IMU name: " << imu_name;
      return 1;
    }
    const auto imu_file_name = (imu_name == "imu") ? "imu_raw.csv" : (imu_name + "_imu.csv");
    const auto imu_path = odo_dir / "applanix" / imu_file_name;
    load_all_imu_meas(imu_path, all_imu_meas, imu_file_name);
    T_imu_robot = load_T_imu_robot(odo_dir, imu_name);
    CLOG(WARNING, "boreas_wrapper") << "Loaded " << all_imu_meas.size() << " IMU measurements";
    CLOG(WARNING, "boreas_wrapper") << "Transform from IMU to robot has been set to:\n" << T_imu_robot;
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

  // thread handling variables
  TestControl test_control(node);

  // main loop
  int frame = 0;
  int imu_counter = 0;
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

    CLOG(WARNING, "boreas_wrapper") << "Loading radar frame " << frame
                          << " with timestamp " << timestamp;

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(timestamp);
    clock_publisher->publish(time_msg);

    std::vector<sensor_msgs::msg::Imu> gyro_msgs;
    // Feed in IMU data if available/desired
    if (use_imu) {
      int64_t timestamp_imu = all_imu_meas[imu_counter](0);
      int64_t start_timestamp;
      int64_t end_timestamp;
      load_radar_time_span(scan, start_timestamp, end_timestamp);

      if (imu_counter == 0) {
        // Find IMU measurement right before radar frame to initialize
        while (all_imu_meas[imu_counter](0) < start_timestamp) {
          ++imu_counter;
        }
      }

      // Loop through all IMU measurements from previous one to end of current radar frame
      // This captures IMU measurements that are between frames
      Eigen::Matrix<double, 4, 1> imu_meas;
      while (imu_counter < all_imu_meas.size() && all_imu_meas[imu_counter](0) < end_timestamp) {
        auto gyro_msg = sensor_msgs::msg::Imu();
        gyro_msg.angular_velocity.x = all_imu_meas[imu_counter](1);
        gyro_msg.angular_velocity.y = all_imu_meas[imu_counter](2);
        gyro_msg.angular_velocity.z = all_imu_meas[imu_counter](3);
        gyro_msg.header.stamp = rclcpp::Time(all_imu_meas[imu_counter](0));
        gyro_msgs.push_back(gyro_msg);
        ++imu_counter;
      }
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