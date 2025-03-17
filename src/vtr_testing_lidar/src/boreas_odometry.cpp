#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_lidar/pipeline.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"

#include "vtr_testing_lidar/utils.hpp"

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

std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path) {
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

EdgeTransform load_T_robot_lidar(const fs::path &path) {
#if true
  std::ifstream ifs(path / "calib" / "T_applanix_lidar.txt", std::ios::in);

  Eigen::Matrix4d T_applanix_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs >> T_applanix_lidar_mat(row, col);
  // Extrinsic from radar to rear axel
  Eigen::Matrix4d T_axel_applanix;
  // Want to estimate at rear axel, this transform has x forward, y right, z down
  T_axel_applanix << 0.0299955, 0.99955003, 0, 0.51,
                    -0.99955003, 0.0299955, 0, 0.0,
                    0, 0, 1, 1.45,
                    0, 0, 0, 1;

  EdgeTransform T_robot_lidar(Eigen::Matrix4d(T_axel_applanix * T_applanix_lidar_mat),
                              Eigen::Matrix<double, 6, 6>::Zero());

#else
  Eigen::Matrix4d T_robot_lidar_mat;
  // clang-format off
  /// results from HERO paper
  T_robot_lidar_mat <<  0.68297386,  0.73044281,  0.        ,  0.26      ,
                       -0.73044281,  0.68297386,  0.        ,  0.        ,
                        0.        ,  0.        ,  1.        , -0.21      ,
                        0.        ,  0.        ,  0.        ,  1.        ;

  // clang-format on
  EdgeTransform T_robot_lidar(T_robot_lidar_mat,
                              Eigen::Matrix<double, 6, 6>::Zero());
#endif

  return T_robot_lidar;
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

  CLOG(WARNING, "test") << "Odometry Directory: " << odo_dir.string();
  CLOG(WARNING, "test") << "Output Directory: " << data_dir.string();

  std::vector<std::string> parts;
  boost::split(parts, odo_dir_str, boost::is_any_of("/"));
  auto stem = parts.back();
  boost::replace_all(stem, "-", "_");
  CLOG(WARNING, "test") << "Publishing status to topic: "
                        << (stem + "_lidar_odometry");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      stem + "_lidar_odometry", 1);

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
  std::string lidar_frame = "lidar";

  const auto T_robot_lidar = load_T_robot_lidar(odo_dir);
  const auto T_lidar_robot = T_robot_lidar.inverse();
  CLOG(WARNING, "test") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_lidar_robot;

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
  for (const auto &dir_entry : fs::directory_iterator{odo_dir / "lidar"})
    if (dir_entry.path().extension() == ".bin") files.push_back(dir_entry);
  std::sort(files.begin(), files.end());
  CLOG(WARNING, "test") << "Found " << files.size() << " lidar data";
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
    const auto [timestamp, points] = load_lidar(it->path().string());

    CLOG(WARNING, "test") << "Loading lidar frame " << frame
                          << " with timestamp " << timestamp;

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(timestamp);
    clock_publisher->publish(time_msg);

    std::vector<sensor_msgs::msg::Imu> gyro_msgs;
    // Feed in IMU data if available/desired
    if (use_imu) {
      int64_t timestamp_imu = all_imu_meas[imu_counter](0);
      int64_t start_timestamp = points(0, 5) * 1.0e9;
      int64_t end_timestamp = points(points.rows() - 1, 5) * 1.0e9;

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
    query_data->T_s_r.emplace(T_lidar_robot);

    // set gyro messages
    if (gyro_msgs.size() > 0) {
      query_data->T_s_r_gyro.emplace(T_imu_robot);
      query_data->gyro_msgs.emplace(gyro_msgs);
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
  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";
}