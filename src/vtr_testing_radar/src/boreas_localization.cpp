#include <algorithm>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_radar/icp_pipeline.hpp"
#include "vtr_radar/direct_pipeline.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"

#include "vtr_testing_common/vtr_testing_common.hpp"
#include "vtr_testing_common/radar_utils.hpp"

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::testing;


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
                                  << (stem + "_radar_localization");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      stem + "_radar_localization", 1);

  // Load IMU data
  const auto use_imu = node->declare_parameter<bool>("boreas.imu.use_imu", false);
  const auto imu_name = node->declare_parameter<std::string>("boreas.imu.imu_name", "dmu");
  const auto force_wrap = node->declare_parameter<bool>("boreas.imu.force_wrap", false);
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
    load_all_imu_meas(imu_path, all_imu_meas, imu_file_name);
    T_imu_robot = load_T_imu_robot(loc_dir, imu_name);
    CLOG(WARNING, "boreas_wrapper") << "Loaded " << all_imu_meas.size() << " IMU measurements";
    CLOG(WARNING, "boreas_wrapper") << "Transform from IMU to robot has been set to:\n" << T_imu_robot;
    CLOG(WARNING, "boreas_wrapper") << "IMU force wrap: " << force_wrap;
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

  CLOG(WARNING, "boreas_wrapper") << "Test vertices: " << ss.str();
  if (reverse) CLOG(WARNING, "boreas_wrapper") << "Running localization in reverse";

  /// NOTE: odometry is teach, localization is repeat
  auto T_loc_odo_init = [&]() {
    const auto T_robot_radar_odo = load_T_robot_radar(odo_dir);
    const auto T_enu_radar_odo = load_T_enu_init(odo_dir, reverse, "radar");

    const auto T_robot_radar_loc = load_T_robot_radar(loc_dir);
    const auto T_enu_radar_loc = load_T_enu_init(loc_dir, false, "radar");

    return T_robot_radar_loc * T_enu_radar_loc.inverse() * T_enu_radar_odo *
           T_robot_radar_odo.inverse();
  }();
  T_loc_odo_init.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());
  CLOG(WARNING, "boreas_wrapper")
      << "Transform from localization to odometry has been set to "
      << T_loc_odo_init.vec().transpose();

  tactic->setPath(sequence, /* trunk sid */ 0, T_loc_odo_init, true);

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string radar_frame = "radar";

  const auto T_robot_radar = load_T_robot_radar(loc_dir);
  const auto T_radar_robot = T_robot_radar.inverse();
  CLOG(WARNING, "boreas_wrapper") << "Transform from " << robot_frame << " to "
                                  << radar_frame << " has been set to" << T_radar_robot;

  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg =
      tf2::eigenToTransform(Eigen::Affine3d(T_radar_robot.inverse().matrix()));
  msg.header.frame_id = robot_frame;
  msg.child_frame_id = radar_frame;
  tf_sbc->sendTransform(msg);

  const auto clock_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // List of radar data
  std::vector<fs::directory_entry> files;
  const auto radar_dir_name = node->declare_parameter<std::string>("boreas.radar_dir_name", "radar");
  for (const auto &dir_entry : fs::directory_iterator{loc_dir / radar_dir_name})
    if (dir_entry.path().extension() == ".png") files.push_back(dir_entry);
  std::sort(files.begin(), files.end());
  CLOG(WARNING, "boreas_wrapper") << "Found " << files.size() << " radar data";
  const auto start_frame = node->declare_parameter<int>("odometry.start_frame", 0);
  const auto end_frame = node->declare_parameter<int>("odometry.end_frame", -1);

  // Load in groundtruth data
  const auto load_gt = node->declare_parameter<bool>("boreas.load_gt", true);
  CLOG(WARNING, "boreas_wrapper") << "Load groundtruth: " << load_gt;
  std::vector<lgmath::se3::Transformation> T_rad_world_gt;
  std::vector<Eigen::Vector<double, 6>> v_rad_gt;
  // Reserve space
  T_rad_world_gt.reserve(files.size());
  v_rad_gt.reserve(files.size());
  if (load_gt) {
    load_radar_groundtruth(loc_dir, T_rad_world_gt, v_rad_gt);
    CLOG(WARNING, "boreas_wrapper") << "Loaded groundtruth for " << T_rad_world_gt.size() << " frames";
  }

  // thread handling variables
  TestControl test_control(node);

  // main loop
  int frame = 0;
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
      int64_t start_timestamp;
      int64_t end_timestamp;
      load_radar_time_span(scan, start_timestamp, end_timestamp);

      CLOG(WARNING, "boreas_wrapper") << "Radar start timestamp: " << start_timestamp;
      CLOG(WARNING, "boreas_wrapper") << "Radar end timestamp: " << end_timestamp;

      const auto make_gyro_msg = [](const IMUMeasurement &m, int64_t stamp_ns) {
        auto gyro_msg = sensor_msgs::msg::Imu();
        gyro_msg.angular_velocity.x = m.angvel_x;
        gyro_msg.angular_velocity.y = m.angvel_y;
        gyro_msg.angular_velocity.z = m.angvel_z;
        gyro_msg.header.stamp = rclcpp::Time(stamp_ns);
        return gyro_msg;
      };

      // direct_odometry.cpp reconstructs gyro/scan timestamps as
      // sec*1e6 + nanosec/1000, i.e. it truncates to microsecond resolution
      // before comparing them. A boundary nudge smaller than 1us can get
      // swallowed by that truncation and land back on the same value, so
      // the wrap margin below must be at least 1000ns.
      constexpr int64_t kWrapMarginNs = 1000;

      // Load gyro measurements bracketing the radar scan timestamps
      const auto begin_it = std::lower_bound(
          all_imu_meas.begin(), all_imu_meas.end(), start_timestamp,
          [](const IMUMeasurement &m, int64_t t) { return m.timestamp_ns < t; });
      const auto end_it = std::lower_bound(
          all_imu_meas.begin(), all_imu_meas.end(), end_timestamp,
          [](const IMUMeasurement &m, int64_t t) { return m.timestamp_ns < t; });

      if (begin_it != all_imu_meas.begin()) {
        const auto &m = *std::prev(begin_it);
        // lower_bound only guarantees m.timestamp_ns < start_timestamp, which
        // should always hold here by construction, but clamp defensively so
        // the downstream strict "< scan start" wrap check (at microsecond
        // resolution) can never be starved by a boundary tie.
        if (m.timestamp_ns > start_timestamp - kWrapMarginNs) {
          CLOG(WARNING, "boreas_wrapper")
              << "IMU sample before scan start lands within " << kWrapMarginNs
              << "ns of the boundary, nudging its timestamp back to satisfy "
                 "the wrap check.";
        }
        gyro_msgs.push_back(make_gyro_msg(
            m, std::min(m.timestamp_ns, start_timestamp - kWrapMarginNs)));
      } else if (force_wrap && !all_imu_meas.empty()) {
        // No real IMU sample exists before the scan start (e.g. the IMU
        // log starts partway through the sequence). force_wrap fabricates
        // a "before" anchor from the nearest available sample so the
        // wrap-around requirement is still satisfied.
        CLOG(WARNING, "boreas_wrapper")
            << "No IMU sample before scan start, force_wrap is fabricating one.";
        gyro_msgs.push_back(
            make_gyro_msg(*begin_it, start_timestamp - kWrapMarginNs));
      }
      for (auto iter = begin_it; iter != end_it; ++iter) {
        gyro_msgs.push_back(make_gyro_msg(*iter, iter->timestamp_ns));
      }
      if (end_it != all_imu_meas.end()) {
        const auto &m = *end_it;
        // lower_bound only guarantees m.timestamp_ns >= end_timestamp, which
        // is not enough: the downstream wrap check requires a sample
        // strictly after the scan's last azimuth timestamp, compared at
        // microsecond resolution. When the real sample lands within one
        // microsecond of (or exactly on) end_timestamp, that check silently
        // fails and the whole frame's localization gets dropped. Nudge
        // forward by kWrapMarginNs in that case, matching the margin
        // force_wrap already uses below.
        if (m.timestamp_ns < end_timestamp + kWrapMarginNs) {
          CLOG(WARNING, "boreas_wrapper")
              << "IMU sample after scan end lands within " << kWrapMarginNs
              << "ns of the boundary, nudging its timestamp forward to "
                 "satisfy the wrap check.";
        }
        gyro_msgs.push_back(make_gyro_msg(
            m, std::max(m.timestamp_ns, end_timestamp + kWrapMarginNs)));
      } else if (force_wrap && !all_imu_meas.empty()) {
        // No real IMU sample exists after the scan end (e.g. the IMU log
        // ends before the radar log). force_wrap fabricates an "after"
        // anchor from the nearest available sample.
        CLOG(WARNING, "boreas_wrapper")
            << "No IMU sample after scan end, force_wrap is fabricating one.";
        gyro_msgs.push_back(make_gyro_msg(*std::prev(all_imu_meas.end()),
                                           end_timestamp + kWrapMarginNs));
      }
      CLOG(WARNING, "boreas_wrapper") << "Loaded " << gyro_msgs.size() << " IMU measurements";
    }

    // Feed in wheel encoder data if available/desired
    std::vector<std::pair<rclcpp::Time, double>> wheel_meas;
    if (use_wheel_encoder) {
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

    // Fill in scan_msg for pipelines that use it
    navtech_msgs::msg::RadarBScanMsg scan_msg;
    scan_msg.b_scan_img.header.stamp = rclcpp::Time(timestamp);
    query_data->scan_msg.emplace(scan_msg);

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