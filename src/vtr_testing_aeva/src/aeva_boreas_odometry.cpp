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

Eigen::MatrixXd readCSVtoEigenXd(std::ifstream &csv) {
  std::string line;
  std::string cell;
  std::vector<std::vector<double>> mat_vec;
  while (std::getline(csv, line)) {
    std::stringstream lineStream(line);
    std::vector<double> row_vec;
    while (std::getline(lineStream, cell, ',')) {
      row_vec.push_back(std::stof(cell));
    }
    mat_vec.push_back(row_vec);
  }
  Eigen::MatrixXd output = Eigen::MatrixXd(mat_vec.size(), mat_vec[0].size());
  for (int i = 0; i < (int)mat_vec.size(); ++i) output.row(i) = Eigen::VectorXd::Map(&mat_vec[i][0], mat_vec[i].size());
  return output;
}

int64_t getStampFromPath(const std::string &path) {
  std::vector<std::string> parts;
  boost::split(parts, path, boost::is_any_of("/"));
  std::string stem = parts[parts.size() - 1];
  boost::split(parts, stem, boost::is_any_of("."));
  int64_t time1 = std::stoll(parts[0]);
  return time1 * 1000;
}

std::vector<Eigen::MatrixXd> loadElevationOrder(const std::string &bo_path) {
  // load values for computing line id from elevation
  std::vector<Eigen::MatrixXd> elevation_order_by_beam_id_;

  // read elevation settings
  std::string path = bo_path; // + "/mean_elevation_beam_order_0";
  std::ifstream csv(path);
  if (!csv) throw std::ios::failure("Error opening csv file");
  Eigen::MatrixXd elevation_order = readCSVtoEigenXd(csv);

  for (int j = 0; j < 4; ++j) {   // 4 beams   
    Eigen::MatrixXd elevation_order_for_this_beam(elevation_order.rows()/4, 2);  // first column is mean elevation, second column is row id
    int h = 0;
    for (int r = 0; r < elevation_order.rows(); ++r) {
      // first column is mean elevation. Second column is beam id
      if (elevation_order(r, 1) == j) {
        elevation_order_for_this_beam(h, 0) = elevation_order(r, 0);
        elevation_order_for_this_beam(h, 1) = r;
        ++h;
      }
    } // end for r
    assert(h == elevation_order.rows()/4);
    elevation_order_by_beam_id_.push_back(elevation_order_for_this_beam);
  } // end for j
  assert(elevation_order_by_beam_id_.size() == 4); // 4 beams

  return elevation_order_by_beam_id_;
}

std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path, const std::string &bo_path, double start_time, double end_time, int64_t filename) {
  // load Aeries I pointcloud
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  unsigned float_offset = 4; // float32
  unsigned fields = 7;  // x, y, z, i, r, t, b
  unsigned point_step = float_offset * fields;
  unsigned N = floor(buffer.size() / point_step);

  std::vector<Eigen::VectorXd> points; // Vector to store valid points dynamically

  auto getFloatFromByteArray = [](char *byteArray, unsigned index) -> float {
    return *((float *)(byteArray + index));
  };

  auto elevation_order = loadElevationOrder(bo_path);

  for (unsigned i(0); i < N; i++) {
    int bufpos = i * point_step;
    int offset = 0;

    // Temporary variables
    double x, y, z, radial_velocity, intensity, time_temp;
    int64_t time_keep;
    int beam_id, line_id, face_id, sensor_id;

    // x, y, z
    x = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    y = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    z = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    
    ++offset;
    // Intensity
    intensity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // Radial velocity
    radial_velocity = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset);
    ++offset;
    // Timestamp
    time_temp = getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset); // sec
    double t = double(filename / 1000) * 1.0e-6;
    time_keep = (int64_t)((time_temp + t) * 1e9);
    time_temp = time_temp + start_time;
    ++offset;
    // Beam id
    beam_id = (int)(getFloatFromByteArray(buffer.data(), bufpos + offset * float_offset));
    // Face id - 0 because not available on aeries I
    face_id = 0;
    // Sensor id
    sensor_id = 0;

    // compute elevation
    const double xy = sqrt(x*x + y*y);
    const double elevation = atan2(z, xy);
    
    // determine row by matching by beam_id (0, 1, 2, or 3) and closest elevation to precalculated values
    // note: elevation_order_by_beam_id_[point.beam_id] first column is mean elevation, second column is row id
    const auto ele_diff = elevation_order[beam_id].col(0).array() - elevation;
    double min_val = ele_diff(0)*ele_diff(0);
    size_t min_id = 0;
    for (size_t i = 1; i < ele_diff.rows(); ++i) {
      const auto val = ele_diff(i) * ele_diff(i);
      if (val < min_val) {
        min_val = val;
        min_id = i;
      }
    }

    line_id = elevation_order[beam_id](min_id, 1);

    // Include if within start and end time
    if (time_temp > start_time && time_temp <= end_time) {
        Eigen::VectorXd point(10);
        //std::cout << "time_keep: " << time_keep << std::endl;
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

std::pair<int64_t, Eigen::MatrixXd> load_new_lidar(const std::string &path, double start_time, double end_time, int64_t filename) {
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

EdgeTransform load_T_lidar_robot(const fs::path &path, bool new_lidar) {
  std::ifstream ifs1(path / "calib" / "T_applanix_aeva.txt", std::ios::in);

  Eigen::Matrix4d T_applanix_aeva_mat;
  if (!ifs1.is_open()) {
    CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_aeva.txt. Loading preset.";
    T_applanix_aeva_mat << 0.0116474, -0.99998734, 0.0, -0.37043642,
                            0.9999333, 0.0116284, 0.0, 0.39745466,
                            0.0, 0.0, 1.0, -0.1032,
                            0.0, 0.0, 0.0, 1.0;
  } else {
    for (size_t row = 0; row < 4; row++)
      for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_aeva_mat(row, col);
  }

  // Extrinsic from applanix to rear axel
  Eigen::Matrix4d T_axel_applanix;
  // Want to estimate at rear axel
  T_axel_applanix << 0.0299955, 0.99955003, 0, 0.51,
                    -0.99955003, 0.0299955, 0, 0.0,
                      0, 0, 1, 1.45,
                      0, 0, 0, 1;

  EdgeTransform T_lidar_robot(Eigen::Matrix4d((T_axel_applanix * T_applanix_aeva_mat).inverse()),   // transform
                              Eigen::Matrix<double, 6, 6>::Zero());                                 // covariance
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
      } else if (imu_file_name == "aeva_imu.csv") {
        imu_meas_mat << imu[0], imu[1], imu[2], imu[3]; // timestamp, angvel_x, angvel_y, angvel_z
      } else {
        CLOG(ERROR, "boreas_wrapper") << "Unknown IMU file name: " << imu_file_name;
        break;
      }        
      all_imu_meas.push_back(imu_meas_mat);
  }
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
    if (!ifs1.is_open()) {
      CLOG(ERROR, "boreas_wrapper") << "Could not open file: " << path / "calib" / "T_applanix_aeva.txt. Loading preset.";
      T_applanix_aeva_mat << 0.0116474, -0.99998734, 0.0, -0.37043642,
                             0.9999333, 0.0116284, 0.0, 0.39745466,
                             0.0, 0.0, 1.0, -0.1032,
                             0.0, 0.0, 0.0, 1.0;
    } else {
      for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_aeva_mat(row, col);
    }

    CLOG(WARNING, "boreas_wrapper") << "T_applanix_aeva_mat: " << T_applanix_aeva_mat;

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
  const auto use_imu = node->declare_parameter<bool>("boreas.use_imu", false);
  const auto imu_name = node->declare_parameter<std::string>("boreas.imu_name", "aeva");
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

  // KTODO: move to preprocessing - maybe keep bias here and make cov a parameter
  const auto sensor_config_path = node->declare_parameter<std::string>("boreas.root_path", "/home/");
  const auto model_name = node->declare_parameter<std::string>("boreas.model_name", "glen");
  const auto aeriesII = node->declare_parameter<bool>("boreas.aeriesII", false);

  CLOG(WARNING, "boreas_wrapper") << "Sensor config path: " << sensor_config_path;
  CLOG(WARNING, "boreas_wrapper") << "Model name: " << model_name;
  CLOG(WARNING, "boreas_wrapper") << "Aeries II: " << aeriesII;

  // Load const_gyro_bias from gyro_bias.txt
  std::string gyro_bias_path = sensor_config_path + "/" + model_name + "/gyro_bias.txt";
  std::ifstream bi_csv(gyro_bias_path);
  Eigen::MatrixXd gbias_matrix = readCSVtoEigenXd(bi_csv);
  std::vector<double> gbias(gbias_matrix.data(), gbias_matrix.data() + gbias_matrix.size());
  std::vector<Eigen::Vector3d> const_gyro_bias;
  for (int i = 0; i < gbias.size(); i += 3) {
    const_gyro_bias.push_back(Eigen::Vector3d(gbias[i], gbias[i+1], gbias[i+2]));
  }

  CLOG(WARNING, "boreas_wrapper") << "Gyro bias: " << const_gyro_bias[0].transpose();

  std::string gyro_cov_path = sensor_config_path + "/" + model_name + "/gyro_cov.txt";
  std::ifstream co_csv(gyro_cov_path);
  std::vector<double> cov_arr;
  if (co_csv.is_open()) {
    std::string line;
    std::getline(co_csv, line);
    std::stringstream ss(line);
    std::string value;
    while (std::getline(ss, value, ',')) {
      cov_arr.push_back(std::stod(value));
    }
  } else {
    CLOG(ERROR, "boreas_wrapper") << "Unable to open gyro_cov file: " + gyro_cov_path;
  }

  Eigen::Matrix3d gyro_invcov = Eigen::Matrix3d::Identity();
  gyro_invcov(0,0) = 2.0/(cov_arr[0]);
  gyro_invcov(1,1) = 2.0/(cov_arr[1]);
  gyro_invcov(2,2) = 2.0/(cov_arr[2]);

  CLOG(WARNING, "boreas_wrapper") << "Gyro cov: " << cov_arr[0] << ", " << cov_arr[1] << ", " << cov_arr[2];

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "lidar";

  const auto T_lidar_robot = load_T_lidar_robot(odo_dir, aeriesII);
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
  const auto start_frame = node->declare_parameter<int>("boreas.start_frame", 0);
  const auto end_frame = node->declare_parameter<int>("boreas.end_frame", -1);

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

    const auto filename = getStampFromPath((it)->path().string());
    int64_t time_delta_micro = filename - first_state_time;
    double start_time = static_cast<double>(time_delta_micro) / 1e9;

    CLOG(WARNING, "boreas_wrapper") << "Loading aeva frame " << frame << " with timestamp " << filename;

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
    
    double dt = 0;
    Eigen::MatrixXd points;
    if (aeriesII) {
      // load Aeries II boreas pointcloud
      std::tie(std::ignore, points) = load_new_lidar(it->path().string(), start_time, end_time, start_name);
    } else {
      dt = 0.1; // aeries I gyro time sync ~0.1s off
      // load Aeries I boreas pointcloud
      auto [fields, points] = load_lidar(it->path().string(), sensor_config_path, start_time, end_time, start_name);
    }

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

    // load gyro data
    std::vector<sensor_msgs::msg::Imu> gyro_msgs;
    // Feed in IMU data if available/desired
    if (use_imu) {
      int64_t timestamp_imu = all_imu_meas[imu_counter](0);
      int64_t start_timestamp = points(0, 5) / 1000;
      int64_t end_timestamp = points(points.rows() - 1, 5) / 1000;

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
        gyro_msg.angular_velocity.x = all_imu_meas[imu_counter](1) + const_gyro_bias[0](0);
        gyro_msg.angular_velocity.y = all_imu_meas[imu_counter](2) + const_gyro_bias[0](1);
        gyro_msg.angular_velocity.z = all_imu_meas[imu_counter](3) + const_gyro_bias[0](2);
        int64_t timestamp = all_imu_meas[imu_counter](0);
        gyro_msg.header.stamp = rclcpp::Time(timestamp * 1000);
        gyro_msgs.push_back(gyro_msg);
        ++imu_counter;
      }
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
      CLOG(WARNING, "boreas_wrapper") << "Loaded " << gyro_msgs.size() << " gyro measurements";
      query_data->T_s_r_gyro.emplace(T_imu_robot);
      query_data->gyro_msgs.emplace(gyro_msgs);
      query_data->gyro_invcov.emplace(gyro_invcov);
    }

    // set timestamp of first frame [ns]
    query_data->first_state_time.emplace(first_state_time);

    // set timestamp of next state [ns]
    query_data->next_state_time.emplace(next_state_time);

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