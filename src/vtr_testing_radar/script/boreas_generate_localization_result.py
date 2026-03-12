import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pyboreas import BoreasDataset
from pylgmath import se3op


def get_inverse_tf(T):
  """Returns the inverse of a given 4x4 homogeneous transform.
    Args:
        T (np.ndarray): 4x4 transformation matrix
    Returns:
        np.ndarray: inv(T)
    """
  T2 = T.copy()
  T2[:3, :3] = T2[:3, :3].transpose()
  T2[:3, 3:] = -1 * T2[:3, :3] @ T2[:3, 3:]
  return T2


class BagFileParser():

  def __init__(self, bag_file):
    try:
      self.conn = sqlite3.connect(bag_file)
    except Exception as e:
      print('Could not connect: ', e)
      raise Exception('could not connect')

    self.cursor = self.conn.cursor()

    ## create a message (id, topic, type) map
    topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()

    self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
    self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
    self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

  # Return messages as list of tuples [(timestamp0, message0), (timestamp1, message1), ...]
  def get_bag_messages(self, topic_name):
    topic_id = self.topic_id[topic_name]
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


def main(dataset_dir, result_dir, quiet=False):
  result_dir = osp.normpath(result_dir)
  odo_input = osp.basename(result_dir)
  loc_inputs = [i for i in os.listdir(result_dir) if (i != odo_input and i.startswith("boreas"))]
  loc_inputs.sort()
  if not quiet:
    print("Result Directory:", result_dir)
    print("Odometry Run:", odo_input)
    print("Localization Runs:", loc_inputs)
    print("Dataset Directory:", dataset_dir)

  # dataset directory and necessary sequences to load
  dataset_odo = BoreasDataset(osp.normpath(dataset_dir), [[odo_input]])

  # generate ground truth pose dictionary
  ground_truth_poses_odo = dict()
  for sequence in dataset_odo.sequences:
    # Using radar frame as robot frame for SE(2) radar odometry
    # This was approved by Tim in October 2025
    T_robot_radar_odo = np.eye(4)

    # build dictionary
    precision = 1e7  # divide by this number to ensure always find the timestamp
    ground_truth_poses_odo.update(
        {int(int(frame.timestamp * 1e9) / precision): frame.pose for frame in sequence.radar_frames})
  if not quiet: print("Loaded number of odometry poses: ", len(ground_truth_poses_odo))

  for i, loc_input in enumerate(loc_inputs):

    # dataset directory and necessary sequences to load
    dataset_loc = BoreasDataset(osp.normpath(dataset_dir), [[loc_input]])

    # generate ground truth pose dictionary
    ground_truth_poses_loc = dict()
    for sequence in dataset_loc.sequences:
      T_applanix_wheel_file = os.path.join(dataset_dir, loc_input, "calib/T_applanix_wheel.txt")
      if not os.path.exists(T_applanix_wheel_file):
        print("File does not exist:", T_applanix_wheel_file, ". Loading default.")
        T_applanix_wheel = np.array([[0.999560,  0.029665, 0.000000, -0.813993],
                                    [-0.029665, 0.999560, 0.000000, -0.455312],
                                    [0.000000, 0.000000, 1.000000, -1.610000],
                                    [0.000000, 0.000000, 0.000000, 1.000000]])
      else:
        T_applanix_wheel = np.loadtxt(T_applanix_wheel_file)
      T_wheel_applanix = get_inverse_tf(T_applanix_wheel)
      T_wheelfwd_wheel = np.array([[0, 1, 0, 0],
                                  [-1, 0, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])
      T_robot_applanix = T_wheelfwd_wheel @ T_wheel_applanix

      # Load in T_radar_applanix
      T_radar_lidar_file = os.path.join(dataset_dir, loc_input, "calib/T_radar_lidar.txt")
      if not os.path.exists(T_radar_lidar_file):
        raise Exception("File does not exist: " + T_radar_lidar_file)
      T_radar_lidar = np.loadtxt(T_radar_lidar_file)
      T_applanix_lidar_file = os.path.join(dataset_dir, loc_input, "calib/T_applanix_lidar.txt")
      if not os.path.exists(T_applanix_lidar_file):
        raise Exception("File does not exist: " + T_applanix_lidar_file)
      T_applanix_lidar = np.loadtxt(T_applanix_lidar_file)
      T_radar_applanix = T_radar_lidar @ get_inverse_tf(T_applanix_lidar)

      # Compute final transform from radar to robot frame
      T_robot_radar_loc = T_robot_applanix @ get_inverse_tf(T_radar_applanix)
      T_radar_robot_loc = get_inverse_tf(T_robot_radar_loc)

      # build dictionary
      precision = 1e7  # divide by this number to ensure always find the timestamp
      ground_truth_poses_loc.update(
          {int(int(frame.timestamp * 1e9) / precision): frame.pose for frame in sequence.radar_frames})

    if not quiet: print("Loaded number of localization poses: ", len(ground_truth_poses_loc))

    loc_dir = osp.join(result_dir, loc_input)

    data_dir = osp.join(loc_dir, "graph/data")
    if not osp.exists(data_dir):
      continue
    if not quiet: print("Looking at result data directory:", data_dir)

    # get bag file
    bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "localization_result")
    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("localization_result")

    result = []
    errors = np.empty((len(messages), 6))
    for i, message in enumerate(messages):

      test_seq_timestamp = int(int(message[1].timestamp) / 1000)
      map_seq_timestamp = int(int(message[1].vertex_timestamp) / 1000)
      T_test_map_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
      T_test_map = se3op.vec2tran(T_test_map_vec)
      T_test_map_in_radar = T_radar_robot_loc @ T_test_map @ T_robot_radar_odo
      T_map_test_in_radar = get_inverse_tf(T_test_map_in_radar)
      T_map_test_in_radar_res = T_map_test_in_radar.flatten().tolist()[:12]
      result.append([test_seq_timestamp, map_seq_timestamp] + T_map_test_in_radar_res)

      if not int(message[1].timestamp / precision) in ground_truth_poses_loc.keys():
        print("WARNING: time stamp not found 1: ", int(message[1].timestamp / precision))
        continue
      if not int(message[1].vertex_timestamp / precision) in ground_truth_poses_odo.keys():
        print("WARNING: time stamp not found 2: ", int(message[1].vertex_timestamp / precision))
        continue

      test_seq_timestamp = int(message[1].timestamp / precision)
      map_seq_timestamp = int(message[1].vertex_timestamp / precision)
      T_test_map_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
      T_test_map = se3op.vec2tran(T_test_map_vec)
      T_test_map_in_radar = T_radar_robot_loc @ T_test_map @ T_robot_radar_odo
      T_map_test_in_radar = get_inverse_tf(T_test_map_in_radar)
      T_test_map_in_radar_gt = get_inverse_tf(
          ground_truth_poses_loc[test_seq_timestamp]) @ ground_truth_poses_odo[map_seq_timestamp]
      # compute error
      errors[i, :] = se3op.tran2vec(T_map_test_in_radar @ T_test_map_in_radar_gt).flatten()

    if not quiet: print(np.mean(np.abs(errors), axis=0))

    output_dir = osp.join(result_dir, "localization_result")
    os.makedirs(output_dir, exist_ok=True)
    with open(osp.join(output_dir, loc_input + ".txt"), "+w") as file:
      writer = csv.writer(file, delimiter=' ')
      writer.writerows(result)
      if not quiet: print("Written to file:", osp.join(output_dir, loc_input + ".txt"))


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--dataset', default=os.getcwd(), type=str, help='path to boreas dataset (contains boreas-*)')
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')
  parser.add_argument('--quiet', action='store_true', help='suppress verbose output (default: False)')

  args = parser.parse_args()

  main(args.dataset, args.path, args.quiet)