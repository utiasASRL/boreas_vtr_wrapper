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


def main(dataset_dir, result_dir, sensor_type):
  result_dir = osp.normpath(result_dir)
  odo_input = osp.basename(result_dir)
  loc_inputs = [i for i in os.listdir(result_dir) if (i != odo_input and i.startswith("20"))]
  loc_inputs.sort()
  print("Result Directory:", result_dir)
  print("Odometry Run:", odo_input)
  print("Localization Runs:", loc_inputs)
  print("Dataset Directory:", dataset_dir)
  
  if sensor_type == "aevaii_boreas":
    T_sr = np.array([[ 0.99982945,  0.01750912,  0.00567659, -1.03971349],
                     [-0.01754661,  0.99973757,  0.01034526, -0.38788971],
                     [-0.00549427, -0.01044368,  0.99993037, -1.69798033],
                     [ 0, 0, 0, 1]]).astype(np.float64)
  elif sensor_type == "aeva_boreas":
    T_sr = np.array([[ 0.9999366830849237,    0.008341717781538466,   0.0075534496251198685, -1.0119098938516395],
                     [-0.008341717774127972,  0.9999652112886684,    -3.150635091210066e-05, -0.3965882433517194],
                     [-0.007553449599178521, -3.1504388681967066e-05, 0.9999714717963843,    -1.6970000000000010],
                     [0, 0, 0, 1]]).astype(np.float64)
  else:
    raise ValueError("Unknown sensor type: ", sensor_type)
  
  # dataset directory and necessary sequences to load
  dataset_odo = BoreasDataset(osp.normpath(dataset_dir), [[odo_input]])

  # generate ground truth pose dictionary
  ground_truth_poses_odo = dict()
  for sequence in dataset_odo.sequences:
    # Ground truth is provided w.r.t sensor, so we set sensor to vehicle transform 
    # New way using rear axle
    T_lidar_robot_odo = T_sr
    T_robot_lidar_odo = get_inverse_tf(T_lidar_robot_odo)

    # build dictionary
    precision = 1e7  # divide by this number to ensure always find the timestamp
    ground_truth_poses_odo.update(
        {int(int(frame.timestamp * 1e9) / precision): frame.pose for frame in sequence.aeva_frames})
  print("Loaded number of odometry poses: ", len(ground_truth_poses_odo))

  for i, loc_input in enumerate(loc_inputs):

    # dataset directory and necessary sequences to load
    dataset_loc = BoreasDataset(osp.normpath(dataset_dir), [[loc_input]])

    # generate ground truth pose dictionary
    ground_truth_poses_loc = dict()
    for sequence in dataset_loc.sequences:
      # Ground truth is provided w.r.t sensor, so we set sensor to vehicle transform
      # New way using rear axle
      T_lidar_robot_loc = T_sr
      T_robot_lidar_loc = get_inverse_tf(T_lidar_robot_loc)

      # build dictionary
      precision = 1e7  # divide by this number to ensure always find the timestamp
      ground_truth_poses_loc.update(
          {int(int(frame.timestamp * 1e9) / precision): frame.pose for frame in sequence.aeva_frames})

    print("Loaded number of localization poses: ", len(ground_truth_poses_loc))

    loc_dir = osp.join(result_dir, loc_input)

    data_dir = osp.join(loc_dir, "graph/data")
    if not osp.exists(data_dir):
      continue
    print("Looking at result data directory:", data_dir)

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
      T_test_map_in_lidar = T_lidar_robot_loc @ T_test_map @ T_robot_lidar_odo
      T_map_test_in_lidar = get_inverse_tf(T_test_map_in_lidar)
      T_map_test_in_lidar_res = T_map_test_in_lidar.flatten().tolist()[:12]
      result.append([test_seq_timestamp, map_seq_timestamp] + T_map_test_in_lidar_res)

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
      T_test_map_in_lidar = T_lidar_robot_loc @ T_test_map @ T_robot_lidar_odo
      T_map_test_in_lidar = get_inverse_tf(T_test_map_in_lidar)
      T_test_map_in_lidar_gt = get_inverse_tf(
          ground_truth_poses_loc[test_seq_timestamp]) @ ground_truth_poses_odo[map_seq_timestamp]
      # compute error
      errors[i, :] = se3op.tran2vec(T_map_test_in_lidar @ T_test_map_in_lidar_gt).flatten()

    print(np.mean(np.abs(errors), axis=0))
    errors[:, 3:] = np.rad2deg(errors[:, 3:])
    print(np.sqrt(np.mean(np.power(errors, 2), axis=0)))

    output_dir = osp.join(result_dir, "localization_result")
    os.makedirs(output_dir, exist_ok=True)
    with open(osp.join(output_dir, loc_input + ".txt"), "+w") as file:
      writer = csv.writer(file, delimiter=' ')
      writer.writerows(result)
      print("Written to file:", osp.join(output_dir, loc_input + ".txt"))
      
  if False:
    bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "odometry_vel_result")
    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("odometry_vel_result")

    vel_results = []
    for _, message in enumerate(messages):
      w_v_r_robot = np.zeros((6))
      w_v_r_robot[0] = message[1].linear.x
      w_v_r_robot[1] = message[1].linear.y
      w_v_r_robot[2] = message[1].linear.z
      w_v_r_robot[3] = message[1].angular.x
      w_v_r_robot[4] = message[1].angular.y
      w_v_r_robot[5] = message[1].angular.z

      w_r_v_lidar = np.zeros((6))
      w_r_v_lidar[:3] = (- w_v_r_robot[:3].reshape(1, 3) @ T_robot_lidar_loc[:3, :3]).flatten()
      w_r_v_lidar[3:] = (- w_v_r_robot[3:].reshape(1, 3) @ T_robot_lidar_loc[:3, :3]).flatten()

      timestamp = int(int(message[0]) / 1000)
      vel_results.append([timestamp] + w_r_v_lidar.flatten().tolist())
    
    output_dir = osp.join(result_dir, "localization_vel_result")
    os.makedirs(output_dir, exist_ok=True)
    with open(osp.join(output_dir, loc_input + "_vel.txt"), "+w") as file:
      writer = csv.writer(file, delimiter=' ')
      writer.writerows(vel_results)
      print("Written to file:", osp.join(output_dir, loc_input + "_vel.txt"))



if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--dataset', default=os.getcwd(), type=str, help='path to boreas dataset (contains boreas-*)')
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')
  parser.add_argument('--type', default=os.getcwd(), type=str, help='dataset type (which sensor?)')

  args = parser.parse_args()

  main(args.dataset, args.path, args.type)