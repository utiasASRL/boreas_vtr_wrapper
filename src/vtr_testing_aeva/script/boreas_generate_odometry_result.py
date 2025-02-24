import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pylgmath import se3op
from pyboreas import BoreasDataset

np.set_printoptions(suppress=True)


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
    self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
    self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

  # Return messages as list of tuples [(timestamp0, message0), (timestamp1, message1), ...]
  def get_bag_messages(self, topic_name):
    topic_id = self.topic_id[topic_name]
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


def main(dataset_dir, result_dir, velocity):
  result_dir = osp.normpath(result_dir)
  odo_input = osp.basename(result_dir)
  odo_input_seq = odo_input.split('.', 1)[0]
  print("Result Directory:", result_dir)
  print("Odometry Run:", odo_input_seq)
  print("Dataset Directory:", dataset_dir)
  
  # Aeries II transformation
  T_sr = np.array([[ 0.99982945,  0.01750912,  0.00567659, -1.03971349],
                   [-0.01754661,  0.99973757,  0.01034526, -0.38788971],
                   [-0.00549427, -0.01044368,  0.99993037, -1.69798033],
                   [ 0, 0, 0, 1]]).astype(np.float64)
  
  T_robot_lidar = get_inverse_tf(T_sr)

  # try:
  #   dataset_odo = BoreasDataset(osp.normpath(dataset_dir), [[odo_input_seq]])
  # except:
  #   print("Data set does not exist:", dataset_dir, odo_input_seq)
  #   return

  odo_dir = osp.join(result_dir, odo_input_seq)

  data_dir = osp.join(odo_dir, "graph/data")
  if not osp.exists(data_dir):
    print("Data directory does not exist:", data_dir)
    return
  print("Looking at result data directory:", data_dir)

  # get bag file
  bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "odometry_result")
  parser = BagFileParser(bag_file)
  messages = parser.get_bag_messages("odometry_result")

  result = []
  for _, message in enumerate(messages):
    timestamp = int(int(message[1].timestamp) / 1000)
    T_w_r_vec = np.array(message[1].t_world_robot.xi)[..., None]
    T_w_r = se3op.vec2tran(T_w_r_vec)
    T_r_w_res = get_inverse_tf(T_w_r).flatten().tolist()[:12]
    result.append([timestamp] + T_r_w_res)
  
  # if velocity:
  #   bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "odometry_vel_result")
  #   parser = BagFileParser(bag_file)
  #   messages = parser.get_bag_messages("odometry_vel_result")

    # vel_results = []
    # for _, message in enumerate(messages):
    #   w_v_r_robot = np.zeros((6))
    #   w_v_r_robot[0] = message[1].linear.x
    #   w_v_r_robot[1] = message[1].linear.y
    #   w_v_r_robot[2] = message[1].linear.z
    #   w_v_r_robot[3] = message[1].angular.x
    #   w_v_r_robot[4] = message[1].angular.y
    #   w_v_r_robot[5] = message[1].angular.z

    #   w_r_v_lidar = np.zeros((6))
    #   w_r_v_lidar[:3] = (- w_v_r_robot[:3].reshape(1, 3) @ T_robot_lidar[:3, :3]).flatten()
    #   w_r_v_lidar[3:] = (- w_v_r_robot[3:].reshape(1, 3) @ T_robot_lidar[:3, :3]).flatten()

    #   timestamp = int(int(message[0]) / 1000)
    #   vel_results.append([timestamp] + w_r_v_lidar.flatten().tolist())

  output_dir = osp.join(result_dir, "odometry_result")
  os.makedirs(output_dir, exist_ok=True)
  with open(osp.join(output_dir, odo_input_seq + ".txt"), "+w") as file:
    writer = csv.writer(file, delimiter=' ')
    writer.writerows(result)
    print("Written to file:", osp.join(output_dir, odo_input_seq + ".txt"))

  output_dir = osp.join(result_dir, "../odometry_result")
  os.makedirs(output_dir, exist_ok=True)
  with open(osp.join(output_dir, odo_input_seq + ".txt"), "+w") as file:
    writer = csv.writer(file, delimiter=' ')
    writer.writerows(result)
    print("Written to file:", osp.join(output_dir, odo_input_seq + ".txt"))
    
  # if velocity:
  #   output_dir = osp.join(result_dir, "odometry_vel_result")
  #   os.makedirs(output_dir, exist_ok=True)
  #   with open(osp.join(output_dir, odo_input + ".txt"), "+w") as file:
  #     writer = csv.writer(file, delimiter=' ')
  #     writer.writerows(vel_results)
  #     print("Written to file:", osp.join(output_dir, odo_input + ".txt"))

  #   output_dir = osp.join(result_dir, "../odometry_vel_result")
  #   os.makedirs(output_dir, exist_ok=True)
  #   with open(osp.join(output_dir, odo_input + ".txt"), "+w") as file:
  #     writer = csv.writer(file, delimiter=' ')
  #     writer.writerows(vel_results)
  #     print("Written to file:", osp.join(output_dir, odo_input + ".txt"))


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--dataset', default=os.getcwd(), type=str, help='path to boreas dataset (contains boreas-*)')
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')
  parser.add_argument('--velocity', default=False, action='store_true', help='evaluate velocity (default: False)')

  args = parser.parse_args()

  main(args.dataset, args.path, args.velocity)