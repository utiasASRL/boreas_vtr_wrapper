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

  try:
    dataset_odo = BoreasDataset(osp.normpath(dataset_dir), [[odo_input_seq]])
  except:
    print("Data set does not exist:", dataset_dir, odo_input_seq)
    return

  odo_dir = osp.join(result_dir, odo_input_seq)

  data_dir = osp.join(odo_dir, "graph/data")
  if not osp.exists(data_dir):
    print("Data directory does not exist:", data_dir)
    return
  print("Looking at result data directory:", data_dir)
  
  T_applanix_wheel_file = os.path.join(dataset_dir, odo_input_seq, "calib/T_applanix_wheel.txt")
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
  
  # get bag file
  bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(data_dir), "odometry_result")
  parser = BagFileParser(bag_file)
  messages = parser.get_bag_messages("odometry_result")

  result = []
  for _, message in enumerate(messages):
    timestamp = int(int(message[1].timestamp) / 1000)
    T_w_r_vec = np.array(message[1].t_world_robot.xi)[..., None]
    T_w_r = se3op.vec2tran(T_w_r_vec)
    T_w_a = T_w_r @ T_robot_applanix
    T_a_w_res = get_inverse_tf(T_w_a).flatten().tolist()[:12]
    result.append([timestamp] + T_a_w_res)

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

  if velocity:
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

      # Transform velocity from robot to applanix frame/origin (lidar results are in applanix frame)
      w_a_v_applanix = - se3op.tranAd(get_inverse_tf(T_robot_applanix)) @ w_v_r_robot.reshape(6, 1)

      timestamp = int(int(message[0]) / 1000)
      vel_results.append([timestamp] + w_a_v_applanix.flatten().tolist())

    output_dir = osp.join(result_dir, "odometry_vel_result")
    os.makedirs(output_dir, exist_ok=True)
    with open(osp.join(output_dir, odo_input + ".txt"), "+w") as file:
      writer = csv.writer(file, delimiter=' ')
      writer.writerows(vel_results)
      print("Written to file:", osp.join(output_dir, odo_input + ".txt"))

    output_dir = osp.join(result_dir, "../odometry_vel_result")
    os.makedirs(output_dir, exist_ok=True)
    with open(osp.join(output_dir, odo_input + ".txt"), "+w") as file:
      writer = csv.writer(file, delimiter=' ')
      writer.writerows(vel_results)
      print("Written to file:", osp.join(output_dir, odo_input + ".txt"))


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