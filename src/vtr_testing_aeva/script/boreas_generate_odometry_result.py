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


def main(dataset_dir, result_dir):
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

  # TODO: robot frame should be at rear-axle of the vehicle, update this!

  T_applanix_aeva = dataset_odo.sequences[0].calib.T_applanix_aeva
  T_robot_applanix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
  
  T_robot_aeva = T_robot_applanix @ T_applanix_aeva
  
  T_s_v = np.array([[0.9999366830849237, 0.008341717781538466, 0.0075534496251198685, -1.0119098938516395],
                    [-0.008341717774127972, 0.9999652112886684, -3.150635091210066e-05, -0.3965882433517194],
                    [-0.007553449599178521, -3.1504388681967066e-05, 0.9999714717963843, -1.697000000000001],
                    [0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]).astype(np.float64)
  
  print("T_applanix_aeva:\n", T_applanix_aeva)
  print("T_robot_aeva should be:\n", T_robot_aeva)  # robot is applanix frame
  print("T_aeva_vehicle should be:\n", T_s_v)       # vehicle is rear axle

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


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--dataset', default=os.getcwd(), type=str, help='path to boreas dataset (contains boreas-*)')
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.dataset, args.path)