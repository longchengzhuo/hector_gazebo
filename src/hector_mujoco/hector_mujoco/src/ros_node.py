import os
import rclpy
from rclpy.node import Node
import argparse
import threading
import importlib
import yaml
import time


from .mujoco_sim_base import MujocoSimBase

from laser_interfaces.msg import RobotState, RobotCommand, ContactState


class RosNode(Node):
    def __init__(self):
        super().__init__('hector_mujoco')
        self.robot_state_publisher = self.create_publisher(RobotState, 'Hector_State', 10)
        self.command_subscription = self.create_subscription(RobotCommand, 'Hector_Command', self.command_callback, 10)
        self.toe_contact_publisher = self.create_publisher(ContactState, 'true_toe_floor_contact', 10)
        self.robot_command = RobotCommand()
        self.rate = self.create_rate(1000)  # 1000hz

    def publish_robot_state(self, robot_state, toe_floor_cs):
        self.robot_state_publisher.publish(robot_state)
        self.toe_contact_publisher.publish(toe_floor_cs)

        

    def command_callback(self, msg):
        self.robot_command = msg
        self._is_command_callback_called = True

    def check_topic_publishers(self, topic_name):
        # Get the list of publishers for the topic
        publishers_info = self.get_publishers_info_by_topic(topic_name)
        
        if publishers_info:
            pass
            # self.get_logger().info(f'Topic "{topic_name}" is being published by {len(publishers_info)} publishers.')
        else:
            # self.get_logger().info(f'Topic "{topic_name}" is not being published.')
            for i in range(len(self.robot_command.motor_command)):
                self.robot_command.motor_command[i].kp = 0.0
                self.robot_command.motor_command[i].kd = 0.1
                self.robot_command.motor_command[i].tau = 0.0
                self.robot_command.motor_command[i].q = 0.0
                self.robot_command.motor_command[i].dq = 0.0



def main(args=None):

    
    corepath = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    rclpy.init(args=args)
    argparser = argparse.ArgumentParser(description='Run the simulation')
    config_path = os.path.join(corepath, 'config', 'hector_v2.yml')
    argparser.add_argument('--conf_path', type=str, help='Path to the configuration file', default=config_path)
    argparser.add_argument('--headless', default=False,action='store_true', help='Run the simulation in headless mode')
    args = argparser.parse_args()

    hector_mujoco = RosNode()

    # load the yaml file
    conf = yaml.load(open(args.conf_path, 'r'), Loader=yaml.FullLoader)

    # over write yaml with cli args
    conf['sim']['model_path'] = corepath + '/' + conf['sim']['model_path']
    conf['sim']['headless'] = args.headless

    # create the simulation object
    sim = MujocoSimBase(**conf['sim'])


    # import the robot robot class
    robot_class_name = conf['robot']['class_entry'].split('.')[-1]
    robot_file_entry = '.'.join(conf['robot']['class_entry'].split('.')[:-1])
    robot_class = importlib.import_module(robot_file_entry)
    # pop entry from conf
    conf['robot'].pop('class_entry')
    # create the robot object
    robot = getattr(robot_class, robot_class_name)(sim, **conf['robot'])

    # main thread publishes state and thread 1 subscribes to command
    # thread 1
    sub_thread = threading.Thread(target=rclpy.spin, args=(hector_mujoco,))    
    sub_thread.start()

    sim.reset()
    robot_state = robot.data2state()
    toe_floor_cs = ContactState()
    while True:
        # if sim.headless or (sim.viewer.is_running() and not sim.viewer_pause):
        if not sim.viewer_pause:
            hector_mujoco.check_topic_publishers('/Hector_Command')
            robot.command2ctrl(hector_mujoco.robot_command)
            sim.step()
            robot_state = robot.data2state()
            toe_floor_cs = ContactState()
            l_toe_floor_contact = sim.contact_bw_bodies('l_toe','floor')[0]
            r_toe_floor_contact = sim.contact_bw_bodies('r_toe','floor')[0]
            toe_floor_cs.contact_state[0] = l_toe_floor_contact
            toe_floor_cs.contact_state[1] = r_toe_floor_contact
        hector_mujoco.publish_robot_state(robot_state, toe_floor_cs)
        hector_mujoco.rate.sleep()

    rclpy.shutdown()
    sub_thread.join()
    pub_thread.join()

if __name__ == '__main__':
    main()