import sys
sys.path.append('./')
from hector_mujoco.hector_mujoco.src.mujoco_sim_base import MujocoSimBase
from hector_mujoco.hector_mujoco.src.robot_base import RobotBase
from laser_interfaces.msg import RobotState, RobotCommand
import numpy as np
import yaml
import argparse
import importlib


def pretty_print_robot_state(state):
    print("torso imu:")
    print("\t quat:",state.imu[0].quaternion)
    print("\t acc:",state.imu[0].accelerometer)
    print("\t gyro:",state.imu[0].gyroscope)
    print("motor_state:")
    print("\t","q","dq","tauest")
    for i in range(len(state.motor_state)):
        print("\t",
            state.motor_state[i].q,
            state.motor_state[i].dq,
            state.motor_state[i].tauest
            )


if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--model', type=str, default='hector_mujoco/hector_mujoco/models/robot.xml')
    argparser.add_argument('--conf_path', type=str, help='Path to the configuration file', default='/home/lkrajan/ros2/hector_ws/src/hector_mujoco/hector_mujoco/config/hector_v2.yml')
    argparser.add_argument('--headless', default=False,action='store_true', help='Run the simulation in headless mode')
    args = argparser.parse_args()

    # load the yaml file
    conf = yaml.load(open(args.conf_path, 'r'), Loader=yaml.FullLoader)

    # over write yaml with cli args
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

    sim.reset()
    
    steps = 0
    CHANGE_MOTORS_EVERY = 2000
    max_steps = int(0.5*sim.model.nu)*CHANGE_MOTORS_EVERY
    print("max_steps:",max_steps)

    nominal_qpos = sim.model.keyframe('home').qpos
    nominal_jpos = nominal_qpos[7:] # it in in the order of config and command message



    sim.reset()
    while True:
        if not sim.viewer_pause:
            rcmd = RobotCommand()

            for i in range(sim.model.nu):
                rcmd.motor_command[i].q = nominal_jpos[i] + 0.01*np.sin(steps*0.01)
                rcmd.motor_command[i].dq = 0.0
                rcmd.motor_command[i].tau = 0.0
                if i in [0,1,2,3, 5,6,7,8]: # leg motors
                    rcmd.motor_command[i].kp = 33.0
                    rcmd.motor_command[i].kd = 0.5
                elif i in [4, 9]: # ankle motors
                    rcmd.motor_command[i].kp = 10.0
                    rcmd.motor_command[i].kd = 0.1
                else: # arm motors
                    rcmd.motor_command[i].kp = 5.0
                    rcmd.motor_command[i].kd = 0.1

            robot.command2ctrl(rcmd)

            sim.step()
            rstate = robot.data2state()
            sim.step()
            steps += 1
