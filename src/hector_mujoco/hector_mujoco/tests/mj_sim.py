import sys
sys.path.append('./')
# import sys
import os
# Add the parent directory to the Python path
corepath = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(corepath)
from hector_mujoco.src.mujoco_sim_base import MujocoSimBase
import yaml
import numpy as np
import argparse 

if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--model', type=str, default='models/robot.xml')
    argparser.add_argument('--conf_path', type=str, help='Path to the configuration file', default='/home/lkrajan/ros2/hector_ws/src/hector_mujoco/hector_mujoco/config/hector_v2.yml')
    argparser.add_argument('--headless', default=False,action='store_true', help='Run the simulation in headless mode')
    args = argparser.parse_args()

    
    # load the yaml file
    conf = yaml.load(open(args.conf_path, 'r'), Loader=yaml.FullLoader)
    conf['sim']['model_path'] = corepath + '/' + conf['sim']['model_path']

    # over write yaml with cli args
    conf['sim']['headless'] = args.headless

    # create the simulation object
    sim = MujocoSimBase(**conf['sim'])
    
    
    sim.reset()
    while True:
        if not sim.viewer_pause:
            sim.step()
            print('l_toe:',sim.contact_bw_bodies('l_toe','floor')[0])
            print('r_toe:',sim.contact_bw_bodies('r_toe','floor')[0])

