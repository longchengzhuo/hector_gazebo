
from laser_interfaces.msg import RobotState, RobotCommand
import numpy as np
import time

class RobotBase:
    def __init__(self, 
                 sim,
                 verbose=False,
                 command2actuator=None,
                 state2actuator=None,
                 ):

        self.sim = sim
        
        # low level actuator command to mujoco actuator
        if command2actuator is None:
            # default command is same order as actuator
            self.command2actuator = {i: sim.obj_id2name(i, type='actuator') for i in range(sim.model.nu)}
        else:
            # else provide a mapping
            self.command2actuator = command2actuator
        
        # low level actuator state to mujoco actuator
        if state2actuator is None:
            # default state is same order as actuator
            self.state2actuator = {i: sim.obj_id2name(i, type='actuator') for i in range(sim.model.nu)}
        else:
            # else provide a mapping
            self.state2actuator = state2actuator

        if verbose:
            print("command2actuator:", self.command2actuator)
            print("state2actuator:", self.state2actuator)

        self.command2actuator_id = {k: sim.obj_name2id(v, type='actuator') for k,v in self.command2actuator.items()}
        self.state2actuator_id = {k: sim.obj_name2id(v, type='actuator') for k,v in self.state2actuator.items()}
        
        # actuator and actuator sensor ids
        self.state2actuator_pos_sensor_id = {}
        self.state2actuator_vel_sensor_id = {}
        self.state2actuator_frc_sensor_id = {}

        for k,v in self.state2actuator.items():

            self.state2actuator_pos_sensor_id.update({k: sim.obj_name2id(v+'_pos', type='sensor')})
            self.state2actuator_vel_sensor_id.update({k: sim.obj_name2id(v+'_vel', type='sensor')})
            self.state2actuator_frc_sensor_id.update({k: sim.obj_name2id(v+'_frc', type='sensor')})



        # torso sensors
        self.torso_acc_sensor_id = sim.obj_name2id('torso_acc', type='sensor')
        self.torso_gyro_sensor_id = sim.obj_name2id('torso_gyr', type='sensor')
        self.torso_quat_sensor_id = sim.obj_name2id('torso_quat', type='sensor')
        self.torso_pos_sensor_id = sim.obj_name2id('torso_pos', type='sensor')
        self.torso_linvel_sensor_id = sim.obj_name2id('torso_linvel', type='sensor')
        

        # container for low level state
        self.curr_robot_state = RobotState()

        # verbose
        if verbose: 
            total_mass = 0  
            print("\nlinks: id, name, mass:")
            for body_id in range(sim.model.nbody):
                body_name = sim.obj_id2name(body_id, type='body')
                # set spacing with predefined max length for name and mass
                print("\t",body_id,body_name,':',round(sim.model.body_mass[body_id],4),"Kg")
                total_mass += sim.model.body_mass[body_id]
            print("total mass:", total_mass, "Kg")


            print("\njoints: id, name, type, range, qposadr, dofadr:")
            for joint_id in range(sim.model.njnt):
                joint_name = sim.obj_id2name(joint_id, type='joint')
                joint_type = sim.model.jnt_type[joint_id]
                joint_range = sim.model.jnt_range[joint_id]
                print("\t",
                    joint_id,
                    joint_name,
                    joint_type,
                    joint_range, 
                    sim.model.jnt_qposadr[joint_id], 
                    sim.model.jnt_dofadr[joint_id]
                    )
            # print("comand id: actuator name")
            # print(self.command2actuator)
            # print("state id: actuator name")
            # print(self.state2actuator)
            print("command id - > actuator id")
            for command_id, actuator_id in self.command2actuator_id.items():
                print("\tcommand_id:",command_id, "actuator_id:", actuator_id, "actuator_name:", self.command2actuator[command_id])
            print("state id - > actuator id")
            for state_id, actuator_id in self.state2actuator_id.items():
                print("\tstate_id:",state_id, "actuator_id:", actuator_id, "actuator_name:", self.state2actuator[state_id])
        
        self.world_root_constraint_id = self.sim.obj_name2id('world_root', type='equality')
        self.set_on_ground_height = np.array([0.0, 0.0, 0.55])

    
    def command2ctrl(self, cmd):
        
        for command_id, actuator_id in self.command2actuator_id.items():            

            
            if cmd.motor_command[command_id].kp != 0:
                # position and (velocity  and/or torque) control
                self.sim.model.actuator_gainprm[actuator_id, 0] =  cmd.motor_command[command_id].kp 
                self.sim.model.actuator_biasprm[actuator_id, 1] = -cmd.motor_command[command_id].kp
                self.sim.model.actuator_biasprm[actuator_id, 2] = -cmd.motor_command[command_id].kd
                # u = q_des + kd*dq_des/kp + tau_ff/kp        
                self.sim.data.ctrl[actuator_id] =  cmd.motor_command[command_id].q + \
                                                cmd.motor_command[command_id].kd*cmd.motor_command[command_id].dq/cmd.motor_command[command_id].kp + \
                                                cmd.motor_command[command_id].tau/cmd.motor_command[command_id].kp
            else:
                # (velocity  and/or torque) control only
                self.sim.model.actuator_gainprm[actuator_id, 0] =  1.0 # pass remaining directly
                self.sim.model.actuator_biasprm[actuator_id, 1] =  0.0
                self.sim.model.actuator_biasprm[actuator_id, 2] =  -cmd.motor_command[command_id].kd
                self.sim.data.ctrl[actuator_id] =  cmd.motor_command[command_id].kd*cmd.motor_command[command_id].dq + \
                                                   cmd.motor_command[command_id].tau   
        
    def data2state(self):
        
        # syn sensor data to state
        torso_acc = self.sim.get_sensordata_from_id(self.torso_acc_sensor_id)
        torso_gyr = self.sim.get_sensordata_from_id(self.torso_gyro_sensor_id)
        torso_quat = self.sim.get_sensordata_from_id(self.torso_quat_sensor_id)
        torso_pos = self.sim.get_sensordata_from_id(self.torso_pos_sensor_id)
        torso_linvel = self.sim.get_sensordata_from_id(self.torso_linvel_sensor_id)
        
        # update torso imu data, 0th imue assumed to be the torso imu
        for i in range(3):
            self.curr_robot_state.imu[0].quaternion[i] = torso_quat[i]
            self.curr_robot_state.imu[0].accelerometer[i] = torso_acc[i]
            self.curr_robot_state.imu[0].gyroscope[i] = torso_gyr[i]
            self.curr_robot_state.body_position[i] = torso_pos[i]
            self.curr_robot_state.body_velocity[i] = torso_linvel[i]
        self.curr_robot_state.imu[0].quaternion[3] = torso_quat[3]

        for k, v in self.state2actuator_pos_sensor_id.items():
            self.curr_robot_state.motor_state[k].q = self.sim.get_sensordata_from_id(v)[0]
        for k, v in self.state2actuator_vel_sensor_id.items():
            self.curr_robot_state.motor_state[k].dq = self.sim.get_sensordata_from_id(v)[0]
        for k, v in self.state2actuator_frc_sensor_id.items():
            self.curr_robot_state.motor_state[k].tauest = self.sim.get_sensordata_from_id(v)[0]

        return self.curr_robot_state
    
    def data2odom(self):
        # update torso position and velocity
        # torso_pos = self.sim.get_sensordata_from_id(self.torso_pos_sensor_id)
        torso_pos = self.sim.data.qpos[0:3]
        # torso_quat = self.sim.get_sensordata_from_id(self.torso_quat_sensor_id)
        torso_quat = self.sim.data.qpos[3:7]
        # torso_vel = self.sim.get_sensordata_from_id(self.torso_vel_sensor_id)
        torso_vel = self.sim.data.qvel[0:3]
        # torso_ang_vel = self.sim.get_sensordata_from_id(self.torso_gyro_sensor_id)
        torso_ang_vel = self.sim.data.qvel[3:6]
        
        # update odom data
        self.curr_odom.pose.pose.position.x = torso_pos[0]
        self.curr_odom.pose.pose.position.y = torso_pos[1]
        self.curr_odom.pose.pose.position.z = torso_pos[2] - 0.55
        
        self.curr_odom.pose.pose.orientation.w = torso_quat[0]
        self.curr_odom.pose.pose.orientation.x = torso_quat[1]
        self.curr_odom.pose.pose.orientation.y = torso_quat[2]
        self.curr_odom.pose.pose.orientation.z = torso_quat[3]
        
        self.curr_odom.twist.twist.linear.x = torso_vel[0]
        self.curr_odom.twist.twist.linear.y = torso_vel[1]
        self.curr_odom.twist.twist.linear.z = torso_vel[2]

        self.curr_odom.twist.twist.angular.x = torso_ang_vel[0]
        self.curr_odom.twist.twist.angular.y = torso_ang_vel[1]
        self.curr_odom.twist.twist.angular.z = torso_ang_vel[2]

        self.curr_odom.header.stamp.sec = int(time.time())
        self.curr_odom.header.stamp.nanosec = int((time.time() - int(time.time()))*1e9)
        self.curr_odom.header.frame_id = 'odom'
        self.curr_odom.child_frame_id = 'trunk'
        
        return self.curr_odom

    def data2acc(self):
        # update torso acceleration
        torso_acc = self.sim.get_sensordata_from_id(self.torso_acc_sensor_id)
        
        # update accel data
        self.curr_acc.linear.x = torso_acc[0]
        self.curr_acc.linear.y = torso_acc[1]
        self.curr_acc.linear.z = torso_acc[2]
        
        return self.curr_acc

    def set_robot_on_ground(self):
        self.sim.model.eq_active0[self.world_root_constraint_id] = 0
        self.sim.data.eq_active[self.world_root_constraint_id] = 0
        self.sim.data.qpos[2] = self.set_on_ground_height[2] + 0.01
        self.sim.data.qvel = 0.0
        self.sim.step()
        self.sim.viewer.sync()  

    def set_robot_off_ground(self):
        self.sim.data.qpos[:2] = 0.0
        self.sim.data.qpos[2] = 1.0
        self.sim.data.qpos[3] = 1.0
        self.sim.data.qpos[4:] = 0.0
        self.sim.data.qvel = 0.0
        for _ in range(5):
            self.sim.step()
        self.sim.model.eq_active0[self.world_root_constraint_id] = 1
        self.sim.data.eq_active[self.world_root_constraint_id] = 1
        for _ in range(10):
            self.sim.step()
        self.sim.viewer.sync()