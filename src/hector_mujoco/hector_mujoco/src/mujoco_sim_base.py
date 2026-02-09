import mujoco.viewer
import numpy as np
import mujoco
import time

class MujocoSimBase:

    def __init__(
                self, 
                model_path, 
                headless=False,
                viewer_fps=24,
                ):
        # Load the model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.world_root_constraint_id = self.obj_name2id('world_root', type='equality')
        self.base_pos_nominal = np.array([0.0, 0.0, 0.55])
        self.data = mujoco.MjData(self.model)
        self.headless = headless
        self.start_time = time.time()
        self.viewer_sync_rate = 1.0 / viewer_fps

        if self.headless:
            self.step = self.step_headless
        else:
            self.viewer = mujoco.viewer.launch_passive(
                                                        self.model, 
                                                        self.data,
                                                        show_left_ui=False,
                                                        show_right_ui=False,
                                                        key_callback=self.viewer_key_callback,
                                                        ) 
            self.viewer_pause = True
            self.step = self.step_head
            
        
        self.step_count = 0
        self.frame_skip = max(1, int(self.viewer_sync_rate / self.model.opt.timestep) )
        


    def viewer_key_callback(self,keycode):
        if chr(keycode) == ' ':
            self.viewer_pause = not self.viewer_pause
            print("[INFO] sim pause:",self.viewer_pause)
        elif chr(keycode) == 'E':
            self.viewer.opt.frame = not self.viewer.opt.frame
        elif chr(keycode) == 'Q':
            self.toggle_robot_on_off_ground()
        elif chr(keycode) == 'ĉ':            
            self.model.eq_data[self.world_root_constraint_id][5] -= 0.1
            print("[INFO] hanging height increasd to ",abs(self.model.eq_data[self.world_root_constraint_id][5]))
        elif chr(keycode) == 'Ĉ':
            self.model.eq_data[self.world_root_constraint_id][5] += 0.1
            print("[INFO] hanging height decreasd to ",abs(self.model.eq_data[self.world_root_constraint_id][5]))

        # print("[INFO] key pressed:",chr(keycode))


    def step_headless(self):
        mujoco.mj_step(self.model, self.data)

    def reset(self):
        mujoco.mj_resetData(self.model,self.data) 
        mujoco.mj_forward(self.model, self.data)
        self.viewer.sync()
        self.viewer_pause = False
        print("[INFO] sim pause:",self.viewer_pause)


    def step_head(self):
        if self.viewer.is_running():

            mujoco.mj_step(self.model, self.data)

            self.step_count = (self.step_count + 1) % self.frame_skip
            if self.step_count == 0:
                self.viewer.sync()
        else:
            exit()

    def obj_name2id(self,name,type='body'):
        type = type.upper()
        return mujoco.mj_name2id(
                                    self.model,
                                    getattr(mujoco.mjtObj, 'mjOBJ_'+type), 
                                    name
                                )

    def obj_id2name(self,obj_id,type='body'):
        type = type.upper() 
        return mujoco.mj_id2name(
                                    self.model,
                                    getattr(mujoco.mjtObj, 'mjOBJ_'+type), 
                                    obj_id
                                )

    def get_sensordata_from_id(self,sensor_id):
            
        start_n = self.model.sensor_adr[sensor_id]
    
        if sensor_id == self.model.nsensor -1:
            return self.data.sensordata[start_n:]
        else:
            end_n = self.model.sensor_adr[sensor_id+1]
            return self.data.sensordata[start_n:end_n]

    def contact_bw_bodies(
                            self,
                            body1,
                            body2,
                          ):

        geoms_body1_ids = []        
        if body1 == 'floor':
            geoms_body1_ids.append(self.obj_name2id(name='floor',type='geom'))
        else:
            body1_id = self.obj_name2id(name=body1,type='body')
            
            for geom_id,body_id in enumerate(self.model.geom_bodyid):
                if body_id == body1_id:
                    geoms_body1_ids.append(geom_id)

        geoms_body2_ids = []
        if body2 == 'floor':
            geoms_body2_ids.append(self.obj_name2id(name='floor',type='geom'))
        else:
            body2_id = self.obj_name2id(name=body2,type='body')
            for geom_id,body_id in enumerate(self.model.geom_bodyid):
                if body_id == body2_id:
                    geoms_body2_ids.append(geom_id)
        
        for n in range(self.data.ncon):
            contact = self.data.contact[n]
            if (contact.geom1 in geoms_body1_ids and contact.geom2 in geoms_body2_ids) or \
               (contact.geom2 in geoms_body1_ids and contact.geom1 in geoms_body2_ids) :
                # to make the function name-order agnostic

                return True,contact
        return False,None


    def toggle_robot_on_off_ground(self):
        init_qp = np.array(self.model.keyframe('home').qpos)
        if self.model.eq_active0[self.world_root_constraint_id] and self.data.eq_active[self.world_root_constraint_id]:
            print("[INFO] setting robot on ground")
            self.data.qpos[:] = init_qp
            self.data.qvel[:] = 0.0
            self.model.eq_active0[self.world_root_constraint_id] = 0
            self.data.eq_active[self.world_root_constraint_id] = 0

            self.viewer.sync()
            # self.viewer_pause = True
            # print("[INFO] sim pause:",self.viewer_pause)
        else:
            init_qp[2] = 0.6
            print("[INFO] setting robot off ground")
            self.data.qpos[:] = init_qp
            self.data.qvel[:] = 0.0
            self.model.eq_active0[self.world_root_constraint_id] = 1
            self.data.eq_active[self.world_root_constraint_id] = 1



