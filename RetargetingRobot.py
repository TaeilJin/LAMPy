import pybullet
import pybullet_data as pd
import tensorflow.compat.v1 as tf
import time
import numpy as np
from tkinter import filedialog

import sys
sys.path.append('D:/TJ_develop/LAMPy/')

from PyBulletRobot import RobotConfig
from PyBulletRobot import PyBulletUtils
from PyBulletRobot import PBR




rb_config = RobotConfig()
rb_utils = PyBulletUtils()
  


rb_config.URDF_FILENAME = filedialog.askopenfilename(title="MARU_URDF")
source_motion_FILENAME = filedialog.askopenfilename(title="MARU_URDF")
def main(argv):
    """
    PyBullet-based Robot Importing and Visualization
    """
    p = pybullet
    p.connect(p.GUI, options="--width=1920 --height=1080 --mp4=\"test.mp4\" --mp4fps=60")
    pybullet.setAdditionalSearchPath(pd.getDataPath())
    #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
    num_frames = 0
    while rb_utils.running:
        """ Update & Visualization
        """
        #--- [Base] for FPS throttling & initialization of PyBullet Viewer
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, 0)
        ground = pybullet.loadURDF(rb_config.GROUND_URDF_FILENAME)
            
        #--- [Base] Stop Visualization using Delete key-board event
        if rb_utils.check_delete_key(p):
            rb_utils.running = False

        #--- [Base] Init Setup & Load Data
        if rb_utils.check_Init_key(p):
            rb_utils.init = True
       

        if rb_utils.init == True:
            # Set robot to default pose to bias knees in the right direction.
            robot = pybullet.loadURDF(rb_config.URDF_FILENAME, rb_config.INIT_POS, rb_config.INIT_ROT)
            PBR_MARU = PBR(robot)
            p.removeAllUserDebugItems()
            
            mocap_motion =["Human motion", source_motion_FILENAME, 1,200]
            joint_pos_data = rb_utils.load_ref_data(mocap_motion[1],mocap_motion[2],mocap_motion[3])
            num_frames = joint_pos_data.shape[0]
            num_markers = joint_pos_data.shape[-1] // rb_config.POS_SIZE
            marker_ids = rb_config.build_markers(num_markers)
            
            f = 0
            rb_utils.data = True
            rb_utils.init = False
        
        #p.removeAllUserDebugItems()
            
        if rb_utils.data == True:
            f = 0

            # Update Routine
            for repeat in range(num_frames):
                time_start = time.time()
                
                f_idx = f % num_frames

                print("Frame {:d}".format(f_idx))
                
                ref_joint_pos = joint_pos_data[f_idx]
                ref_joint_pos = np.reshape(ref_joint_pos, [-1, rb_config.POS_SIZE])
                ref_joint_pos = rb_config.process_ref_joint_pos_data(ref_joint_pos)
                rb_config.set_maker_pos(ref_joint_pos, marker_ids)

                PBR_MARU.set_pose(np.concatenate([ref_joint_pos[0], rb_config.INIT_ROT, rb_config.DEFAULT_JOINT_POSE]))

                f += 1

                #--- [Base] Visualization
                pybullet.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
                #pybullet.stepSimulation()
                #--- [Base] FPS throttling
                time_end = time.time()
                sleep_dur = rb_utils.FRAME_DURATION - (time_end - time_start)
                sleep_dur = max(0, sleep_dur)
                time.sleep(sleep_dur)
            
            for m in marker_ids:
                p.removeBody(m)
            
            rb_utils.data = False
            marker_ids=[]
        #--- [Base] Visualization
        pybullet.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
        rb_utils.init = False
        
        
        

    pybullet.disconnect()
if __name__ == "__main__":
  tf.app.run(main)
  