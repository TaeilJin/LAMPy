from Model.core import Model
from Model.SPACE.Ours import builder_Space
from Model.SPACE.generator import GenerateFunc
from PoseProcessing import PoseProcess
from VisualData.skel import SkeletonInfo

import datetime
import numpy as np
import os
# model building
model = Model('hparams/Space.json',datetime)
built = model.buildmodel(builder_Space)

# visualization
vis =SkeletonInfo()

process = PoseProcess()
gen = GenerateFunc()



# testing data
item = f"{model.test_dir}/Walk To Stop.fbx.npz"
datas = np.load(item)['clips'].astype(np.float32)

# generate data 
scaled_joints = process.Normalize(datas[:,:75],model.scaler,0,75)
scaled_vel = process.Normalize(datas[:,-3:],model.scaler,0,3)
surfaces = datas[:,75 + 630*3:75 + 630*3 + 1620]
lengths = datas[:,75 + 630*3 + 1620:75 + 630*3 + 1620+24]
env_all = datas[:,75+ 630*2:75+630*3]

#env_all , ref_all, vel_all = gen.generate_sample_withRef(built['graph'],scaled_joints[np.newaxis,...],scaled_vel[np.newaxis,...],\
#                                                         lengths[np.newaxis,...],surfaces[np.newaxis,...],model.scaler)

a,file = os.path.splitext(os.path.basename(item))

#np.savez(f'{os.path.dirname(item)}/{file}' + "env.npz", clips=env_all)
#env_all = np.load(f'{os.path.dirname(item)}/{file}' + "env.npz")['clips'].astype(np.float32)
# 
#unscaled_joints = process.unNormalize(ref_all,mtest.scaler,0,75)
#unscaled_vel = process.unNormalize(vel_all,mtest.scaler,0,3)
#world_joints, trajectory = process.gen_world_pos_data(datas[:,:75],datas[:,-3:])

# visualizing
#vis.mixamo_SkelInfo()
#vis.plot_animation_withTR(world_joints, trajectory, vis.parents,filename= f'{item}',axis_scale=3.0)

import TCP as Core_tcp
from MBS.retarget import MBS_Retarget
import json
import ctypes

tcp = Core_tcp.TCP_Connection('143.248.6.198',80)
#mbs = MBS_Retarget()

while True:
    tcp.recieve()

    if not tcp.data:
        break

    try :
        tcp.recieve_Json()
    except(json.JSONDecodeError) as e:
        print(f"예외 발생 : {e}")

    if(tcp.data_packet['text_indicator'] == "Space"):
        print("Space is came !")
        i = tcp.data_packet['nFrame'] 
        print(i)
        # float 배열을 바이트로 변환
        float_output_array = env_all[i,:].flatten().astype(ctypes.c_float)
        
        tcp.send(float_output_array)

tcp.client_socket.close()