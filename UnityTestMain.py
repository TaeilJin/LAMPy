from Model.core import Model
from Model.SPACE.Ours import builder_Space
from PoseProcessing import PoseProcess
from Model.SPACE.generator import GenerateFunc
import TCP as Core_tcp
import json
import datetime
import torch
import numpy as np
import time
import ctypes

# model building
model = Model('hparams/Space.json',datetime)
built = model.buildmodel(builder_Space)
space_gen = GenerateFunc()

# tcp connection
tcp = Core_tcp.TCP_Connection('143.248.6.198',80)

#
process = PoseProcess()

while True:
    tcp.recieve(57)

    if not tcp.data:
        break

    try :
        tcp.recieve_Json()
    except(json.JSONDecodeError) as e:
        print(f"예외 발생 : {e}")

    if(tcp.data_packet['text_indicator'] == "Space"):
        #print("Space is came !")
        i = tcp.data_packet['nFrame'] 
        
        control = control_all[:,i:(i+seqlen+1),:]
        refpose = autoreg_all[:,(i+seqlen):(i+seqlen+1),:]
        surface = surface_all[:,(i+seqlen):(i+seqlen+1),:]
        surface = torch.from_numpy(np.swapaxes(surface,1,2))
        # prepare conditioning for moglow (control + previous poses)
        descriptor = space_gen.prepare_cond(autoreg.copy(), control.copy())
        #env = env_all[:,(i+seqlen):(i+seqlen+1),:]
        #env = torch.from_numpy(np.swapaxes(env,1,2)).to(self.data_device)
        len = len_all[:,(i+seqlen):(i+seqlen+1),:]
        len = torch.from_numpy(np.swapaxes(len,1,2))

        # condition (vel + env)
        cond = control_all[:,(i+seqlen):(i+seqlen+1),:]
        cond = torch.from_numpy(np.swapaxes(cond,1,2))
        cond = torch.cat((cond,len),dim=1)

        # descriptor (sequence + env)
        nBatch, nFeatures, nTimesteps = descriptor.shape
        descriptor = torch.cat((descriptor,len,surface),dim=1)

        # sample from Moglow
        sampled_z_label = torch.zeros((nBatch,630,1))

        #start = time.time()
        
        sampled = graph(z=sampled_z_label, cond=descriptor, eps_std=1.0, reverse=True)

        #print(" computation time: " , time.time()-start)

        sampled = sampled.cpu().numpy()[:,:,0]

        # store the sampled frames
        #sampled_all[:,(i+seqlen),:] = sampled # sampled
        # update saved pose sequence
        autoreg = np.concatenate((autoreg[:,1:,:].copy(), refpose), axis=1)

        # float 배열을 바이트로 변환
        float_output_array = sampled.flatten().astype(ctypes.c_float)
        
        tcp.send(float_output_array)
    

    if(tcp.data_packet['text_indicator'] == "initSpace"):
        print("initSpace")
        with torch.no_grad():
            graph = built['graph']

            # testing data
            item = f"{model.test_dir}/Walk To Stop.fbx.npz"
            datas = np.load(item)['clips'].astype(np.float32)

            # generate data 
            scaled_joints = process.Normalize(datas[:,:75],model.scaler,0,75)
            scaled_vel = process.Normalize(datas[:,-3:],model.scaler,0,3)
            
            surface_all = datas[np.newaxis,:,75 + 630*3:75 + 630*3 + 1620]
            len_all = datas[np.newaxis,:,75 + 630*3 + 1620:75 + 630*3 + 1620+24]
            env_all = datas[np.newaxis,:,75:75+630]

            autoreg_all = scaled_joints.copy()[np.newaxis,...]
            control_all = scaled_vel.copy()[np.newaxis,...]
            
            seqlen = 10
            
            # Initialize the pose sequence with ground truth test data
            
            # Initialize the lstm hidden state
            if hasattr(graph, "module"):
                graph.module.init_lstm_hidden()
            else:
                graph.init_lstm_hidden()
                
            nn,n_timesteps,n_feats = autoreg_all.shape # joints ref
            sampled_all = np.zeros((nn, n_timesteps, 630)) # env float
            #reference_all = np.zeros((nn, n_timesteps, n_feats)) # 
            autoreg = np.zeros((nn, seqlen, n_feats), dtype=np.float32) #initialize from a mean pose

    
            

    # if(tcp.data_packet['text_indicator'] == "doRetarget"):
    #     # float 배열 생성
    #     float_array = (ctypes.c_float * len(tcp.data_packet['floatArray']))(*tcp.data_packet['floatArray'])
    #     mbs.updateMBSPoseFromUnity(0,float_array)
        
    #     # do retarget
    #     base_offset = (ctypes.c_float * len(tcp.data_packet['base_offset']))(*tcp.data_packet['base_offset'])
    #     base_offset[0] = -base_offset[0] # left-hand coordinate to right-hand coordinate
    #     mbs.doRetarget(base_offset)

    #     # output mbs tar
    #     float_output_array = (ctypes.c_float * (mbs.numlinks*4+3))()
    #     mbs.exportMBSPoseFromUnity(1,float_output_array)

    #     # float 배열을 바이트로 변환
    #     tcp.send(float_output_array)

tcp.client_socket.close()