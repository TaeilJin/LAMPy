import numpy as np
import joblib
import os
import tkinter as tk
from tkinter import filedialog

from sklearn.preprocessing import StandardScaler
def select_folder():
    folder_path = filedialog.askdirectory()
    if folder_path:
        print(f"선택한 폴더 경로: {folder_path}")
    return folder_path
def find_files_in_folder(folder_path, keyword):
    file_list = []
    
    # 폴더 내의 모든 파일과 폴더 리스트를 가져옴
    items = os.listdir(folder_path)
    
    for item in items:
        item_path = os.path.join(folder_path, item)
        
        # 파일인지 확인
        if os.path.isfile(item_path):
            
            # 파일 이름에 특정 키워드가 포함되어 있는지 확인
            if (keyword in item and ".meta" not in item and ".npz" not in item):
                file_list.append(item_path)
    
    return file_list

def get_files(directory,keyword):
    return [os.path.join(directory,f) for f in sorted(list(os.listdir(directory)))
    if os.path.isfile(os.path.join(directory,f))
    and f.endswith(keyword)]

def loadExtractedTXTasNPZ(filename):

    Name, Extension =os.path.splitext(filename)
        
    if(os.path.exists(f"{Name}.npz")):
        datas = np.load(f"{Name}.npz")['clips'].astype(np.float32)
    else:
        datas = np.loadtxt(filename, delimiter=" ")
        np.savez_compressed(f"{Name}.npz", clips = datas)
        datas = np.load(f"{Name}.npz")['clips'].astype(np.float32)
    return datas

def SlicingForSequence(learningdata, window=81, window_step=40, Train=True):
    """ Slide over windows """
    windows = []
    
    #-window//8
    if(Train):
        for j in range(0, len(learningdata), window_step):
        
            """ If slice too small pad out by repeating start and end poses """
            slice = learningdata[j:j+window]
            
            if len(slice) < window:
                left  = slice[:1].repeat((window-len(slice))//2 + (window-len(slice))%2, axis=0)
                left[:,-(3+4):-(4)] = 0.0 # local position is stopped
                right = slice[-1:].repeat((window-len(slice))//2, axis=0)
                right[:,-(3+4):-(4)] = 0.0 # local position is stopped
                slice = np.concatenate([left, slice, right], axis=0)

            if len(slice) != window: raise Exception()
            
            windows.append(slice)
        """ update scaler """
    else:    
        windows.append(learningdata)
    
    return windows


def Normalize_motion(pose_data,scaler,dim=75):
    shape = pose_data.shape
    std = scaler.scale_[:dim]
    std[std<1e-3] = 1e-3
    
    if len(shape) == 2 : 
        scaled = (pose_data - scaler.mean_[:dim]) / std
    else:
        flat = pose_data.reshape((shape[0]*shape[1], shape[2]))
        scaled = (flat - scaler.mean_[:dim]) / std
        scaled = scaled.reshape(shape)
    return scaled.astype(np.float32)     

def Normalize_vel(vel_data,scaler):
    shape = vel_data.shape
    std = scaler.scale_[-3:]
    std[std<1e-3] = 1e-3
    if len(shape) == 2 : 
        scaled = (vel_data - scaler.mean_[-3:]) / std
    else:
        flat = vel_data.reshape((shape[0]*shape[1], shape[2]))
        scaled = (flat - scaler.mean_[-3:]) / std
        scaled = scaled.reshape(shape)
    return scaled.astype(np.float32)        

def unNormalize_motion(pose_data,scaler,dim=75):
    shape = pose_data.shape
    if len(shape) == 2 : 
        scaled = scaler.mean_[:dim] + pose_data * scaler.scale_[:dim]
    else:
        flat = pose_data.reshape((shape[0]*shape[1], shape[2]))
        scaled = scaler.mean_[:dim] + flat * scaler.scale_[:dim]
        scaled = scaled.reshape(shape)
    return scaled.astype(np.float32)

def unNormalize_vel(vel_data,scaler):
    shape = vel_data.shape
    if len(shape) == 2 : 
        scaled = scaler.mean_[-3:] + vel_data * scaler.scale_[-3:]
    else:
        flat = vel_data.reshape((shape[0]*shape[1], shape[2]))
        scaled = scaler.mean_[-3:] + flat * scaler.scale_[-3:]
        scaled = scaled.reshape(shape)
    return scaled.astype(np.float32)        

def partial_fit(data,scaler):
    shape = data.shape
    if(len(shape) == 3):
        flat = data.copy().reshape((shape[0]*shape[1], shape[2]))
    else:
        flat = data
    scaler.partial_fit(flat)
    
    return scaler

def standardize(data, scaler):
    shape = data.shape
    flat = data.copy().reshape((shape[0]*shape[1], shape[2]))
    scaled = scaler.transform(flat).reshape(shape)
    return scaled

def concat_sequence_3(seqlen, data):
    """ 
    Concatenates a sequence of features to one.
    """
    n_timesteps,n_feats = data.shape
    L = n_timesteps-(seqlen-1)
    inds = np.zeros((L, seqlen)).astype(int)

    #create indices for the sequences we want
    rng = np.arange(0, n_timesteps)
    for ii in range(0,seqlen):  
        inds[:, ii] = np.transpose(rng[ii:(n_timesteps-(seqlen-ii-1))])  

    #slice each sample into L sequences and store as new samples 
    cc=data[inds,:].copy()

    #print ("cc: " + str(cc.shape))

    #reshape all timesteps and features into one dimention per sample
    dd = cc.reshape((L, seqlen*n_feats))
    #print ("dd: " + str(dd.shape))
    return dd 

"""generate data of scaler of IITP example
"""

def gen_scaler_Of_IITP(pose_X,vel_X, scaler, data_root =None):
    # scaler update
    scaling_data = np.concatenate((pose_X,vel_X),axis=-1)
        
    # scaler partial fit
    scaler = partial_fit(scaling_data, scaler)
    
    # # scaler 저장하기
    joblib.dump(scaler,os.path.join(data_root,f'{data_root}/mixamo.pkl'))

    # scaler 불러오기
    scaler = joblib.load(os.path.join(data_root,f'{data_root}/mixamo.pkl'))
    
    return scaler

def Construct_SequentialData_IITP(seqlen,data,num_env,num_jointlength,num_vel,scaler):
        
    # sequence data joint(66) + ee(30)+ fcontact(2)+ env(2830) + vel(3) + label(1)
    joint_data = data[...,:-(num_env+num_jointlength+num_vel)].copy() # joint data
    joint_num = joint_data.shape[-1]
    if num_env is not 0 :
        env_data = data[...,(joint_num):(joint_num+ num_env)].copy() # label data
        len_data = data[...,(joint_num+ num_env):(joint_num+ num_env +num_jointlength)].copy() # label data
        vel_data = data[...,(joint_num+ num_env+num_jointlength):(joint_num+ num_env+num_jointlength +num_vel) ].copy()
    
    # scaling
    scaled_joint = Normalize_motion(joint_data, scaler, dim=75)
    scaled_vel = Normalize_vel(vel_data,scaler)

    # current pose (output)
    n_frames = scaled_joint.shape[0]
    current_x = concat_sequence_3(1, scaled_joint[seqlen:n_frames,:])
    current_env = concat_sequence_3(1, env_data[seqlen:n_frames,:])
    current_len = concat_sequence_3(1, len_data[seqlen:n_frames,:])

    # control autoreg(10) + control(11 or 1)
    current_withPast_vel = concat_sequence_3(seqlen +1, scaled_vel)
    current_vel = concat_sequence_3(1, scaled_vel[seqlen:n_frames,:])
  
    autoreg_seq = concat_sequence_3(seqlen,scaled_joint[:n_frames-1,:])
    PastWith_curVel = np.concatenate((autoreg_seq,current_withPast_vel),axis=-1)
    #autoreg_seq_single_control = np.concatenate((autoreg_seq,current_vel),axis=-1)
    
    return current_x, current_env,current_len,current_vel, PastWith_curVel

def ConstructData_IITP(folder_path):

    filenames = find_files_in_folder(folder_path, ".txt")

    # output folder
    create_dir_save_trainable_data = f'{folder_path}/npz'
    if not os.path.exists(create_dir_save_trainable_data):
        os.makedirs(create_dir_save_trainable_data)
    # scaler update
    scaler = StandardScaler()
    for i in range(len(filenames)):
        print(filenames[i])
        # data load
        datas = loadExtractedTXTasNPZ(filenames[i])
        # scaler update
        scaler = gen_scaler_Of_IITP(datas[...,74:74+75],datas[...,-3:],scaler,create_dir_save_trainable_data)
    mean_pos = scaler.mean_[:75]
    vars_pos = scaler.scale_[:75]
    mean_vel = scaler.mean_[75:]
    vars_vel = scaler.scale_[75:]
    np.savetxt(f'{create_dir_save_trainable_data}/mean_pos.txt',mean_pos,delimiter=" ")
    np.savetxt(f'{create_dir_save_trainable_data}/vars_pos.txt',vars_pos,delimiter=" ")
    np.savetxt(f'{create_dir_save_trainable_data}/mean_vel.txt',mean_vel,delimiter=" ")
    np.savetxt(f'{create_dir_save_trainable_data}/vars_vel.txt',vars_vel,delimiter=" ")
    
    # sequential data
    datafilecnt_train = 0
    files = get_files(folder_path,'.npz')
    for i in range(0,len(files)):
        data = np.load(files[i])['clips'].astype(np.float32)
        # 80 frames
        windows = SlicingForSequence(data,Train = True)
        
        #
        for window in windows: 
            current_x, current_env,current_len,current_vel, PastWith_curVel = Construct_SequentialData_IITP(10,window,num_env=2640,num_jointlength=24,num_vel=3,scaler=scaler)
            #
            for i in range(0,current_x.shape[0]):
                print("datafilecnt_train"+str(datafilecnt_train))
                np.savez_compressed(f'{create_dir_save_trainable_data}/scaled_x_{str(datafilecnt_train)}.npz', clips = current_x[i,...])
                np.savez_compressed(f'{create_dir_save_trainable_data}/scaled_env_{str(datafilecnt_train)}.npz', clips = current_env[i,...])
                np.savez_compressed(f'{create_dir_save_trainable_data}/scaled_len_{str(datafilecnt_train)}.npz', clips = current_len[i,...])
                
                np.savez_compressed(f'{create_dir_save_trainable_data}/scaled_vel_{str(datafilecnt_train)}.npz', clips = current_vel[i,...])
                np.savez_compressed(f'{create_dir_save_trainable_data}/scaled_velWithPast_{str(datafilecnt_train)}.npz', clips = PastWith_curVel[i,...])
            
                datafilecnt_train += 1
            
# GUI 창 생성
root = tk.Tk()
root.withdraw()  # 기본 창 숨기기

# "폴더 선택" 버튼 생성
selected_folder = select_folder()

ConstructData_IITP(selected_folder)