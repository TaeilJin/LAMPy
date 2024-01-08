import numpy as np
from scipy.spatial.transform import Rotation as R

class PoseProcess():
    
    def __init__(self):
        print('hello')
        
    def Normalize(self,pose_data,scaler, dim_start=0, size=75):
        shape = pose_data.shape
        std = scaler.scale_[dim_start:dim_start+ size]
        std[std<1e-3] = 1e-3
        
        if len(shape) == 2 : 
            scaled = (pose_data - scaler.mean_[dim_start:dim_start+ size]) / std
        else:
            flat = pose_data.reshape((shape[0]*shape[1], shape[2]))
            scaled = (flat - scaler.mean_[dim_start:dim_start+ size]) / std
            scaled = scaled.reshape(shape)
        return scaled.astype(np.float32)     

    def unNormalize(self,pose_data,scaler, dim_start=0, size=75):
        shape = pose_data.shape
        if len(shape) == 2 : 
            scaled = scaler.mean_[dim_start:dim_start+ size] + pose_data * scaler.scale_[dim_start:dim_start+ size]
        else:
            flat = pose_data.reshape((shape[0]*shape[1], shape[2]))
            scaled = scaler.mean_[dim_start:dim_start+ size] + flat * scaler.scale_[dim_start:dim_start+ size]
            scaled = scaled.reshape(shape)
        return scaled.astype(np.float32)


    def gen_world_pos_data(self,i_joints,i_rootvel):
        """
        Args:
        return joints, root trajectory of motion
        """
        joints = i_joints.copy()
        roots = i_rootvel.copy()

        rot = R.from_quat([0,0,0,1])
        translation = np.array([[0,0,0]])
        translations = np.zeros((joints.shape[0],3))
        
        root_dx, root_dz, root_dr = roots[...,-3], roots[...,-2], roots[...,-1]
        
        joints = joints.reshape((len(joints), -1, 3))
        for i in range(len(joints)):
            
            translation = translation + rot.apply(np.array([root_dx[i], 0, root_dz[i]]))
            rot = R.from_rotvec(np.array([0,-root_dr[i],0]))*rot
            
            joints[i,:,:] = rot.apply(joints[i])
            joints[i,:,0] = joints[i,:,0] + translation[0,0]
            joints[i,:,2] = joints[i,:,2] + translation[0,2]
                        
            
            translations[i,:] = translation
        
        return joints, translations

    