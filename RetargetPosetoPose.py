import torch
import numpy as np
import theseus as th

from PyMBS import MBS
from VisualData import Vis

# .txt 파일에서 정보를 읽어옴
def read_env_info_from_file(file_path):
    
    with open(file_path, 'r') as file:
        lines = file.read().strip().split('\n')

    # 각 줄을 파싱하여 텐서 리스트 생성
    tensor_list = []
    for line in lines:
        data = line.split(" ")
        tensor_list.append(torch.tensor([float(item) for item in data]))

    # 모든 텐서의 크기가 동일한지 확인
    tensor_sizes = [tensor.size() for tensor in tensor_list]
    if not all(size == tensor_sizes[0] for size in tensor_sizes):
        raise ValueError("All tensors must have the same size.")

    # 텐서를 결합하여 하나의 행렬로 만듦
    combined_tensor = torch.stack(tensor_list)

    return combined_tensor


# MBS
mbs = MBS.from_mbs_file("RetargetedYbot.txt")
print(mbs.getParents())
mbs.updateKinematicsUptoPos()
default_mat = mbs.getCompactArray()

# 2. initial value 
# 환경 데이터 불러오기
env_source_info = read_env_info_from_file("Data/hc_hd_for_TJ/25_Relative/hc_hd_bl_1.txt")
env_source_keyjoints_info = read_env_info_from_file("Data/hc_hd_for_TJ/15_Relative/hc_hd_bl_1.txt")

# target source pose
target_whole_poses = env_source_info[:,:25*9].reshape(-1,25,9)
target_key_poses = env_source_keyjoints_info[:,:15*9].reshape(-1,15,9)
default_mat = default_mat.repeat(target_whole_poses.shape[0], 1)

# target variables
target_w_pose: torch.Tensor = target_whole_poses[:,:,:3]
target_w_forward: torch.Tensor = target_whole_poses[:,:,3:6]
target_w_up: torch.Tensor = target_whole_poses[:,:,6:9]

target_k_pose: torch.Tensor = target_key_poses[:,:,:3]
target_k_forward: torch.Tensor = target_key_poses[:,:,3:6]
target_k_up: torch.Tensor = target_key_poses[:,:,6:9]

# target keyjoint generation
surface_envs = env_source_keyjoints_info[:,180:180+540*3].reshape(-1,540,3)
np_importance = env_source_keyjoints_info[:,-6*540:]
np_importance = np_importance[:,-540:] # base 

surface_envs[:,:,0] = surface_envs[:,:,0]*-1

# data modification
target_w_pose[:,:,0] = target_w_pose[:,:,0]*-1
target_w_up[:,:,0] = target_w_up[:,:,0]*-1
target_w_forward[:,:,0] = target_w_forward[:,:,0]*-1

target_k_pose[:,:,0] = target_k_pose[:,:,0]*-1
target_k_up[:,:,0] = target_k_up[:,:,0]*-1
target_k_forward[:,:,0] = target_k_forward[:,:,0]*-1

# 3. setup the optimization layer
class OptModel_YW(torch.nn.Module):
    def __init__(self,nFrames, mbs):
        super().__init__()
        objective = th.Objective()
        # 1. optimizable variables
        optim_vars = [th.Vector(tensor=(torch.zeros((nFrames, mbs._dof),dtype=torch.float32)), name="theta_opt")]
        # 2. auxiliary variables
        aux_vars = [
            th.Variable(torch.zeros((nFrames,mbs._num_joints, 3)), name="targeted_wposition"), 
            th.Variable(torch.zeros((nFrames,15, 3)), name="targeted_kposition"), 
            th.Variable(torch.zeros((nFrames,mbs._num_joints * 3 + 3)),name="q"),
            th.Variable(torch.zeros((nFrames,15, 3)), name="targeted_kup"), 
            th.Variable(torch.zeros((nFrames,15, 3)), name="targeted_kfor")
            ]

        # 3. objetive setting
        obj_dim_dir = 24*3
        obj_dim_key = 15*3 + 15*3 + 15*3
        obj_dim_res = 25*3
        cost_function = th.AutoDiffCostFunction(
        optim_vars, self._error_fn, obj_dim_key + obj_dim_res , aux_vars=aux_vars, cost_weight=th.ScaleCostWeight(1.0)
        )
        objective.add(cost_function)
        self.layer = th.TheseusLayer(th.LevenbergMarquardt(objective, max_iterations=15,abs_err_tolerance=1e-3,step_size=0.7))

    def _error_fn(self,optim_vars, aux_vars):
        
        (theta,) = optim_vars
        (tar_pose, tar_keypose, residual, tar_keyup, tar_keyfor) = aux_vars
        
        # mbs update
        mbs.setFromCompactArray(theta.tensor)
        mbs.updateKinematicsUptoPos()

        # Cost Function

        # Selected body 
        l_b = [ 0, 1, 3, 4, 6, 8, 9, 13, 15, 16, 17, 20, 22, 23, 24]

        c_env = abs(mbs._joints[l_b[0]]._wmat[:,:,3] - tar_keypose[:,0])
        c_env_u = abs(mbs._joints[l_b[0]]._wmat[:,:,1] - tar_keyup[:,0])
        c_env_f = abs(mbs._joints[l_b[0]]._wmat[:,:,2] - tar_keyfor[:,0])
        for i in range(1,len(l_b)):
            w_p_weight = 1.0
            w_d_weight = 1.0
            c_env = torch.cat([c_env,abs(mbs._joints[l_b[i]]._wmat[:,:,3] - tar_keypose[:,i])],dim=-1) * w_p_weight
            c_env_u = torch.cat([c_env_u,abs(mbs._joints[l_b[i]]._wmat[:,:,1] - tar_keyup[:,i])],dim=-1) * w_d_weight
            c_env_f = torch.cat([c_env_f,abs(mbs._joints[l_b[i]]._wmat[:,:,2] - tar_keyfor[:,i])],dim=-1) * w_d_weight
            
        
        # residual
        c_res = abs(theta[:,3:] - residual[:,3:]) *0.1
        return torch.cat([c_env,c_res,c_env_u,c_env_f],dim=-1)
    
    def forward(self, targets):
        solution, info = self.layer.forward(
        input_tensors=targets,
        optimizer_kwargs={"backward_mode": "implicit"},
        )
        print(f"Optim info : {info.status} Optim error:", info.last_err.item())
        return solution["theta_opt"]    


# 4. optimization routine
optim_layer = OptModel_YW(1,mbs)
# 4.1. relative vector generation
# input
# target_joint : (nBatch, 3)
# surface points : (nBatch, nP, 3)
# output
# surface points weight : (nBatch,nP,1)
def calc_relative_weights(target_joint, samples):
    min_dist = 10000.0
    max_dist = 0.0
    sum_dist = 0.0
    
    src_sp_size = samples.shape[1]
    nBatch = target_joint.shape[0]

    mag = torch.zeros((nBatch, src_sp_size,1))
    direction = torch.zeros((nBatch, src_sp_size, 3))
    weight = torch.zeros((nBatch, src_sp_size,1))
    
    for j in range(src_sp_size):
        dir_vector = target_joint - samples[:, j]
        mg = torch.norm(dir_vector, dim=1)
        direction[:, j] = dir_vector / mg.unsqueeze(1)
        
        mag[:, j,0] = mg
    direction[torch.isnan(direction)] =0.0
    mask = mag[:] < 1e-3

    min_dist = torch.min(mag, dim=1).values
    max_dist = torch.max(mag, dim=1).values
    sum_dist = torch.sum(mag, dim=1)
    
    mid_dist = sum_dist / src_sp_size
    weightsum = torch.zeros((nBatch, 1))
    
    for k in range(src_sp_size):
        mask = mag[:, k] >= mid_dist
        weight[:, k][mask] = 1e-10
        mask = mag[:, k] <= min_dist
        weight[:, k][mask] = 1.0
        mask = (mag[:, k] > min_dist) & (mag[:, k] < mid_dist)
        weight[:, k][mask] = 1 - (mag[:, k][mask] - min_dist[mask]) / (mid_dist[mask] - min_dist[mask])
        
        
    weightsum = torch.sum(weight, dim=1)
    indices = torch.where(weightsum < 1e-3)
    weight = weight / weightsum.unsqueeze(1)
    weight[indices] = 0.0

    return direction, mag, weight

# 4.2. new position generation
def calc_new_pos(src_point, src_samples, tar_samples):
    directions, mags, weights = calc_relative_weights(src_point, src_samples)

    new_jnt_pos = torch.zeros((src_point.shape[0],3))
    for j in range((src_samples.shape[1])):
        new_pos = weights[:,j] * (directions[:,j] * mags[:,j] + tar_samples[:,j])
        new_jnt_pos = new_jnt_pos + new_pos

    tar_point = new_jnt_pos
    return tar_point

target_joint = torch.tensor([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
srcsamples = torch.tensor([
    [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]],
    [[1.5, 2.5, 3.5], [4.5, 5.5, 6.5], [7.5, 8.5, 9.5]]
])

target_envs = surface_envs.clone()
target_envs[:,:,0] = target_envs[:,:,0] + 0.3
for j in range(0,15):
    new_joint = calc_new_pos(target_k_pose[:,j,:],surface_envs,target_envs)

    target_k_pose[:,j,:] = new_joint

world_positions = np.zeros((target_k_pose.shape[0],25,3))
res = mbs.getCompactArray()
for s_f in range(0,world_positions.shape[0]):
    
    
    res[s_f:s_f+1,:3] = target_k_pose[s_f:s_f+1,0,:]
    inputs = {"theta_opt": res, 
                "targeted_wposition" : target_w_pose[s_f:s_f+1], 
                "targeted_kposition" :target_k_pose[s_f:s_f+1],
                "q" :(res),
                "targeted_kup" : target_k_up[s_f:s_f+1],
                "targeted_kfor" : target_k_forward[s_f:s_f+1]
                }
    output = optim_layer(inputs)

    # recursive
    res = output

    # 5. output 
    mbs.setFromCompactArray(output)
    mbs.updateKinematicsUptoPos()
    print("Outer loss: ", (mbs._joints[0]._wmat[:,:,3] - target_w_pose[s_f:s_f+1,0,:]).mean())

    mbs.getMotionMatrix()
    world_positions[s_f:s_f+1] = mbs._motion_mat[:,:,:,3].permute(1,0,2).numpy()


# 6. visualize
vis = Vis()


# 6.1 visualizing the conditions 
np_targets = target_k_pose.numpy()
np_targetforwards = target_k_forward.numpy()
np_targetup = target_k_up.numpy()
np_envs = surface_envs.numpy()
# np_imp의 값이 0.5 이하인 인덱스 찾기
indices = np.where(np_importance <= 0.5)

# 인덱스를 사용하여 np_envs 배열의 값을 변경
for i, j in zip(indices[0], indices[1]):
    np_envs[i, j] = np.ones(3) * 10  # 혹은 다른 값을 할당할 수도 있습니다.

vis.plot_animation_env_pose(world_positions,np_targets,np_envs,mbs.getParents(),
                            target_for=None, target_up=None,
                            filename="env_partial",axis_scale=1)

