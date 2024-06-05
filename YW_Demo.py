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
# source data
env_source_info: torch.Tensor = read_env_info_from_file("Data/hc_hd_for_TJ/Source_Extraction/lc_ld_nb_r_6.txt")
source_pose: torch.Tensor = env_source_info[10:,:15*9].reshape(-1,15,9)
source_posisition: torch.Tensor = source_pose[:,:,:3]
source_forward: torch.Tensor = source_pose[:,:,3:6]
source_up: torch.Tensor = source_pose[:,:,6:9]
source_surfacepoints : torch.Tensor = env_source_info[10:,168:168+540*3].reshape(-1,540,3)
source_importance: torch.Tensor = env_source_info[10:,3948:].reshape(-1,540,6)

# target data
target_importance: torch.Tensor = read_env_info_from_file("Data/hc_hd_for_TJ/Target_cont_Inference/Importance/lc_ld_nb_r_6.txt").reshape(-1,540,6)
target_surfacepoints: torch.Tensor = read_env_info_from_file("Data/hc_hd_for_TJ/Target_cont_Inference/env_Pos/lc_ld_nb_r_6.txt").reshape(-1,540,3)

target_pose: torch.Tensor = read_env_info_from_file("Data/hc_hd_for_TJ/Target_cont_Inference/Generated_Motion/lc_ld_nb_r_6.txt")
target_pose: torch.Tensor = target_pose.reshape(-1,15,9)
target_posisition: torch.Tensor = target_pose[:,:,:3]
target_forward: torch.Tensor = target_pose[:,:,3:6]
target_up: torch.Tensor = target_pose[:,:,6:9]

# data modification
source_posisition[:,:,0] = source_posisition[:,:,0]*-1
source_forward[:,:,0] = source_forward[:,:,0]*-1
source_up[:,:,0] = source_up[:,:,0]*-1
source_surfacepoints[:,:,0] = source_surfacepoints[:,:,0]*-1

target_posisition[:,:,0] = target_posisition[:,:,0]*-1
target_forward[:,:,0] = target_forward[:,:,0]*-1
target_up[:,:,0] = target_up[:,:,0]*-1
target_surfacepoints[:,:,0] = target_surfacepoints[:,:,0]*-1

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

# target_joint = torch.tensor([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
# srcsamples = torch.tensor([
#     [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]],
#     [[1.5, 2.5, 3.5], [4.5, 5.5, 6.5], [7.5, 8.5, 9.5]]
# ])

# corresponding index
threshold = 0.3
binary_tensor1 = (source_importance[:,:,2] >= threshold).int()
binary_tensor2 = (target_importance[:,:,2] >= threshold).int()
# 마지막 axis 값 비교
mask = (binary_tensor1 == binary_tensor2)
expanded_mask = mask.unsqueeze(-1).expand(-1, -1, 3)

masked_src_surfpoints = torch.where(expanded_mask, torch.tensor(1000.0), source_surfacepoints)
masked_tar_surfpoints = torch.where(expanded_mask, torch.tensor(1000.0), target_surfacepoints)


# 내가 원하는 Frame 에서만 변형을 진행한다.

target_posisition = torch.zeros_like(source_posisition)
#for j in range(0,15):
new_joint = calc_new_pos(source_posisition[:,14,:],masked_src_surfpoints,masked_tar_surfpoints)

target_posisition[:,14,:] = new_joint

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
