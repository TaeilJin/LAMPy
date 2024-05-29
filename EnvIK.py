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

# IK
# 1. cost function
def targeted_pose_error(optim_vars, aux_vars):
    (theta,) = optim_vars
    (tar_pose, tar_for, tar_up, residual) = aux_vars
    
    # mbs update
    mbs.setFromCompactArray(theta.tensor)
    mbs.updateKinematicsUptoPos()

    # Cost Function
    
    # selected body 
    l_b = [ 0, 1, 3, 4, 6, 8, 9, 13, 15, 16, 17, 20, 22, 23, 24]
    
    c_env = abs(mbs._joints[l_b[0]]._wmat[:,:,3] - tar_pose[:,0])
    c_env_f = abs(mbs._joints[l_b[0]]._wmat[:,:,2] - tar_for[:,0]) * 0.0
    c_env_u = abs(mbs._joints[l_b[0]]._wmat[:,:,1] - tar_up[:,0])

    for i in range(1,len(l_b)):
        c_env = torch.cat([c_env,abs(mbs._joints[l_b[i]]._wmat[:,:,3] - tar_pose[:,i])],dim=-1)
        c_env_f = torch.cat([c_env_f,abs(mbs._joints[l_b[i]]._wmat[:,:,2] - tar_for[:,i])],dim=-1) *0.0
        if(l_b[i] == 1 or l_b[i] == 6 or l_b[i] == 8 or l_b[i] == 4):
            c_env_u = torch.cat([c_env_u,abs(mbs._joints[l_b[i]]._wmat[:,:,1] - tar_up[:,i])],dim=-1) * 10.0
        else:
            c_env_u = torch.cat([c_env_u,abs(mbs._joints[l_b[i]]._wmat[:,:,1] - tar_up[:,i])],dim=-1) * 0.0
    
    # residual
    # selected body 
    l_ub = [ 18,14,13]
    theta_3  = theta.tensor[:,3:].reshape(-1,25,3)
    res_3 = residual.tensor[:,3:].reshape(-1,25,3)
    c_res = abs(theta_3[:,l_ub] - res_3[:,l_ub])*0.0
    c_res = c_res.reshape(-1,3*3)
    return torch.cat([c_env,c_env_f,c_env_u,c_res],dim=-1)

# 2. initial value 
# 환경 데이터 불러오기
#env_info = read_env_info_from_file("hc_hd_bl_1.txt")
env_info2 = read_env_info_from_file("hcw_hdw_world.txt")
target_poses = env_info2[:,:15*9].reshape(-1,15,9)

target_positions = target_poses[:,:,:3]
target_forward = target_poses[:,:,3:6]
target_up = target_poses[:,:,6:9]

#target_positions = env_info[:,:15*3].reshape(-1,15,3)
default_mat = default_mat.repeat(target_positions.shape[0], 1)

target_pose: torch.Tensor = target_positions
target_forward: torch.Tensor = target_forward
target_up: torch.Tensor = target_up
res : torch.Tensor = default_mat
aux_vars = [th.Variable(target_pose, name="targeted_position"), th.Variable(target_forward, name="targeted_forward"), th.Variable(target_up, name="targeted_up"), th.Variable(res,name="q")]

target_theta = torch.rand(target_positions.shape[0], mbs._dof, dtype=torch.float32)
theta_opt = torch.zeros_like(target_theta)
optim_vars = (th.Vector(tensor=(theta_opt), name="theta_opt"),)



# 3. optimization layer construction
objective = th.Objective()
cost_function = th.AutoDiffCostFunction(
    optim_vars, targeted_pose_error, 15*9 + 3*3 , aux_vars=aux_vars, cost_weight=th.ScaleCostWeight(1.0)
)
objective.add(cost_function)
layer = th.TheseusLayer(th.LevenbergMarquardt(objective, max_iterations=15,abs_err_tolerance=1e-3))

# 4. optimization routine
with torch.no_grad():
    default_mat[:,:3] = target_positions[:,0,:]
    inputs = {"theta_opt": default_mat, "targeted_pose": target_positions}
    solution, info = layer.forward(
            input_tensors=inputs,
            optimizer_kwargs={"backward_mode": "implicit"},
        )
    
# 5. output 
mbs.setFromCompactArray(solution["theta_opt"])
mbs.updateKinematicsUptoPos()
print("Outer loss: ", (mbs._joints[0]._wmat[:,:,3] - target_positions[:,0,:]).mean())
print(info.status)

# visualize
vis = Vis()
# mbs.setFromCompactArray(target_theta)
# mbs.updateKinematicsUptoPos()

mbs.getMotionMatrix()
world_positions = mbs._motion_mat[:,:,:,3].permute(1,0,2).numpy()

np_targets = target_positions.numpy()
np_targetforwards = target_forward.numpy()
np_targetup = target_up.numpy()

np_envs = env_info2[:,180:180+540*3].reshape(-1,540,3).numpy()
np_importance = env_info2[:,-6*540:]
np_importance = np_importance[:,-540:] # base 

# np_imp의 값이 0.5 이하인 인덱스 찾기
indices = np.where(np_importance <= 0.5)

# 인덱스를 사용하여 np_envs 배열의 값을 변경
for i, j in zip(indices[0], indices[1]):
    np_envs[i, j] = np.ones(3) * 10  # 혹은 다른 값을 할당할 수도 있습니다.

vis.plot_animation_env_pose(world_positions,np_targets,np_envs,mbs.getParents(),
                            np_targetforwards,np_targetup,
                            "env",axis_scale=1)

#new_arr = np.repeat(world_positions, repeats=np_targets.shape[0], axis=0)
