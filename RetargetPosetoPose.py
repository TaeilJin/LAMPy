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
env_source_info = read_env_info_from_file("Data/hc_hd_for_TJ/25_World/hc_hd_bl_1.txt")
env_source_keyjoints_info = read_env_info_from_file("Data/hc_hd_for_TJ/15_World/hc_hd_bl_1.txt")

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
            th.Variable(torch.zeros((nFrames,mbs._num_joints * 3 + 3)),name="q")
            ]

        # 3. objetive setting
        obj_dim_dir = 24*3
        obj_dim_key = 15*3
        obj_dim_res = 25*3
        cost_function = th.AutoDiffCostFunction(
        optim_vars, self._error_fn, obj_dim_dir + obj_dim_key + obj_dim_res , aux_vars=aux_vars, cost_weight=th.ScaleCostWeight(1.0)
        )
        objective.add(cost_function)
        self.layer = th.TheseusLayer(th.LevenbergMarquardt(objective, max_iterations=15,abs_err_tolerance=1e-3,step_size=0.7))

    def _error_fn(self,optim_vars, aux_vars):
        
        (theta,) = optim_vars
        (tar_pose, tar_keypose, residual) = aux_vars
        
        # mbs update
        mbs.setFromCompactArray(theta.tensor)
        mbs.updateKinematicsUptoPos()

        # Cost Function
        
        # selected body 
        l_b = [ 0, 1, 3, 4, 6, 8, 9, 13, 15, 16, 17, 20, 22, 23, 24]
        
        target_dir = abs((tar_pose[:,1] - tar_pose[:,0]) - (mbs._joints[1]._wmat[:,:,3] - mbs._joints[0]._wmat[:,:,3]))
        for index, parent_j in enumerate(mbs.getParents()):
            if parent_j != -1 and index != 1 :
                if index == 2 and index == 8 and index == 12 and index==14 and index==18 and index==19 and index==21 :
                    target_dir = torch.cat([target_dir,abs((tar_pose[:,index] - tar_pose[:,parent_j]) - (mbs._joints[index]._wmat[:,:,3] - mbs._joints[parent_j]._wmat[:,:,3]))],dim=-1) *0.0
                else:
                    target_dir = torch.cat([target_dir,abs((tar_pose[:,index] - tar_pose[:,parent_j]) - (mbs._joints[index]._wmat[:,:,3] - mbs._joints[parent_j]._wmat[:,:,3]))],dim=-1)*0.0    
        target_dir = target_dir.reshape(-1,24*3)


        c_env = abs(mbs._joints[l_b[0]]._wmat[:,:,3] - tar_keypose[:,0])
        
        for i in range(1,len(l_b)):
            c_env = torch.cat([c_env,abs(mbs._joints[l_b[i]]._wmat[:,:,3] - tar_keypose[:,i])],dim=-1)
        
        # residual
        c_res = abs(theta[:,3:] - residual[:,3:]) *0.1
        return torch.cat([target_dir,c_env,c_res],dim=-1)
    
    def forward(self, targets):
        solution, info = self.layer.forward(
        input_tensors=targets,
        optimizer_kwargs={"backward_mode": "implicit"},
        )
        print(f"Optim info : {info.status} Optim error:", info.last_err.item())
        return solution["theta_opt"]    


# 4. optimization routine
optim_layer = OptModel_YW(1,mbs)

world_positions = np.zeros((target_k_pose.shape[0],25,3))
res = mbs.getCompactArray()
for s_f in range(0,world_positions.shape[0]):
    
    
    res[s_f:s_f+1,:3] = target_w_pose[s_f:s_f+1,0,:]
    inputs = {"theta_opt": res, 
                "targeted_wposition" : target_w_pose[s_f:s_f+1], 
                "targeted_kposition" :target_k_pose[s_f:s_f+1],
                "q" :(res)
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

np_envs = env_source_keyjoints_info[:,180:180+540*3].reshape(-1,540,3).numpy()
np_importance = env_source_keyjoints_info[:,-6*540:]
np_importance = np_importance[:,-540:] # base 

# np_imp의 값이 0.5 이하인 인덱스 찾기
indices = np.where(np_importance <= 0.5)

# 인덱스를 사용하여 np_envs 배열의 값을 변경
for i, j in zip(indices[0], indices[1]):
    np_envs[i, j] = np.ones(3) * 10  # 혹은 다른 값을 할당할 수도 있습니다.

vis.plot_animation_env_pose(world_positions,np_targets,np_envs,mbs.getParents(),
                            target_for=None, target_up=None,
                            filename="env_partial",axis_scale=1)

