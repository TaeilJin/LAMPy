import pybullet
import numpy as np

class PBR():
    def __init__(self, robot):
        self.robot = robot
        self.ROOT_POS_DIM = 3
        self.ROOT_ROT_DIM = 4

    def get_root_pos(self,pose):
        return pose[0:self.ROOT_POS_DIM]
    
    def get_root_rot(self,pose):
        return pose[self.ROOT_POS_DIM:(self.ROOT_POS_DIM + self.ROOT_ROT_DIM)]

    def set_pose(self, pose):
        """set joint pose

        Args:
            pose (pose vector): [Root Pos, Root Rot, Whole Joint Rotation]
        """
        num_joints = pybullet.getNumJoints(self.robot)
        root_pos = self.get_root_pos(pose)
        root_rot = self.get_root_rot(pose)
        pybullet.resetBasePositionAndOrientation(self.robot, root_pos, root_rot)

        for j in range(num_joints):
            j_info = pybullet.getJointInfo(self.robot, j)
            j_state = pybullet.getJointStateMultiDof(self.robot, j)

            j_pose_idx = j_info[3]
            j_pose_size = len(j_state[0])
            j_vel_size = len(j_state[1])
            
            if (j_pose_size > 0):
                j_pose = pose[j_pose_idx:(j_pose_idx + j_pose_size)]
                j_vel = np.zeros(j_vel_size)
                joint_name = j_info[1].decode("utf-8")
                if( joint_name == "RA1"):
                    print(joint_name)
                    j_pose[0] = 45.0 * (3.14/180)
                pybullet.resetJointStateMultiDof(self.robot, j, j_pose, j_vel)

        return