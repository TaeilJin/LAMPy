import numpy as np
from pybullet_utils import transformations
import pybullet
from PyBulletRobot.utilities import pose3d

class RobotConfig:
    """
    Configuration class to store various default parameters related to URDF-based the robot
    """
    def __init__(self):
        # Ground URDF 
        self.GROUND_URDF_FILENAME = "plane_implicit.urdf"
        # URDF 파일 경로
        self.URDF_FILENAME = "robot_urdf/KIST MAHRU-WL_w_Battery.urdf"
        
        # 초기 위치 및 회전
        self.INIT_POS = np.array([0, 0, 1])
        self.INIT_ROT = transformations.quaternion_from_euler(ai=0, aj=0, ak=-np.pi, axes="sxyz")
        
        # 시뮬레이션 관련 조인트 및 오프셋
        self.SIM_TOE_JOINT_IDS = [5, 15, 10, 20]
        self.SIM_HIP_JOINT_IDS = [2, 12, 7, 17]
        self.SIM_TIP_JOINT_IDS = [4, 5, 13, 15, 9, 10, 18, 20, 1, 6]
        self.SIM_ROOT_OFFSET = np.array([0, 0, 0])
        self.SIM_TOE_OFFSET_LOCAL = [
            np.array([-0.02, 0.0, 0.0]),
            np.array([-0.02, 0.0, 0.01]),
            np.array([-0.02, 0.0, 0.0]),
            np.array([-0.02, 0.0, 0.01])
        ]
        
        # 기본 조인트 포즈 및 댐핑
        self.DEFAULT_JOINT_POSE = np.array([0, -0.35, 0, 0, -0.4,
                                            0.35, 0, 0, 0.4, 0,
                                            -0.2, 0.5, 0, 0, 0,
                                            0.2, -0.5, 0, 0, 0,
                                            0])
        self.JOINT_DAMPING = [0.5, 0.05, 0.01, 0.5, 0.05, 0.01, 0.5, 0.05, 0.01, 0.5, 0.05, 0.01]
        
        # 전방 방향 오프셋
        self.FORWARD_DIR_OFFSET = np.array([0, 0, 0.025])

        self.REF_PELVIS_JOINT_ID = 0
        self.REF_NECK_JOINT_ID = 0
        self.REF_SHOUL_JOINT_IDS = [1, 6]
        self.REF_CLE_JOINT_IDS = [11, 16]
        self.REF_HIP_JOINT_IDS = [ 2, 12, 7, 17]
        self.REF_TOE_JOINT_IDS = [5, 15, 10, 20]

        self.POS_SIZE = 3
        self.ROT_SIZE = 4

        self.REF_POS_SCALE = 1
        self.REF_COORD_ROT = transformations.quaternion_from_euler(0, 0, 0)
        self.REF_POS_OFFSET = np.array([1, 0, 0])
        self.REF_ROOT_ROT = transformations.quaternion_from_euler(0, 0, 0)
        self.DEFAULT_ROT = np.array([1, 0, 0, 1])
    
    def build_markers(self,num_markers): # func buil_markers(num_markers) : 상체/하체/발의 마커를 생성 및 색을 입히는 함수 -> 마커를 리턴

        marker_radius = 0.02

        markers = []
        for i in range(num_markers):
            if (i == self.REF_NECK_JOINT_ID) or (i == self.REF_PELVIS_JOINT_ID)\
                or (i in self.REF_HIP_JOINT_IDS):
                col = [0, 0, 1, 1]
            elif (i in self.REF_TOE_JOINT_IDS):
                col = [1, 0, 0, 1]
            else:
                col = [0, 1, 0, 1]

            virtual_shape_id = pybullet.createVisualShape(shapeType=pybullet.GEOM_SPHERE,
                                                        radius=marker_radius,
                                                        rgbaColor=col)
            body_id =  pybullet.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=-1,
                                        baseVisualShapeIndex=virtual_shape_id,
                                        basePosition=[0,0,0],
                                        useMaximalCoordinates=True)
            markers.append(body_id)

        return markers

    def process_ref_joint_pos_data(self,joint_pos): # * 모션 데이터를 파이불릿에 넣을 수 있는 데이터로 바꿈
        proc_pos = joint_pos.copy()
        num_pos = joint_pos.shape[0]

        for i in range(num_pos):
            curr_pos = proc_pos[i]
            curr_pos = pose3d.QuaternionRotatePoint(curr_pos, self.REF_COORD_ROT)
            curr_pos = pose3d.QuaternionRotatePoint(curr_pos, self.REF_ROOT_ROT)
            curr_pos = curr_pos * self.REF_POS_SCALE + self.REF_POS_OFFSET
            proc_pos[i] = curr_pos

        return proc_pos
    
    def set_maker_pos(self,marker_pos, marker_ids): # * 걍 마커 찍는 함수 크게 안중요
        num_markers = len(marker_ids)
        assert(num_markers == marker_pos.shape[0]) 

        for i in range(num_markers):
            curr_id = marker_ids[i]
            curr_pos = marker_pos[i]

            pybullet.resetBasePositionAndOrientation(curr_id, curr_pos, self.DEFAULT_ROT)

        return
# # RobotConfig 인스턴스 생성
# robot_config = RobotConfig()

# # 사용 예시
# print(robot_config.URDF_FILENAME)
# print(robot_config.INIT_POS)
# # 필요한 속성들에 접근하여 사용할 수 있습니다.
