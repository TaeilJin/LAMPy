
import numpy as np
class PyBulletUtils:
    def __init__(self) :
        self.running = True
        self.init = False
        self.data = False
        self.FRAME_DURATION = 1/60 #0.01667
        self.f1_pressed = False
    
    # ESC 키 입력을 확인하는 함수
    def check_delete_key(self,p):
        keys = p.getKeyboardEvents()
        for k, v in keys.items():
            if k == p.B3G_DELETE:
                return True
        return False
    
     # ESC 키 입력을 확인하는 함수
    def check_Init_key(self,p):
        keys = p.getKeyboardEvents()
        for k, v in keys.items():
            if k == p.B3G_F1 and (v & p.KEY_IS_DOWN):
                return True
        return False
        
    
    def load_ref_data(self,JOINT_POS_FILENAME, FRAME_START, FRAME_END): # 레퍼런스 모션 데이터를 받아옴
        joint_pos_data = np.loadtxt(JOINT_POS_FILENAME, delimiter=",")

        start_frame = 0 if (FRAME_START is None) else FRAME_START
        end_frame = joint_pos_data.shape[0] if (FRAME_END is None) else FRAME_END
        joint_pos_data = joint_pos_data[start_frame:end_frame]

        return joint_pos_data
    
    