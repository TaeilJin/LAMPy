from VisualData.core import Vis
import numpy as np

class SkeletonInfo(Vis):

    def __init__(self):
        super().__init__()
        self.parents = np.array([0,
            1,2,3,4, 
            1,6,7,8,
            1,10,11,
            12,13,14,15,
            12,17,
            12,19,20,21]) - 1
        self.head_idx = 17
        self.lhand_idx = 15
        self.rhand_idx = 21
        self.lfoot_idx = 4
        self.rfoot_idx = 8


    def smpl_SkelInfo(self):
        self.parents = np.array([0,
            1,2,3,4, 
            1,6,7,8,
            1,10,11,
            12,13,14,15,
            12,17,
            12,19,20,21]) - 1
        self.head_idx = 17
        self.lhand_idx = 15
        self.rhand_idx = 21
        self.lfoot_idx = 4
        self.rfoot_idx = 8
    
    def mixamo_SkelInfo(self):
        self.parents = np.array([0,1,2,3,4,5, 
            1,7,8,9,10,
            1,12,13,
            14,15,16,17,
            14,19,20,
            14,22,23,24]) - 1
       