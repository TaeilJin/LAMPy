from MBS.core import MBS
import ctypes
import numpy as np

class MBS_DesIK(MBS):

    def __init__(self):
        super().__init__()
        """
        target points ("mappingname","bone name")
        """
        self.lib.SETTING_DESDIRJOINTS.argtypes = [ctypes.c_char_p , ctypes.c_char_p]
        
        self.lib.ADD_DESIRED_POINTS.argtypes =[ctypes.c_char_p,ctypes.POINTER(ctypes.c_float), ctypes.c_float]
        self.lib.ADD_DESIRED_DIR.argtypes=[ctypes.c_char_p, ctypes.POINTER(ctypes.c_float), ctypes.c_float]
        self.lib.SET_DESIRED_POINTS.argtypes=[ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.c_float]
        self.lib.SET_DESIRED_DIRS.argtypes=[ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.c_float]

        self.lib.INIT_IK.argtypes =[]
        self.lib.DO_POSE_IK.argtypes =[]
        self.Name =[]
        self.RealName =[]
        
    
    def find_indices(self, my_list, target_value):
        #indices = [index for index, value in enumerate(my_list) if value == target_value]
        index  = my_list.index(target_value.decode())
        return index

    def initIK(self, MBS):
        """
        Args:
        Src Text of MBS File

        Returns:
        Construct Target MBS
        & Initialize Retargeting Solver (tarPoints,tarEndPoints : desiredPoints,desiredDirs)
        """
        # load MBS
        self.numlinks = self.loadMBS(MBS)
        self.numlinks = self.lib.INIT_IK()
        print(f"TARGET numlink : {self.numlinks}")

        # construct target points, desired points 
        for i in range(0,self.numlinks):
            joint_name = self.lib.INIT_JOINT_LIST(i).decode()
            print(joint_name)

            result_desired_joints = self.lib.SETTING_DESDIRJOINTS(f"t{i}".encode('utf-8'),joint_name.encode('utf-8'))
            
            self.Name.append(f"t{i}")
            self.RealName.append(joint_name)
        
        print(f"SET_DES_JOINTS : {result_desired_joints}" )

        # initial desired 
        for i in range(0,self.numlinks):
            arr = np.array([0,0,0],dtype=np.float32)
            arr_ptr = arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            desPoints = self.lib.ADD_DESIRED_POINTS(f"t{i}".encode('utf-8'),arr_ptr, 1.0)
            if(i > 0):
                desDirs = self.lib.ADD_DESIRED_DIR(f"t{i}".encode('utf-8'),arr_ptr,1.0)

        print( f"Desired Points: {desPoints} Desired Dirs: {desDirs}" )

    def SetDesPositionArray(self, des_index, des_pos_arr, des_weight_arr):
        for i, des in enumerate(des_index):
            arr = (ctypes.c_float * len(des_pos_arr[3*i:3*i+3]))(*des_pos_arr[3*i:3*i+3])
            if (des == 1):
                self.lib.SET_DESIRED_POINTS(i,arr,des_weight_arr[i])
            else:
                self.lib.SET_DESIRED_POINTS(i,arr,0.0)
    
    def SetDesDirectionArray(self, des_index, des_dir_arr, des_weight_arr):
        for i, des in enumerate(des_index):
            arr = (ctypes.c_float * len(des_dir_arr[3*i:3*i+3]))(*des_dir_arr[3*i:3*i+3])
            if (des == 1):
                self.lib.SET_DESIRED_DIRS(i,arr,des_weight_arr[i])
            else:
                self.lib.SET_DESIRED_DIRS(i,arr,0.0)
    
    def SetDesPosition(self, joint_name, world_pos = 0.0, weight=1.0):
        i = self.find_indices(self.RealName,joint_name)
        #n = self.Name[self.find_indices(self.RealName,joint_name)]
        self.lib.SET_DESIRED_POINTS(i,world_pos,weight)
    
    def SetDesDirection(self, joint_name, world_dir, weight=1.0):
        i = self.find_indices(self.RealName,joint_name)
        self.lib.ADD_DESIRED_DIR(i,world_dir,weight)

    def doIK(self):
        """
        Args:
        do retargeting solver
        """
        self.lib.DO_POSE_IK()

        
    