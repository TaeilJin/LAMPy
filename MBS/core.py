import ctypes
import os


class MBS():

    def __init__(self):
        
        """ external dll """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        osg_folder_path = os.path.join(script_dir,'vs2015')
        lapack_folder_path = os.path.join(script_dir,'lapack/bin')
        dll_path = os.path.join(script_dir, 'MBS_CDLL.dll')

        os.environ['PATH'] = osg_folder_path + os.pathsep + os.environ['PATH']
        os.environ['PATH'] = lapack_folder_path + os.pathsep + os.environ['PATH']
       
        self.numlinks = 0

        print(dll_path)
        self.lib = ctypes.WinDLL(dll_path) # 'D:/TJ_develop/MW/MW_HumanMotionRetargeting/Examples/MBS_CDLL/x64/Release/MBS_CDLL.dll'

        """ define dll function in/out """
        self.lib.LOAD_MBS.argtypes = [ctypes.c_char_p]
        self.lib.INIT_JOINT_LIST.argtypes=[ctypes.c_int]
        self.lib.INIT_JOINT_LIST.restype=ctypes.c_char_p
        self.lib.UPDATE_POSE_UnitytoMW.argtypes =[ctypes.c_int,ctypes.POINTER(ctypes.c_float)]
        self.lib.OUTPUT_JOINT_POSE_UNITY.argtypes =[ctypes.c_int, ctypes.POINTER(ctypes.c_float)] 
    
    def loadMBS(self,MBS):
        self.numlinks = self.lib.LOAD_MBS(MBS)

    def updateMBSPoseFromUnity(self,mbs_int,float_array):
        """
        Args:
        update mbs [src:0,tar:1,ikmbs:2]
        """
        self.lib.UPDATE_POSE_UnitytoMW(mbs_int,float_array)
    
    def exportUnityPoseFromMBS(self,mbs_int,float_array):
        """
        Args:
        export mbs [src:0,tar:1,ikmbs:2]
        """
        self.lib.OUTPUT_JOINT_POSE_UNITY(mbs_int,float_array)

