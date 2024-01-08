from MBS.core import MBS
import ctypes

class MBS_Retarget(MBS):

    def __init__(self):
        super().__init__()
        """ define dll function in/out """
        self.lib.LOAD_SRC_TAR_MBS.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        self.lib.INIT_MAPPING_fromTXT.argtypes = [ctypes.c_char_p]
        self.lib.DO_RETARGET_OUTPUT.argtypes =[ctypes.c_float,ctypes.c_float,ctypes.c_float]
        #self.lib.SAVE_BVH.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_float, ctypes.c_float]

        self.numlinks = 0
        
    def initRetarget(self, srcMBS, tarMBS, MappingTxt):
        """
        Args:
        none

        Returns:
        Construct Source and Target MBS
        & Initialize Retargeting Solver
        """
        self.numlinks = self.lib.LOAD_SRC_TAR_MBS(srcMBS,tarMBS)

        self.numlinks =self.lib.INIT_RETARGET()
        print(f"TARGET numlink : {self.numlinks}")

        self.lib.INIT_MAPPING_fromTXT(MappingTxt)
    
    def doRetarget(self,base_offset):
        """
        Args:
        do retargeting solver
        """
        self.lib.DO_RETARGET_OUTPUT(base_offset[0],base_offset[1],base_offset[2])

        
    