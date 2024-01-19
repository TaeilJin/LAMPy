

# File MBS\_CDLL.cpp



[**FileList**](files.md) **>** [**MBS\_CDLL**](dir_ddc84d54b9d1cd5babbd6fb469a13a43.md) **>** [**MBS\_CDLL.cpp**](_m_b_s___c_d_l_l_8cpp.md)

[Go to the source code of this file](_m_b_s___c_d_l_l_8cpp_source.md)



* `#include <Windows.h>`
* `#include "MBS/MBSLoader.h"`
* `#include "saveBVH.h"`
* `#include "mgPoseTransfer_IK.h"`





















## Public Attributes

| Type | Name |
| ---: | :--- |
|  double | [**DEBUG\_DRAW\_CONSTRAINT\_SIZE**](#variable-debug_draw_constraint_size)   = = 2<br> |
|  [**mgPoseIKSolver**](classmg_pose_i_k_solver.md) \* | [**g\_poseTrans\_Avatar**](#variable-g_posetrans_avatar)  <br> |
|  arma::mat | [**g\_refCoord**](#variable-g_refcoord)  <br> |
|  [**saveBVH**](classsave_b_v_h.md) \* | [**g\_saveBVH**](#variable-g_savebvh)  <br> |
|  gMultibodySystem \* | [**mbs**](#variable-mbs)  <br> |
|  gMultibodySystem \* | [**mbs\_src**](#variable-mbs_src)  <br> |
|  gMultibodySystem \* | [**mbs\_tar**](#variable-mbs_tar)  <br> |
|  float | [**srcFrameTime**](#variable-srcframetime)   = = 30<br> |
|  arma::mat | [**tarQuaternions**](#variable-tarquaternions)  <br> |
















## Public Functions

| Type | Name |
| ---: | :--- |
|  DLL\_EXPORT int | [**ADD\_DESIRED\_DIR**](#function-add_desired_dir) (LPCSTR mapped\_name, float \* desireds\_dir, float weight\_dir) <br>_Adding desired direction class from constructed points._  |
|  DLL\_EXPORT int | [**ADD\_DESIRED\_POINTS**](#function-add_desired_points) (LPCSTR mapped\_name, float \* desireds, float weight\_pos) <br>_Adding desired points class from constructed points._  |
|  DLL\_EXPORT void | [**DO\_POSE\_IK**](#function-do_pose_ik) () <br> |
|  DLL\_EXPORT void | [**DO\_RETARGET\_OUTPUT**](#function-do_retarget_output) (float x\_offset, float y\_offset, float z\_offset) <br> |
|  DLL\_EXPORT void | [**GEN\_MBS\_TXTFILE**](#function-gen_mbs_txtfile) (LPCSTR tarCharacterMotionFile, LPCSTR tarCharactertxt) <br> |
|  DLL\_EXPORT void | [**GET\_DIRECTIONS**](#function-get_directions) (int i, int \* idx0, int \* idx1) <br> |
|  DLL\_EXPORT int | [**INIT\_DATA**](#function-init_data) (int mbs\_i, int Frames) <br> |
|  DLL\_EXPORT int | [**INIT\_IK**](#function-init_ik) () <br>_Initialize Retargeting Solver (processing mbs is constructed)_  |
|  DLL\_EXPORT LPCSTR | [**INIT\_JOINT\_LIST**](#function-init_joint_list) (int i) <br>_Ouput name of whole joints._  |
|  DLL\_EXPORT void | [**INIT\_MAPPING\_fromTXT**](#function-init_mapping_fromtxt) (char \* txtfile) <br> |
|  DLL\_EXPORT int | [**INIT\_RETARGET**](#function-init_retarget) () <br> |
|  DLL\_EXPORT int | [**LOAD\_MBS**](#function-load_mbs) (LPCSTR chaSrcTXTFile) <br> |
|  DLL\_EXPORT int | [**LOAD\_SRC\_MOTION**](#function-load_src_motion) (LPCSTR srcfilename, float src\_scale) <br> |
|  DLL\_EXPORT int | [**LOAD\_SRC\_TAR\_MBS**](#function-load_src_tar_mbs) (LPCSTR chaSrcTXTFile, LPCSTR chaTarTXTFile) <br> |
|  DLL\_EXPORT int | [**MAPPING\_DIRS**](#function-mapping_dirs) (const char \* j\_start, const char \* j\_to, float weightDir) <br> |
|  DLL\_EXPORT int | [**MAPPING\_JOINTS**](#function-mapping_joints) (const char \* mapped\_name, const char \* src\_name, const char \* tar\_name) <br> |
|  DLL\_EXPORT int | [**MAPPING\_JOINTS\_withAXIS**](#function-mapping_joints_withaxis) (const char \* mapped\_name, const char \* src\_name, const char \* tar\_name, gVec3 g\_src\_X, gVec3 g\_src\_Y, gVec3 g\_src\_Z, gVec3 g\_tar\_X, gVec3 g\_tar\_Y, gVec3 g\_tar\_Z) <br> |
|  DLL\_EXPORT int | [**MAPPING\_POS**](#function-mapping_pos) (const char \* j\_to, float weightPos) <br> |
|  DLL\_EXPORT const float \* | [**MEASURE\_DIRECTION**](#function-measure_direction) (int idx0, int idx1) <br> |
|  DLL\_EXPORT const float \* | [**MEASURE\_POSITION**](#function-measure_position) (int idx0, float \* desPoints) <br> |
|  gVec3 | [**MW\_GRAVITY\_VECTOR**](#function-mw_gravity_vector) (0, -9. 8, 0) <br> |
|  gVec3 | [**MW\_GROUND\_NORMAL**](#function-mw_ground_normal) (0, 1, 0) <br> |
|  DLL\_EXPORT void | [**OUTPUT\_JOINT\_POSE\_UNITY**](#function-output_joint_pose_unity) (int mbs\_i, float \* output\_POSE) <br>_Exporting pose array of Unity from current MBS pose._  |
|  DLL\_EXPORT void | [**OUTPUT\_JOINT\_QUATERNION\_UNITY**](#function-output_joint_quaternion_unity) (int jointindex, float \* output\_Quat) <br> |
|  DLL\_EXPORT int | [**READ\_DIRECTION\_SIZE**](#function-read_direction_size) () <br> |
|  DLL\_EXPORT float | [**READ\_FrameTime**](#function-read_frametime) () <br> |
|  DLL\_EXPORT LPCSTR | [**READ\_JOINTNAME**](#function-read_jointname) (int i) <br> |
|  DLL\_EXPORT const float \* | [**READ\_MBS\_JOINT\_LOCAL\_QUAT**](#function-read_mbs_joint_local_quat) (int joint\_id) <br> |
|  DLL\_EXPORT const float \* | [**READ\_MBS\_JOINT\_POSITION**](#function-read_mbs_joint_position) (int mbs\_i, int joint\_id) <br> |
|  DLL\_EXPORT const float \* | [**READ\_MBS\_JOINT\_WORLD\_QUAT**](#function-read_mbs_joint_world_quat) (int joint\_id) <br> |
|  DLL\_EXPORT const float \* | [**READ\_MBS\_POSE**](#function-read_mbs_pose) (int mbs\_i) <br> |
|  DLL\_EXPORT const float \* | [**READ\_SRC\_POINTS**](#function-read_src_points) (int joint\_id) <br> |
|  DLL\_EXPORT const float \* | [**READ\_TAR\_POINTS**](#function-read_tar_points) (int joint\_id) <br> |
|  DLL\_EXPORT int | [**READ\_TOTALFRAMES**](#function-read_totalframes) () <br> |
|  DLL\_EXPORT void | [**SAVE\_BVH**](#function-save_bvh) (LPCSTR bvh\_file\_path, int mbs\_i, float frametime, float scale) <br> |
|  DLL\_EXPORT int | [**SETTING\_DESDIRJOINTS**](#function-setting_desdirjoints) (LPCSTR mapped\_name, LPCSTR src\_name) <br>_Constructing points for IK solving._  |
|  DLL\_EXPORT int | [**SETUP\_DES\_DIR\_JOINTS**](#function-setup_des_dir_joints) (LPCSTR mapped\_name, LPCSTR joint\_name) <br> |
|  DLL\_EXPORT void | [**SET\_DESIRED\_BASE**](#function-set_desired_base) (float \* base) <br> |
|  DLL\_EXPORT int | [**SET\_DESIRED\_DIRS**](#function-set_desired_dirs) (int i, float \* des\_dir, float weight\_dir) <br>_Set the desired dirs object._  |
|  DLL\_EXPORT int | [**SET\_DESIRED\_POINTS**](#function-set_desired_points) (int i, float \* desireds, float weight\_pos) <br>_Set the desired points object._  |
|  DLL\_EXPORT void | [**SET\_MBS\_FromEXP**](#function-set_mbs_fromexp) (int mbs\_i, float \* joint\_vec) <br> |
|  DLL\_EXPORT void | [**SET\_MBS\_JOINT\_FromEXP**](#function-set_mbs_joint_fromexp) (int joint\_id, float \* joint\_vec) <br> |
|  DLL\_EXPORT void | [**SET\_MBS\_JOINT\_FromQuat**](#function-set_mbs_joint_fromquat) (int joint\_id, float \* joint\_vec) <br> |
|  DLL\_EXPORT void | [**SRC\_RESTORECOORD**](#function-src_restorecoord) () <br> |
|  DLL\_EXPORT void | [**SRC\_STORECOORD**](#function-src_storecoord) () <br> |
|  void | [**SetJntRotDirOBJ**](#function-setjntrotdirobj) ([**mgPoseIKSolver**](classmg_pose_i_k_solver.md) \* poseTrans, char \* txt\_id, char \* src\_jnt, char \* tar\_jnt) <br> |
|  void | [**SetJntRotDirOBJ**](#function-setjntrotdirobj) ([**mgPoseIKSolver**](classmg_pose_i_k_solver.md) \* poseTrans, char \* txt\_id, char \* src\_jnt, gVec3 s\_X, gVec3 s\_Z, char \* tar\_jnt, gVec3 t\_X, gVec3 t\_Z) <br> |
|  void | [**SetJntRotDirOBJ**](#function-setjntrotdirobj) ([**mgPoseIKSolver**](classmg_pose_i_k_solver.md) \* poseTrans, char \* txt\_id, char \* src\_jnt, gVec3 s\_X, gVec3 s\_Z, gVec3 s\_Y, char \* tar\_jnt, gVec3 t\_X, gVec3 t\_Z, gVec3 t\_Y) <br> |
|  gRotMat | [**TranseRotUnitytoMW**](#function-transerotunitytomw) (float x, float y, float z, float w) <br> |
|  DLL\_EXPORT void | [**UPDATE\_BASE\_POSITION\_UnitytoMW**](#function-update_base_position_unitytomw) (int mbs\_i, float \* input\_pos) <br> |
|  DLL\_EXPORT void | [**UPDATE\_JOINTVEC**](#function-update_jointvec) (int mbs\_i, int iter) <br> |
|  DLL\_EXPORT void | [**UPDATE\_JOINT\_QUATERNION\_UnitytoMW**](#function-update_joint_quaternion_unitytomw) (int mbs\_i, int jointidx, float \* input\_Quat) <br> |
|  DLL\_EXPORT void | [**UPDATE\_JOINT\_QUATERNION\_UnitytoMW\_usingName**](#function-update_joint_quaternion_unitytomw_usingname) (LPCSTR jointname, float \* input\_Quat) <br> |
|  DLL\_EXPORT void | [**UPDATE\_MBS**](#function-update_mbs) (int mbs\_i) <br> |
|  DLL\_EXPORT void | [**UPDATE\_POSE\_UnitytoMW**](#function-update_pose_unitytomw) (int mbs\_i, float \* input\_Pose) <br>_Updating pose of MBS by [given] pose array of Unity(LAMP)_  |
|  DLL\_EXPORT void | [**UPDATE\_POSE\_fromData**](#function-update_pose_fromdata) (int mbs\_i, int iter) <br> |
|  DLL\_EXPORT void | [**UPDATE\_SRC\_POINTS**](#function-update_src_points) () <br> |
|  DLL\_EXPORT void | [**UPDATE\_TAR\_POINTS**](#function-update_tar_points) () <br> |



























## Macros

| Type | Name |
| ---: | :--- |
| define  | [**DLL\_EXPORT**](_m_b_s___c_d_l_l_8cpp.md#define-dll_export)  \_\_declspec(dllimport)<br> |

## Public Attributes Documentation




### variable DEBUG\_DRAW\_CONSTRAINT\_SIZE 

```C++
double DEBUG_DRAW_CONSTRAINT_SIZE;
```






### variable g\_poseTrans\_Avatar 

```C++
mgPoseIKSolver* g_poseTrans_Avatar;
```






### variable g\_refCoord 

```C++
arma::mat g_refCoord;
```






### variable g\_saveBVH 

```C++
saveBVH* g_saveBVH;
```






### variable mbs 

```C++
gMultibodySystem* mbs;
```






### variable mbs\_src 

```C++
gMultibodySystem* mbs_src;
```






### variable mbs\_tar 

```C++
gMultibodySystem* mbs_tar;
```






### variable srcFrameTime 

```C++
float srcFrameTime;
```






### variable tarQuaternions 

```C++
arma::mat tarQuaternions;
```



## Public Functions Documentation




### function ADD\_DESIRED\_DIR 

_Adding desired direction class from constructed points._ 
```C++
DLL_EXPORT int ADD_DESIRED_DIR (
    LPCSTR mapped_name,
    float * desireds_dir,
    float weight_dir
) 
```





**Parameters:**


* `mapped_name` Name of point 
* `desireds_dir` desired direction 
* `weight_dir` desired weight 



**Returns:**

DLL\_EXPORT 





        



### function ADD\_DESIRED\_POINTS 

_Adding desired points class from constructed points._ 
```C++
DLL_EXPORT int ADD_DESIRED_POINTS (
    LPCSTR mapped_name,
    float * desireds,
    float weight_pos
) 
```





**Parameters:**


* `mapped_name` Name of point 
* `desireds` desired point 
* `weight_pos` desired weight 



**Returns:**

DLL\_EXPORT 





        



### function DO\_POSE\_IK 

```C++
DLL_EXPORT void DO_POSE_IK () 
```






### function DO\_RETARGET\_OUTPUT 

```C++
DLL_EXPORT void DO_RETARGET_OUTPUT (
    float x_offset,
    float y_offset,
    float z_offset
) 
```






### function GEN\_MBS\_TXTFILE 

```C++
DLL_EXPORT void GEN_MBS_TXTFILE (
    LPCSTR tarCharacterMotionFile,
    LPCSTR tarCharactertxt
) 
```






### function GET\_DIRECTIONS 

```C++
DLL_EXPORT void GET_DIRECTIONS (
    int i,
    int * idx0,
    int * idx1
) 
```






### function INIT\_DATA 

```C++
DLL_EXPORT int INIT_DATA (
    int mbs_i,
    int Frames
) 
```






### function INIT\_IK 

_Initialize Retargeting Solver (processing mbs is constructed)_ 
```C++
DLL_EXPORT int INIT_IK () 
```





**Returns:**

DLL\_EXPORT 





        



### function INIT\_JOINT\_LIST 

_Ouput name of whole joints._ 
```C++
DLL_EXPORT LPCSTR INIT_JOINT_LIST (
    int i
) 
```





**Parameters:**


* `i` Index of joint 



**Returns:**

DLL\_EXPORT 





        



### function INIT\_MAPPING\_fromTXT 

```C++
DLL_EXPORT void INIT_MAPPING_fromTXT (
    char * txtfile
) 
```






### function INIT\_RETARGET 

```C++
DLL_EXPORT int INIT_RETARGET () 
```






### function LOAD\_MBS 

```C++
DLL_EXPORT int LOAD_MBS (
    LPCSTR chaSrcTXTFile
) 
```






### function LOAD\_SRC\_MOTION 

```C++
DLL_EXPORT int LOAD_SRC_MOTION (
    LPCSTR srcfilename,
    float src_scale
) 
```






### function LOAD\_SRC\_TAR\_MBS 

```C++
DLL_EXPORT int LOAD_SRC_TAR_MBS (
    LPCSTR chaSrcTXTFile,
    LPCSTR chaTarTXTFile
) 
```






### function MAPPING\_DIRS 

```C++
DLL_EXPORT int MAPPING_DIRS (
    const char * j_start,
    const char * j_to,
    float weightDir
) 
```






### function MAPPING\_JOINTS 

```C++
DLL_EXPORT int MAPPING_JOINTS (
    const char * mapped_name,
    const char * src_name,
    const char * tar_name
) 
```






### function MAPPING\_JOINTS\_withAXIS 

```C++
DLL_EXPORT int MAPPING_JOINTS_withAXIS (
    const char * mapped_name,
    const char * src_name,
    const char * tar_name,
    gVec3 g_src_X,
    gVec3 g_src_Y,
    gVec3 g_src_Z,
    gVec3 g_tar_X,
    gVec3 g_tar_Y,
    gVec3 g_tar_Z
) 
```






### function MAPPING\_POS 

```C++
DLL_EXPORT int MAPPING_POS (
    const char * j_to,
    float weightPos
) 
```






### function MEASURE\_DIRECTION 

```C++
DLL_EXPORT const float * MEASURE_DIRECTION (
    int idx0,
    int idx1
) 
```






### function MEASURE\_POSITION 

```C++
DLL_EXPORT const float * MEASURE_POSITION (
    int idx0,
    float * desPoints
) 
```






### function MW\_GRAVITY\_VECTOR 

```C++
gVec3 MW_GRAVITY_VECTOR (
    0,
    -9. 8,
    0
) 
```






### function MW\_GROUND\_NORMAL 

```C++
gVec3 MW_GROUND_NORMAL (
    0,
    1,
    0
) 
```






### function OUTPUT\_JOINT\_POSE\_UNITY 

_Exporting pose array of Unity from current MBS pose._ 
```C++
DLL_EXPORT void OUTPUT_JOINT_POSE_UNITY (
    int mbs_i,
    float * output_POSE
) 
```





**Parameters:**


* `mbs_i` Type of MBS (0: source , 1: target, 2: processing) 
* `output_POSE` pose array of Unity 



**Returns:**

DLL\_EXPORT 





        



### function OUTPUT\_JOINT\_QUATERNION\_UNITY 

```C++
DLL_EXPORT void OUTPUT_JOINT_QUATERNION_UNITY (
    int jointindex,
    float * output_Quat
) 
```






### function READ\_DIRECTION\_SIZE 

```C++
DLL_EXPORT int READ_DIRECTION_SIZE () 
```






### function READ\_FrameTime 

```C++
DLL_EXPORT float READ_FrameTime () 
```






### function READ\_JOINTNAME 

```C++
DLL_EXPORT LPCSTR READ_JOINTNAME (
    int i
) 
```






### function READ\_MBS\_JOINT\_LOCAL\_QUAT 

```C++
DLL_EXPORT const float * READ_MBS_JOINT_LOCAL_QUAT (
    int joint_id
) 
```






### function READ\_MBS\_JOINT\_POSITION 

```C++
DLL_EXPORT const float * READ_MBS_JOINT_POSITION (
    int mbs_i,
    int joint_id
) 
```






### function READ\_MBS\_JOINT\_WORLD\_QUAT 

```C++
DLL_EXPORT const float * READ_MBS_JOINT_WORLD_QUAT (
    int joint_id
) 
```






### function READ\_MBS\_POSE 

```C++
DLL_EXPORT const float * READ_MBS_POSE (
    int mbs_i
) 
```






### function READ\_SRC\_POINTS 

```C++
DLL_EXPORT const float * READ_SRC_POINTS (
    int joint_id
) 
```






### function READ\_TAR\_POINTS 

```C++
DLL_EXPORT const float * READ_TAR_POINTS (
    int joint_id
) 
```






### function READ\_TOTALFRAMES 

```C++
DLL_EXPORT int READ_TOTALFRAMES () 
```






### function SAVE\_BVH 

```C++
DLL_EXPORT void SAVE_BVH (
    LPCSTR bvh_file_path,
    int mbs_i,
    float frametime,
    float scale
) 
```






### function SETTING\_DESDIRJOINTS 

_Constructing points for IK solving._ 
```C++
DLL_EXPORT int SETTING_DESDIRJOINTS (
    LPCSTR mapped_name,
    LPCSTR src_name
) 
```





**Parameters:**


* `mapped_name` Name of point 
* `src_name` Name of joint 



**Returns:**

DLL\_EXPORT 





        



### function SETUP\_DES\_DIR\_JOINTS 

```C++
DLL_EXPORT int SETUP_DES_DIR_JOINTS (
    LPCSTR mapped_name,
    LPCSTR joint_name
) 
```






### function SET\_DESIRED\_BASE 

```C++
DLL_EXPORT void SET_DESIRED_BASE (
    float * base
) 
```






### function SET\_DESIRED\_DIRS 

_Set the desired dirs object._ 
```C++
DLL_EXPORT int SET_DESIRED_DIRS (
    int i,
    float * des_dir,
    float weight_dir
) 
```





**Parameters:**


* `i` 
* `des_dir` 
* `weight_dir` 



**Returns:**

DLL\_EXPORT 





        



### function SET\_DESIRED\_POINTS 

_Set the desired points object._ 
```C++
DLL_EXPORT int SET_DESIRED_POINTS (
    int i,
    float * desireds,
    float weight_pos
) 
```





**Parameters:**


* `i` 
* `desireds` 
* `weight_pos` 



**Returns:**

DLL\_EXPORT 





        



### function SET\_MBS\_FromEXP 

```C++
DLL_EXPORT void SET_MBS_FromEXP (
    int mbs_i,
    float * joint_vec
) 
```






### function SET\_MBS\_JOINT\_FromEXP 

```C++
DLL_EXPORT void SET_MBS_JOINT_FromEXP (
    int joint_id,
    float * joint_vec
) 
```






### function SET\_MBS\_JOINT\_FromQuat 

```C++
DLL_EXPORT void SET_MBS_JOINT_FromQuat (
    int joint_id,
    float * joint_vec
) 
```






### function SRC\_RESTORECOORD 

```C++
DLL_EXPORT void SRC_RESTORECOORD () 
```






### function SRC\_STORECOORD 

```C++
DLL_EXPORT void SRC_STORECOORD () 
```






### function SetJntRotDirOBJ 

```C++
void SetJntRotDirOBJ (
    mgPoseIKSolver * poseTrans,
    char * txt_id,
    char * src_jnt,
    char * tar_jnt
) 
```






### function SetJntRotDirOBJ 

```C++
void SetJntRotDirOBJ (
    mgPoseIKSolver * poseTrans,
    char * txt_id,
    char * src_jnt,
    gVec3 s_X,
    gVec3 s_Z,
    char * tar_jnt,
    gVec3 t_X,
    gVec3 t_Z
) 
```






### function SetJntRotDirOBJ 

```C++
void SetJntRotDirOBJ (
    mgPoseIKSolver * poseTrans,
    char * txt_id,
    char * src_jnt,
    gVec3 s_X,
    gVec3 s_Z,
    gVec3 s_Y,
    char * tar_jnt,
    gVec3 t_X,
    gVec3 t_Z,
    gVec3 t_Y
) 
```






### function TranseRotUnitytoMW 

```C++
gRotMat TranseRotUnitytoMW (
    float x,
    float y,
    float z,
    float w
) 
```






### function UPDATE\_BASE\_POSITION\_UnitytoMW 

```C++
DLL_EXPORT void UPDATE_BASE_POSITION_UnitytoMW (
    int mbs_i,
    float * input_pos
) 
```






### function UPDATE\_JOINTVEC 

```C++
DLL_EXPORT void UPDATE_JOINTVEC (
    int mbs_i,
    int iter
) 
```






### function UPDATE\_JOINT\_QUATERNION\_UnitytoMW 

```C++
DLL_EXPORT void UPDATE_JOINT_QUATERNION_UnitytoMW (
    int mbs_i,
    int jointidx,
    float * input_Quat
) 
```






### function UPDATE\_JOINT\_QUATERNION\_UnitytoMW\_usingName 

```C++
DLL_EXPORT void UPDATE_JOINT_QUATERNION_UnitytoMW_usingName (
    LPCSTR jointname,
    float * input_Quat
) 
```






### function UPDATE\_MBS 

```C++
DLL_EXPORT void UPDATE_MBS (
    int mbs_i
) 
```






### function UPDATE\_POSE\_UnitytoMW 

_Updating pose of MBS by [given] pose array of Unity(LAMP)_ 
```C++
DLL_EXPORT void UPDATE_POSE_UnitytoMW (
    int mbs_i,
    float * input_Pose
) 
```





**Parameters:**


* `mbs_i` Type of MBS (0: source , 1: target, 2: processing) 
* `input_Pose` pose array of Unity 



**Returns:**

DLL\_EXPORT 





        



### function UPDATE\_POSE\_fromData 

```C++
DLL_EXPORT void UPDATE_POSE_fromData (
    int mbs_i,
    int iter
) 
```






### function UPDATE\_SRC\_POINTS 

```C++
DLL_EXPORT void UPDATE_SRC_POINTS () 
```






### function UPDATE\_TAR\_POINTS 

```C++
DLL_EXPORT void UPDATE_TAR_POINTS () 
```



## Macro Definition Documentation





### define DLL\_EXPORT 

```C++
#define DLL_EXPORT __declspec(dllimport)
```




------------------------------
The documentation for this class was generated from the following file `MBS_CDLL/MBS_CDLL.cpp`

