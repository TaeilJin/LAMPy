

# Class mgPoseIKSolver



[**ClassList**](annotated.md) **>** [**mgPoseIKSolver**](classmg_pose_i_k_solver.md)



_This class provides method to transfer a pose from a character to another character. Refer to Example8\_CharacterRetargetting project for the usage of this class._ [More...](#detailed-description)

* `#include <mgPoseTransfer_IK.h>`















## Classes

| Type | Name |
| ---: | :--- |
| struct | [**idxDesired**](structmg_pose_i_k_solver_1_1idx_desired.md) <br> |
| struct | [**idxDesiredDir**](structmg_pose_i_k_solver_1_1idx_desired_dir.md) <br> |
| struct | [**idxDirTransfer**](structmg_pose_i_k_solver_1_1idx_dir_transfer.md) <br> |
| struct | [**idxPosTransfer**](structmg_pose_i_k_solver_1_1idx_pos_transfer.md) <br> |






## Public Attributes

| Type | Name |
| ---: | :--- |
|  double | [**LevMarInfo**](#variable-levmarinfo)  <br> |
|  double | [**LevMarOpts**](#variable-levmaropts)  <br> |
|  std::vector&lt; [**idxDesiredDir**](structmg_pose_i_k_solver_1_1idx_desired_dir.md) &gt; | [**desiredDirs**](#variable-desireddirs)  <br> |
|  std::vector&lt; [**idxDesired**](structmg_pose_i_k_solver_1_1idx_desired.md) &gt; | [**desiredPoints**](#variable-desiredpoints)  <br> |
|  std::vector&lt; [**idxDirTransfer**](structmg_pose_i_k_solver_1_1idx_dir_transfer.md) &gt; | [**directions**](#variable-directions)  <br> |
|  std::vector&lt; std::string &gt; | [**namePoints**](#variable-namepoints)  <br> |
|  std::vector&lt; [**idxPosTransfer**](structmg_pose_i_k_solver_1_1idx_pos_transfer.md) &gt; | [**positions**](#variable-positions)  <br> |
|  arma::vec | [**preferredTarCoords**](#variable-preferredtarcoords)  <br> |
|  double | [**preferredTarCoordsWeight**](#variable-preferredtarcoordsweight)  <br> |
|  double | [**scale**](#variable-scale)  <br> |
|  gMultibodySystem \* | [**src**](#variable-src)  <br> |
|  std::vector&lt; gAttachedPoint &gt; | [**srcPoints**](#variable-srcpoints)  <br> |
|  gMultibodySystem \* | [**tar**](#variable-tar)  <br> |
|  std::vector&lt; gAttachedPoint &gt; | [**tarParentPoints**](#variable-tarparentpoints)  <br> |
|  std::vector&lt; gAttachedPoint &gt; | [**tarPoints**](#variable-tarpoints)  <br> |
















## Public Functions

| Type | Name |
| ---: | :--- |
|  int | [**DesiredLevMar**](#function-desiredlevmar) (gXMat & offset) <br> |
|  int | [**addDesired**](#function-adddesired) (const char \* map\_name, gVec3 desired) <br> |
|  bool | [**addDesiredDirObjective**](#function-adddesireddirobjective) (const char \* pointName, double weight\_dir, gVec3 desireddir) <br> |
|  void | [**addDesiredJoint**](#function-adddesiredjoint) (const char \* name, gLink & srcLink, gVec3 & srcPosLocal) <br> |
|  void | [**addDesiredJointPoseDir**](#function-adddesiredjointposedir) (const char \* name, gLink & srcLink, gVec3 & srcPosLocal, gLink & dir\_end, gVec3 & dir\_end\_PosLocal) <br> |
|  bool | [**addDesiredObjective**](#function-adddesiredobjective) (const char \* pointName, double weight, gVec3 desiredpos) <br> |
|  bool | [**addDirectionObjective**](#function-adddirectionobjective) (const char \* pointName0, const char \* pointName1, double weight) <br> |
|  void | [**addPoint**](#function-addpoint) (const char \* name, gLink & srcLink, gVec3 & srcPosLocal, gLink & tarLink, gVec3 & tarPosLocal) <br> |
|  void | [**addPoint\_inWorldOffset**](#function-addpoint_inworldoffset) (const char \* name, gLink & srcLink, gVec3 & srcPosWorld, gLink & tarLink, gVec3 & tarPosWorld) <br> |
|  bool | [**addPositionObjective**](#function-addpositionobjective-12) (const char \* pointName, double weight) <br> |
|  bool | [**addPositionObjective**](#function-addpositionobjective-22) (const char \* pointName, double weight, double scale) <br> |
|   | [**mgPoseIKSolver**](#function-mgposeiksolver) (gMultibodySystem \* srcCharacter, gMultibodySystem \* tarCharacter) <br> |
|  int | [**transferPoseLevMar**](#function-transferposelevmar) (gXMat & offset) <br> |
| virtual  | [**~mgPoseIKSolver**](#function-mgposeiksolver) () <br> |




























# Detailed Description




**Date:**

2015/03/26 




**Author:**

Sung-Hee Lee 




**Warning:**


 





    
## Public Attributes Documentation




### variable LevMarInfo 

```C++
double mgPoseIKSolver::LevMarInfo[LM_INFO_SZ];
```






### variable LevMarOpts 

```C++
double mgPoseIKSolver::LevMarOpts[5];
```






### variable desiredDirs 

```C++
std::vector<idxDesiredDir> mgPoseIKSolver::desiredDirs;
```






### variable desiredPoints 

```C++
std::vector<idxDesired> mgPoseIKSolver::desiredPoints;
```






### variable directions 

```C++
std::vector<idxDirTransfer> mgPoseIKSolver::directions;
```






### variable namePoints 

```C++
std::vector<std::string> mgPoseIKSolver::namePoints;
```






### variable positions 

```C++
std::vector<idxPosTransfer> mgPoseIKSolver::positions;
```






### variable preferredTarCoords 

```C++
arma::vec mgPoseIKSolver::preferredTarCoords;
```






### variable preferredTarCoordsWeight 

```C++
double mgPoseIKSolver::preferredTarCoordsWeight;
```






### variable scale 

```C++
double mgPoseIKSolver::scale;
```






### variable src 


```C++
gMultibodySystem* mgPoseIKSolver::src;
```



tar: target character to transfer pose to 
src: source character to import pose from 
tar/srcPoints: a set of sample points attached to characters. tarPoints and srcPoints must be consistent (i.e., if tarPoints[0] 
is attached to tar's right elbow, srcPoints[0] must be attached to the src's right elbow) 
hence, it must be that tarPoints.size() = srcPoints.size() = namePoints.size() 
namePoints stores the name of the points used for identifying the points. 
directions: list of indices of two points to form a direction vector 
positions: list of point indices that will be glued from srcPoints to tarPoints 

how to transfer pose: 
We try to make the direction vector made by two points (from pointPairs) of tarCharacter match that of the corresponding vector of srcCharacter 



        



### variable srcPoints 

```C++
std::vector<gAttachedPoint> mgPoseIKSolver::srcPoints;
```






### variable tar 

```C++
gMultibodySystem* mgPoseIKSolver::tar;
```






### variable tarParentPoints 

```C++
std::vector<gAttachedPoint> mgPoseIKSolver::tarParentPoints;
```






### variable tarPoints 

```C++
std::vector<gAttachedPoint> mgPoseIKSolver::tarPoints;
```



## Public Functions Documentation




### function DesiredLevMar 

```C++
int mgPoseIKSolver::DesiredLevMar (
    gXMat & offset
) 
```






### function addDesired 

```C++
int mgPoseIKSolver::addDesired (
    const char * map_name,
    gVec3 desired
) 
```






### function addDesiredDirObjective 

```C++
bool mgPoseIKSolver::addDesiredDirObjective (
    const char * pointName,
    double weight_dir,
    gVec3 desireddir
) 
```






### function addDesiredJoint 

```C++
inline void mgPoseIKSolver::addDesiredJoint (
    const char * name,
    gLink & srcLink,
    gVec3 & srcPosLocal
) 
```






### function addDesiredJointPoseDir 

```C++
inline void mgPoseIKSolver::addDesiredJointPoseDir (
    const char * name,
    gLink & srcLink,
    gVec3 & srcPosLocal,
    gLink & dir_end,
    gVec3 & dir_end_PosLocal
) 
```






### function addDesiredObjective 

```C++
bool mgPoseIKSolver::addDesiredObjective (
    const char * pointName,
    double weight,
    gVec3 desiredpos
) 
```






### function addDirectionObjective 


```C++
bool mgPoseIKSolver::addDirectionObjective (
    const char * pointName0,
    const char * pointName1,
    double weight
) 
```



add a direction objective for pose transfer direction is defined by a vector from pointName0 to pointName1 weight: importance of this objective returns true if succeed.


add a direction objective for pose transfer direction is defined by a vector from pointName0 to pointName1 weight: importance of this objective 


        



### function addPoint 


```C++
inline void mgPoseIKSolver::addPoint (
    const char * name,
    gLink & srcLink,
    gVec3 & srcPosLocal,
    gLink & tarLink,
    gVec3 & tarPosLocal
) 
```



add points used for pose transfer. name: name of point. used for identification. srcLink/tarLink: link of src/tar to attach a point to srcPosLocal/tarPosLocal: position of a point wrt srcLink/tarLink's body frame 
 


        



### function addPoint\_inWorldOffset 

```C++
inline void mgPoseIKSolver::addPoint_inWorldOffset (
    const char * name,
    gLink & srcLink,
    gVec3 & srcPosWorld,
    gLink & tarLink,
    gVec3 & tarPosWorld
) 
```






### function addPositionObjective [1/2]


```C++
bool mgPoseIKSolver::addPositionObjective (
    const char * pointName,
    double weight
) 
```



add a position objective for pose transfer this tries to match the absolute position of the point of target character to that of source character. weight: importance of this objective returns true if succeed.


add a position objective for pose transfer this tries to match the absolute position of the point of target character to that of source character. weight: importance of this objective 


        



### function addPositionObjective [2/2]

```C++
bool mgPoseIKSolver::addPositionObjective (
    const char * pointName,
    double weight,
    double scale
) 
```






### function mgPoseIKSolver 


```C++
inline mgPoseIKSolver::mgPoseIKSolver (
    gMultibodySystem * srcCharacter,
    gMultibodySystem * tarCharacter
) 
```



motion will be transferred from srcCharacter to tarCharacter. 


        



### function transferPoseLevMar 


```C++
int mgPoseIKSolver::transferPoseLevMar (
    gXMat & offset
) 
```



this method transfers pose from src to tar 


        



### function ~mgPoseIKSolver 

```C++
inline virtual mgPoseIKSolver::~mgPoseIKSolver () 
```




------------------------------
The documentation for this class was generated from the following file `MBS_CDLL/mgPoseTransfer_IK.h`

