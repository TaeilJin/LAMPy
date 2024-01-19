

# Class saveBVH



[**ClassList**](annotated.md) **>** [**saveBVH**](classsave_b_v_h.md)






















## Public Types

| Type | Name |
| ---: | :--- |
| enum  | [**RotSeq**](#enum-rotseq)  <br> |




















## Public Functions

| Type | Name |
| ---: | :--- |
|  std::string | [**puttap**](#function-puttap) (int n) <br> |
|  void | [**quaternion2Euler**](#function-quaternion2euler) (gQuat q, double res, RotSeq rotSeq) <br> |
|  int | [**saveBVHFile**](#function-savebvhfile) (gMultibodySystem \* character, const char \* filename, arma::mat Quat\_Motion, mgBone::\_AXISORDER axisorder, float frametime=1.0f/60.0f, float scale=1.0f) <br> |
|  void | [**saveBVH\_start2end**](#function-savebvh_start2end) (arma::mat motion, bCharacter \* character, int key\_start, int key\_end, char \* filename) <br> |
|  void | [**savejoint**](#function-savejoint-12) (FILE \* fp, mgBone \* bone, int i) <br> |
|  void | [**savejoint**](#function-savejoint-22) (FILE \* fp, gLink \* bone, int i, mgBone::\_AXISORDER axisorder, float scale) <br> |
|  void | [**threeaxisrot**](#function-threeaxisrot) (double r11, double r12, double r21, double r31, double r32, double res) <br> |
|  void | [**twoaxisrot**](#function-twoaxisrot) (double r11, double r12, double r21, double r31, double r32, double res) <br> |




























## Public Types Documentation




### enum RotSeq 

```C++
enum saveBVH::RotSeq {
    zyx,
    zyz,
    zxy,
    zxz,
    yxz,
    yxy,
    yzx,
    yzy,
    xyz,
    xyx,
    xzy,
    xzx
};
```



## Public Functions Documentation




### function puttap 

```C++
inline std::string saveBVH::puttap (
    int n
) 
```






### function quaternion2Euler 

```C++
inline void saveBVH::quaternion2Euler (
    gQuat q,
    double res,
    RotSeq rotSeq
) 
```






### function saveBVHFile 

```C++
inline int saveBVH::saveBVHFile (
    gMultibodySystem * character,
    const char * filename,
    arma::mat Quat_Motion,
    mgBone::_AXISORDER axisorder,
    float frametime=1.0f/60.0f,
    float scale=1.0f
) 
```






### function saveBVH\_start2end 

```C++
inline void saveBVH::saveBVH_start2end (
    arma::mat motion,
    bCharacter * character,
    int key_start,
    int key_end,
    char * filename
) 
```






### function savejoint [1/2]

```C++
inline void saveBVH::savejoint (
    FILE * fp,
    mgBone * bone,
    int i
) 
```






### function savejoint [2/2]

```C++
inline void saveBVH::savejoint (
    FILE * fp,
    gLink * bone,
    int i,
    mgBone::_AXISORDER axisorder,
    float scale
) 
```






### function threeaxisrot 

```C++
inline void saveBVH::threeaxisrot (
    double r11,
    double r12,
    double r21,
    double r31,
    double r32,
    double res
) 
```






### function twoaxisrot 

```C++
inline void saveBVH::twoaxisrot (
    double r11,
    double r12,
    double r21,
    double r31,
    double r32,
    double res
) 
```




------------------------------
The documentation for this class was generated from the following file `MBS_CDLL/saveBVH.h`

