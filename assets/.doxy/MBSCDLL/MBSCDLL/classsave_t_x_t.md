

# Class saveTXT



[**ClassList**](annotated.md) **>** [**saveTXT**](classsave_t_x_t.md)










































## Public Functions

| Type | Name |
| ---: | :--- |
|  std::string | [**puttap**](#function-puttap) (int n) <br> |
|  void | [**saveMotion**](#function-savemotion) (FILE \* fp, arma::mat Quat\_Motion, int numLinks) <br> |
|  void | [**saveMotionFile**](#function-savemotionfile) (bCharacter \* character, const char \* filename, arma::mat Quat\_Motion) <br> |
|  void | [**saveQuat**](#function-savequat) (FILE \* fp, gQuat quat, bool b\_enter) <br> |
|  void | [**saveTmat**](#function-savetmat) (FILE \* fp, gXMat gmat, char \* name) <br> |
|  void | [**saveVec3**](#function-savevec3-12) (FILE \* fp, gVec3 vec, bool b\_enter, char \* name) <br> |
|  void | [**saveVec3**](#function-savevec3-22) (FILE \* fp, gVec3 vec, bool b\_enter) <br> |




























## Public Functions Documentation




### function puttap 

```C++
inline std::string saveTXT::puttap (
    int n
) 
```






### function saveMotion 

```C++
inline void saveTXT::saveMotion (
    FILE * fp,
    arma::mat Quat_Motion,
    int numLinks
) 
```






### function saveMotionFile 

```C++
inline void saveTXT::saveMotionFile (
    bCharacter * character,
    const char * filename,
    arma::mat Quat_Motion
) 
```






### function saveQuat 

```C++
inline void saveTXT::saveQuat (
    FILE * fp,
    gQuat quat,
    bool b_enter
) 
```






### function saveTmat 

```C++
inline void saveTXT::saveTmat (
    FILE * fp,
    gXMat gmat,
    char * name
) 
```






### function saveVec3 [1/2]

```C++
inline void saveTXT::saveVec3 (
    FILE * fp,
    gVec3 vec,
    bool b_enter,
    char * name
) 
```






### function saveVec3 [2/2]

```C++
inline void saveTXT::saveVec3 (
    FILE * fp,
    gVec3 vec,
    bool b_enter
) 
```




------------------------------
The documentation for this class was generated from the following file `MBS_CDLL/saveTXT.h`

