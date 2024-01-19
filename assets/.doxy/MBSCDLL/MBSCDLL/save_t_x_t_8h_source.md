

# File saveTXT.h

[**File List**](files.md) **>** [**MBS\_CDLL**](dir_ddc84d54b9d1cd5babbd6fb469a13a43.md) **>** [**saveTXT.h**](save_t_x_t_8h.md)

[Go to the documentation of this file](save_t_x_t_8h.md)

```C++

#pragma once
#include <Windows.h>
#include <iostream>
#include <algorithm>

#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osgDB/ReadFile>

#include "Character/bCharacterLoader.h"
#include "Loader/MotionLoader.h"
#include "Loader/mgSkeletonToBCharacter.h"
#include "MocapProcessor/mgMBSUtil.h"
#include "MocapProcessor/mgUtility.h"
#include "Visualizer/gBdOsgSystem.h"
#include "Visualizer/gOSGSkin.h"
#include "Visualizer/gOSGShape.h"

#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Animation>
#include <osgAnimation/Skeleton>
#include <osgAnimation/Bone>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/StackedRotateAxisElement>
#include <osgAnimation/StackedMatrixElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedScaleElement>
#include <osg/TriangleIndexFunctor>
#include <osgDB/Options>
#include <osg/BlendFunc>

class saveTXT
{

public:
    
    std::string puttap(int n) {
        std::string output;
        for (int i = 0; i < n; i++) {
            output += "\t";
        }
        return output;
    }
    void saveVec3(FILE* fp, gVec3 vec, bool b_enter, char* name) {
        if (b_enter == true)
            fprintf(fp, "%s %g %g %g\n", name, vec.x(), vec.y(), vec.z());
        else
            fprintf(fp, "%s %g %g %g ", name, vec.x(), vec.y(), vec.z());
    };
    void saveVec3(FILE* fp, gVec3 vec, bool b_enter) {
        if(b_enter==true)
            fprintf(fp, "%g %g %g\n", vec.x(), vec.y(), vec.z());
        else
            fprintf(fp, "%g %g %g ", vec.x(), vec.y(), vec.z());
    };

    void saveQuat(FILE* fp, gQuat quat, bool b_enter) {
        if (b_enter == true)
            fprintf(fp, "%g %g %g %g\n", quat.x(), quat.y(), quat.z(), quat.w());
        else
            fprintf(fp, "%g %g %g %g ", quat.x(), quat.y(), quat.z(), quat.w());
    };

    void saveTmat(FILE* fp, gXMat gmat, char* name) {
        fprintf(fp, "%s ",name);
        saveVec3(fp, gmat.trn(), false);
        saveVec3(fp, gmat.rotX(), false);
        saveVec3(fp, gmat.rotY(), false);
        saveVec3(fp, gmat.rotZ(), true);
    };
    void saveMotion(FILE* fp, arma::mat Quat_Motion, int numLinks) {
        
        for (int i = 0; i < Quat_Motion.n_cols; i++) {
            arma::vec Quat_Pose = Quat_Motion.col(i);
            for (int j = 0; j < numLinks; j++) {
                if (j == 0) {
                    gVec3 pos;
                    pos.setX(Quat_Pose[4]);
                    pos.setY(Quat_Pose[5]);
                    pos.setZ(Quat_Pose[6]);
                    gQuat q;
                    q.setX(Quat_Pose[0]);
                    q.setY(Quat_Pose[1]);
                    q.setZ(Quat_Pose[2]);
                    q.setW(Quat_Pose[3]);
                    q.normalize();
                    // save text file
                    saveVec3(fp, pos, false);
                    saveQuat(fp, q, false);
                }
                else if (j == (numLinks - 1)) {
                    gQuat q;
                    q.setX(Quat_Pose[0]);
                    q.setY(Quat_Pose[1]);
                    q.setZ(Quat_Pose[2]);
                    q.setW(Quat_Pose[3]);
                    q.normalize();
                    saveQuat(fp, q, true);
                }
                else {
                    gQuat q;
                    q.setX(Quat_Pose[0]);
                    q.setY(Quat_Pose[1]);
                    q.setZ(Quat_Pose[2]);
                    q.setW(Quat_Pose[3]);
                    q.normalize();
                    saveQuat(fp, q, false);
                }


            }
        }
    };

    void saveMotionFile(bCharacter* character, const char* filename, arma::mat Quat_Motion) {
        FILE* fp = fopen(filename, "w");
        saveMotion(fp, Quat_Motion, character->numLinks());
    }
};


```

