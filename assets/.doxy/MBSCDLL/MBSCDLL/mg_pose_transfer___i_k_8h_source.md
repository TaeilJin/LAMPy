

# File mgPoseTransfer\_IK.h

[**File List**](files.md) **>** [**MBS\_CDLL**](dir_ddc84d54b9d1cd5babbd6fb469a13a43.md) **>** [**mgPoseTransfer\_IK.h**](mg_pose_transfer___i_k_8h.md)

[Go to the documentation of this file](mg_pose_transfer___i_k_8h.md)

```C++

//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _POSE_TRANSFER_Dance_H_
#define _POSE_TRANSFER_Dance_H_

#include "mbs/gMultibodySystem.h"
#include "levmar.h"

class mgPoseIKSolver{

public:

    struct idxDirTransfer{
        int idx0; //index of starting point 
        int idx1; //index of end point to define a direction vector
        double weight; //importance 
    };

    struct idxPosTransfer{
        int idx; //point index
        double weight; //importance
        double scale; //
    };
    struct idxDesired {
        int idx; // src point index
        double weight; //  importance
        gVec3 pos_desired; // desired position
    };
    struct idxDesiredDir {
        int idx; // point index
        double weight_dir; // importance
        gVec3 dir_desired; // desired direction

    };

    
    arma::vec preferredTarCoords; //preferred coordinates (compact coordinates) of tar..note first 6 components (for baselink) are ignored. initially set to zero.
    double preferredTarCoordsWeight; //weight for the preferred coordinates
    double scale;
    gMultibodySystem* src;
    gMultibodySystem* tar;  
    std::vector<std::string> namePoints;  //name of points
    std::vector<gAttachedPoint> srcPoints;//points in src
    std::vector<gAttachedPoint> tarPoints;//points in tar
    std::vector<gAttachedPoint> tarParentPoints;//points in tar
    std::vector<idxDesired>     desiredPoints; // points for specific desired
    std::vector<idxDesiredDir>  desiredDirs; // points for specific desired direction
    std::vector<idxDirTransfer> directions;
    std::vector<idxPosTransfer> positions;
    
    
    /* LevMarOpts: opts[0-4] = minim. options [\tau, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
    //                    * scale factor for initial \mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2 and the
    //                    * step used in difference approximation to the Jacobian. If \delta<0, the Jacobian is approximated
    //                    * with central differences which are more accurate (but slower!) compared to the forward differences
    //                    * employed by default. Set to NULL for defaults to be used.
    //                    */
    double LevMarOpts[5]; //option for levmar


    //                    /* O: information regarding the minimization. Set to NULL if don't care
    //                     * info[0]= ||e||_2 at initial p.
    //                     * info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||Dp||_2, \mu/max[J^T J]_ii ], all computed at estimated p.
    //                     * info[5]= # iterations,
    //                     * info[6]=reason for terminating: 1 - stopped by small gradient J^T e
    //                     *                                 2 - stopped by small Dp
    //                     *                                 3 - stopped by itmax
    //                     *                                 4 - singular matrix. Restart from current p with increased \mu
    //                     *                                 5 - no further error reduction is possible. Restart with increased mu
    //                     *                                 6 - stopped by small ||e||_2
    //                     *                                 7 - stopped by invalid (i.e. NaN or Inf) "func" values; a user error
    //                     * info[7]= # function evaluations
    //                     * info[8]= # Jacobian evaluations
    //                     * info[9]= # linear systems solved, i.e. # attempts for reducing error
    //                     */
    double LevMarInfo[LM_INFO_SZ]; //performance report from levmar is stored here.

    mgPoseIKSolver(gMultibodySystem* srcCharacter, gMultibodySystem* tarCharacter)
    {
        //assert(srcCharacter && tarCharacter);
        src = srcCharacter;
        tar = tarCharacter;
        preferredTarCoords.zeros(tar->dof());
        preferredTarCoordsWeight = 0.1f;// 0.01;
        scale = 1.0;
        double stopThresh = 1e-5;
        LevMarOpts[0]=1e-3;
        LevMarOpts[1]=stopThresh;
        LevMarOpts[2]=stopThresh;
        LevMarOpts[3]=stopThresh;
        LevMarOpts[4]=LM_DIFF_DELTA;
    }

    virtual ~mgPoseIKSolver(){}

    void addPoint(const char* name, gLink& srcLink, gVec3& srcPosLocal, gLink& tarLink, gVec3& tarPosLocal)
    {
        namePoints.push_back(name);
        gAttachedPoint ps(&srcLink,srcPosLocal);
        gAttachedPoint pt(&tarLink,tarPosLocal);
        srcPoints.push_back(ps);
        tarPoints.push_back(pt);        
    }
    // 
    void addPoint_inWorldOffset(const char* name, gLink& srcLink, gVec3& srcPosWorld, gLink& tarLink, gVec3& tarPosWorld)
    {
        namePoints.push_back(name);
        gAttachedPoint ps; ps.setLocalPositionInWorld(&srcLink, srcPosWorld);
        gAttachedPoint pt; pt.setLocalPositionInWorld(&tarLink, tarPosWorld);
        srcPoints.push_back(ps);
        tarPoints.push_back(pt);
    }
    void addDesiredJoint(const char* name, gLink& srcLink, gVec3& srcPosLocal)
    {
        namePoints.push_back(name);
        gAttachedPoint pt(&srcLink, srcPosLocal);
        tarPoints.push_back(pt);
    }
    void addDesiredJointPoseDir(const char* name, gLink& srcLink, gVec3& srcPosLocal, gLink& dir_end, gVec3& dir_end_PosLocal)
    {
        namePoints.push_back(name);
        gAttachedPoint pt(&srcLink, srcPosLocal);
        gAttachedPoint pt_child(&dir_end, dir_end_PosLocal);
        tarPoints.push_back(pt);
        tarParentPoints.push_back(pt_child);
    }

    bool addDirectionObjective(const char* pointName0, const char* pointName1, double weight);
    
    bool addPositionObjective(const char* pointName, double weight);
    bool addPositionObjective(const char* pointName, double weight, double scale);
    bool addDesiredObjective(const char* pointName, double weight, gVec3 desiredpos);
    bool addDesiredDirObjective(const char* pointName, double weight_dir, gVec3 desireddir);
    /*
    *this method transfers pose from source character to target character using levenberg-marquardt method
    offset: offset from sourse to target character
    returns 1 if succeed. 
    */
    int transferPoseLevMar(gXMat& offset);
    
    int addDesired(const char* map_name, gVec3 desired);
    int DesiredLevMar(gXMat& offset);


};

#endif

```

