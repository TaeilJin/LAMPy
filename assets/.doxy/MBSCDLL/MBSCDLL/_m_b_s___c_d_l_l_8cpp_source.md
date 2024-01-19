

# File MBS\_CDLL.cpp

[**File List**](files.md) **>** [**MBS\_CDLL**](dir_ddc84d54b9d1cd5babbd6fb469a13a43.md) **>** [**MBS\_CDLL.cpp**](_m_b_s___c_d_l_l_8cpp.md)

[Go to the documentation of this file](_m_b_s___c_d_l_l_8cpp.md)

```C++


#include <Windows.h>
#include "MBS/MBSLoader.h"
#include "saveBVH.h"
#include "mgPoseTransfer_IK.h"

#ifdef  UNITY_MW_DLL_TEST_EXPORTS
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT __declspec(dllimport)
#endif //  DLL_TEST_EXPORTS


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);

gMultibodySystem* mbs;
//-- retarget Xsense->Ybot
gMultibodySystem* mbs_src;
gMultibodySystem* mbs_tar;

mgPoseIKSolver* g_poseTrans_Avatar; // Ybot -> Avatar

float srcFrameTime = 30;
arma::mat tarQuaternions;
arma::mat g_refCoord;
saveBVH* g_saveBVH;

void SetJntRotDirOBJ(mgPoseIKSolver* poseTrans, char* txt_id, char* src_jnt, char* tar_jnt) {

    poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
}
void SetJntRotDirOBJ(mgPoseIKSolver* poseTrans, char* txt_id, char* src_jnt, gVec3 s_X, gVec3 s_Z, char* tar_jnt, gVec3 t_X, gVec3 t_Z) {
    gLink* srcLink = poseTrans->src->findLink(src_jnt);
    gLink* tarLink = poseTrans->tar->findLink(tar_jnt);

    if (srcLink == NULL || tarLink == NULL)
        std::cout << "you should check joint names" << std::endl;
    else {
        poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
        char p_x[45]; char p_z[45]; char p_y[45];
        strcpy(p_x, txt_id); strcat(p_x, "_x");
        strcpy(p_y, txt_id); strcat(p_y, "_y");
        strcpy(p_z, txt_id); strcat(p_z, "_z");
        //poseTrans->addPoint(p_x, *poseTrans->src->findLink(src_jnt), s_X, *poseTrans->tar->findLink(tar_jnt), t_X);
        //poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), gVec3(0, 1, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 1, 0));
        //poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), s_Z, *poseTrans->tar->findLink(tar_jnt), t_Z);

        poseTrans->addPoint(p_x, *poseTrans->src->findLink(src_jnt), poseTrans->src->findLink(src_jnt)->frame().rotX(),
            *poseTrans->tar->findLink(tar_jnt), poseTrans->tar->findLink(tar_jnt)->frame().rotX());
        poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), poseTrans->src->findLink(src_jnt)->frame().rotY(),
            *poseTrans->tar->findLink(tar_jnt), poseTrans->tar->findLink(tar_jnt)->frame().rotY());
        poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), poseTrans->src->findLink(src_jnt)->frame().rotZ(),
            *poseTrans->tar->findLink(tar_jnt), poseTrans->tar->findLink(tar_jnt)->frame().rotZ());



        poseTrans->addDirectionObjective(txt_id, p_x, 1.0);
        poseTrans->addDirectionObjective(txt_id, p_z, 1.0);
        poseTrans->addDirectionObjective(txt_id, p_y, 1.0);
    }
}
void SetJntRotDirOBJ(mgPoseIKSolver* poseTrans, char* txt_id, char* src_jnt, gVec3 s_X, gVec3 s_Z, gVec3 s_Y, char* tar_jnt, gVec3 t_X, gVec3 t_Z, gVec3 t_Y) {
    gLink* srcLink = poseTrans->src->findLink(src_jnt);
    gLink* tarLink = poseTrans->tar->findLink(tar_jnt);

    if (srcLink == NULL || tarLink == NULL)
        std::cout << "you should check joint names" << std::endl;
    else {
        poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
        char p_x[45]; char p_z[45]; char p_y[45];
        strcpy(p_x, txt_id); strcat(p_x, "_x");
        strcpy(p_y, txt_id); strcat(p_y, "_y");
        strcpy(p_z, txt_id); strcat(p_z, "_z");
        //poseTrans->addPoint(p_x, *poseTrans->src->findLink(src_jnt), s_X, *poseTrans->tar->findLink(tar_jnt), t_X);
        //poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), s_Y, *poseTrans->tar->findLink(tar_jnt), t_Y);
        //poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), s_Z, *poseTrans->tar->findLink(tar_jnt), t_Z);

        gVec3 sj_pos = poseTrans->src->findLink(src_jnt)->frame().trn();
        gVec3 tj_pos = poseTrans->tar->findLink(tar_jnt)->frame().trn();
        poseTrans->addPoint_inWorldOffset(p_x, *poseTrans->src->findLink(src_jnt), gVec3(sj_pos.x() + 1, sj_pos.y(), sj_pos.z()), *poseTrans->tar->findLink(tar_jnt), gVec3(tj_pos.x()+1, tj_pos.y(), tj_pos.z()));
        poseTrans->addPoint_inWorldOffset(p_y, *poseTrans->src->findLink(src_jnt), gVec3(sj_pos.x() + 0, sj_pos.y()+1, sj_pos.z()), *poseTrans->tar->findLink(tar_jnt), gVec3(tj_pos.x() + 0, tj_pos.y()+1, tj_pos.z()));
        poseTrans->addPoint_inWorldOffset(p_z, *poseTrans->src->findLink(src_jnt), gVec3(sj_pos.x() + 0, sj_pos.y(), sj_pos.z()+1), *poseTrans->tar->findLink(tar_jnt), gVec3(tj_pos.x() + 0, tj_pos.y(), tj_pos.z()+1));

        poseTrans->addDirectionObjective(txt_id, p_x, 1.0);
        poseTrans->addDirectionObjective(txt_id, p_z, 1.0);
        poseTrans->addDirectionObjective(txt_id, p_y, 1.0);
    }
}
gRotMat TranseRotUnitytoMW(float x, float y, float z, float w) {
    gQuat quat; quat.set(x,y,z,w);
    
    gQuat quat_rH = quat;
    gQuat quat_lH;
    //convert righthand quaternion to lefthand quaternion
    quat_lH.setX(quat_rH.x());
    quat_lH.setY(-1.0 * quat_rH.y());
    quat_lH.setZ(-1.0 * quat_rH.z());
    quat_lH.setW(quat_rH.w());

    /*gRotMat rot_LeftHanded = quat.inRotMatrix();
    gRotMat rot_RightHanded;
    double* convert_L2R = new double[9];
    convert_L2R[0] = rot_LeftHanded.e(0);    convert_L2R[3] = -1 * rot_LeftHanded.e(3); convert_L2R[6] = -1 * rot_LeftHanded.e(6);
    convert_L2R[1] = -1 * rot_LeftHanded.e(1); convert_L2R[4] = rot_LeftHanded.e(4);    convert_L2R[7] = rot_LeftHanded.e(7);
    convert_L2R[2] = -1 * rot_LeftHanded.e(2); convert_L2R[5] = rot_LeftHanded.e(5);    convert_L2R[8] = rot_LeftHanded.e(8);
    rot_RightHanded.set(convert_L2R);*/

    return quat_lH.inRotMatrix();
}
extern "C"
{
    /* MBS CDLL */
    // generate the MBS File
    DLL_EXPORT void GEN_MBS_TXTFILE(LPCSTR tarCharacterMotionFile, LPCSTR tarCharactertxt) {
        MotionLoader loader;
        loader.loadMotionFile(tarCharacterMotionFile);
        mgData* motion = loader.getMotion();
        mgSkeleton* skeleton = loader.getSkeleton();
        srcFrameTime = motion->frameTime;
        
        std::cout << "|--- load src character file ---|" << std::endl;
        mgSkeletonToBCharacter::saveToBCharacter(skeleton, tarCharactertxt, 70.0);
        
    }

    

    // (0: src, 1 : tar, 2 : IK)
    // update specific MBS 
    DLL_EXPORT void UPDATE_MBS(int mbs_i) {
        if(mbs_i == 0)
            mbs_src->updateKinematicsUptoPos();
        else if (mbs_i == 1)
            mbs_tar->updateKinematicsUptoPos();
        else if (mbs_i == 2)
            mbs->updateKinematicsUptoPos();
    }
    // update pose of MBS
    DLL_EXPORT void SET_MBS_FromEXP(int mbs_i, float* joint_vec) {
        
        if (mbs_i == 0) {
            // float �迭�� arma::vec�� ����
            int ndof = mbs_src->numLinks() * 3 + 3;
            arma::vec lca;
            lca.set_size(ndof);
            std::memcpy(lca.memptr(), joint_vec, ndof);
            mbs_src->setFromCompactCoordArray(lca);
        }
        else if (mbs_i == 1) {
            // float �迭�� arma::vec�� ����
            int ndof = mbs_tar->numLinks() * 3 + 3;
            arma::vec lca;
            lca.set_size(ndof);
            std::memcpy(lca.memptr(), joint_vec, ndof);
            mbs_tar->setFromCompactCoordArray(lca);
        }
        else if (mbs_i == 2) {
            // float �迭�� arma::vec�� ����
            int ndof = mbs->numLinks() * 3 + 3;
            arma::vec lca;
            lca.set_size(ndof);
            std::memcpy(lca.memptr(), joint_vec, ndof);
            mbs->setFromCompactCoordArray(lca);
        }
        //
    }
    DLL_EXPORT void SET_MBS_JOINT_FromEXP(int joint_id, float* joint_vec) {
        gRotMat rot; rot.makeExp(gVec3(joint_vec[0], joint_vec[1], joint_vec[2]));
        mbs->link(joint_id)->setFromCompactCoordArray(rot.cptr());
    }
    DLL_EXPORT void SET_MBS_JOINT_FromQuat(int joint_id, float* joint_vec) {
        gQuat quat; quat.set(joint_vec[0], joint_vec[1], joint_vec[2], joint_vec[3]);
        
        gXMat mat_G = mbs->link(joint_id)->localFrame();
        mat_G.setRot(quat.inRotMatrix());

        mbs->link(joint_id)->setFromCompactCoordArray(mat_G.rotInQuat().cptr());
    }
    
    // read pose of MBS (float*)
    DLL_EXPORT const float* READ_MBS_POSE(int mbs_i) {

        if (mbs_i == 0) {
            std::cout << " mbs dof " << mbs_src->dof() << std::endl;

            arma::vec lca(mbs_src->dof());
            mbs_src->getCompactCoordArray(lca);

            float* data = new float[mbs_src->dof()];

            for (int i = 0; i < mbs_src->dof(); i++) {
                data[i] = lca[i];
            }

            //std::cout << " lca " << lca << std::endl;

            return data;
        }
        else if (mbs_i == 1) {
            std::cout << " mbs dof " << mbs_tar->dof() << std::endl;

            arma::vec lca(mbs_tar->dof());
            mbs_tar->getCompactCoordArray(lca);

            float* data = new float[mbs_tar->dof()];

            for (int i = 0; i < mbs_tar->dof(); i++) {
                data[i] = lca[i];
            }

            //std::cout << " lca " << lca << std::endl;

            return data;
        }
        else if (mbs_i == 2) {
            std::cout << " mbs dof " << mbs->dof() << std::endl;

            arma::vec lca(mbs->dof());
            mbs->getCompactCoordArray(lca);

            float* data = new float[mbs->dof()];

            for (int i = 0; i < mbs->dof(); i++) {
                data[i] = lca[i];
            }

            //std::cout << " lca " << lca << std::endl;

            return data;
        }
    }

    // read position of MBS (float*)
    DLL_EXPORT const float* READ_MBS_JOINT_POSITION(int mbs_i, int joint_id) {
        
        static float data[] = { 9.20f, 2.19f, 11.23f };

        if (mbs_i == 0) {
            gVec3 trn = mbs_src->link(joint_id)->frame().trn();
            //std::cout << " hi " << trn << std::endl;
            data[0] = trn.x(); data[1] = trn.y(); data[2] = trn.z();
        }
        else if(mbs_i ==1) {
            gVec3 trn = mbs_tar->link(joint_id)->frame().trn();
            //std::cout << " hi " << trn << std::endl;
            data[0] = trn.x(); data[1] = trn.y(); data[2] = trn.z();
        }
        else if (mbs_i == 2) {
            gVec3 trn = mbs->link(joint_id)->frame().trn();
            //std::cout << " hi " << trn << std::endl;
            data[0] = trn.x(); data[1] = trn.y(); data[2] = trn.z();
        }
        return data;
    }
    DLL_EXPORT const float* READ_MBS_JOINT_WORLD_QUAT(int joint_id) {

        static float data[] = { 9.20f, 2.19f, 11.23f , 25.61f };
        gQuat quat =  mbs->link(joint_id)->frame().rot().inQuat();
        data[0] = quat.x(); data[1] = quat.y(); data[2] = quat.z(); data[3] = quat.w();

        return data;
    }
    DLL_EXPORT const float* READ_MBS_JOINT_LOCAL_QUAT(int joint_id) {
        static float data[] = { 9.20f, 2.19f, 11.23f , 25.61f };
        gQuat quat = mbs->link(joint_id)->localFrame().rot().inQuat();
        data[0] = quat.x(); data[1] = quat.y(); data[2] = quat.z(); data[3] = quat.w();

        return data;
    }

    // read name of Joint (string)
    DLL_EXPORT LPCSTR READ_JOINTNAME(int i) {

        LPCSTR joint_name = mbs->link(i)->name();

        return joint_name;
    }
    


    /* Retarget CDLL */
    DLL_EXPORT int LOAD_SRC_TAR_MBS(LPCSTR chaSrcTXTFile, LPCSTR chaTarTXTFile) {
        MBSLoader tar_loader;
        mbs_src = new gMultibodySystem();
        tar_loader.loadModelUnity(chaSrcTXTFile, mbs_src, 1.0);
        for (int j = 0; j < mbs_src->numLinks(); j++) {
            std::cout << " joint " << mbs_src->link(j)->name() << " world position " << mbs_src->link(j)->frame().trn() <<
                " x " << mbs_src->link(j)->frame().rotX() << " y " << mbs_src->link(j)->frame().rotY() << " z " << mbs_src->link(j)->frame().rotZ() << std::endl;
        }
        mbs_tar = new gMultibodySystem();
        tar_loader.loadModelUnity(chaTarTXTFile, mbs_tar, 1.0);
        for (int j = 0; j < mbs_tar->numLinks(); j++)
            std::cout << " joint " << mbs_tar->link(j)->name() << " world position " << mbs_tar->link(j)->frame().trn() <<
            " x " << mbs_tar->link(j)->frame().rotX() << " y " << mbs_tar->link(j)->frame().rotY() << " z " << mbs_tar->link(j)->frame().rotZ() << std::endl;


        //  std::cout << " joint " << src->link(j)->name() << " x " << src->link(j)->frame().rotX() << " y " <<
        //  src->link(j)->frame().rotY() << " z " << src->link(j)->frame().rotZ() << std::endl;
        return mbs_src->numLinks();
    }
    DLL_EXPORT int INIT_RETARGET() {

        //tarQuaternions = arma::mat(mbs_tar->sizeSafeCoordArray(), Frames, arma::fill::zeros);

        g_poseTrans_Avatar = new mgPoseIKSolver(mbs_src, mbs_tar);
        g_poseTrans_Avatar->scale = mbs_tar->link(0)->frame().trn().y() / mbs_src->link(0)->frame().trn().y();//1.0;
        std::cout << " scale " << g_poseTrans_Avatar->scale << std::endl;
        return g_poseTrans_Avatar->tar->numLinks();
    }
    DLL_EXPORT int LOAD_SRC_MOTION(LPCSTR srcfilename, float src_scale) {
        std::cout << "|--- load src motion file ---|" << std::endl;
        // get src motion file
        MotionLoader loader;
        loader.setTranslateScale(src_scale);
        loader.loadMotionFile(srcfilename);
        mgData* motion = loader.getMotion();
        mgSkeleton* skeleton = loader.getSkeleton();
        srcFrameTime = motion->frameTime;

        std::cout << "|--- load src motion file ---|" << std::endl;
        // load src motion (pose sequence) : nMotion is a total number of frame for motion data
        arma::mat refCoord(mbs_src->sizeCompactCoordArray(), motion->nMotion, arma::fill::zeros);
        for (int f = 0; f < motion->nMotion; f++)
        {
            arma::vec coord;

            mgMBSUtil::getCoordArrayFromRawData(
                coord,
                mbs_src,
                skeleton,
                motion->motions[f]
            );

            //refCoord.col(f) = coord;
            refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
        }
        g_refCoord = refCoord;
        std::cout << " rows: " << g_refCoord.n_rows << " cols: " << g_refCoord.n_cols << std::endl;

        return g_refCoord.n_cols;
    }
    DLL_EXPORT int READ_TOTALFRAMES() {
        return g_refCoord.n_cols;
    }
    DLL_EXPORT float READ_FrameTime() {
        return srcFrameTime;
    }

    /* Mapping Joints */
    DLL_EXPORT void INIT_MAPPING_fromTXT(char* txtfile) {

        
        std::ifstream inFile(txtfile);

        if (!inFile) {
            std::cerr << "unable to open file " << std::endl;
            exit(1);
        }

        int numLines = 0;
        std::string line;
        std::vector<float> ee_ind;

        while (getline(inFile, line)) {

            std::stringstream ss(line);

            std::vector<std::string> data;
            std::string element;
            while (ss >> element) {
                data.push_back(element);
            }
            //
            float ind = std::stof(data[0]);
            ee_ind.push_back(ind);

            // Create an output string stream
            std::ostringstream oss;

            // Write the string and float to the output stream
            oss << "j" << numLines;

            // Get the resulting string from the output stream
            std::string result = oss.str();
            const char* charPtr = result.c_str();
            // Allocate a buffer for the char* string
            char* buffer = new char[result.length() + 1];
            std::strcpy(buffer, charPtr);

            //std::cout << " buffer " << buffer << std::endl;
            //chain
            //SetJntRotDirOBJ(g_poseTrans_Avatar, buffer, (char*)data[1].c_str(), (char*)data[2].c_str());

            if (ind == 1) {
                gVec3 srcX(std::stof(data[3]), std::stof(data[4]), std::stof(data[5]));
                gVec3 tarX(std::stof(data[6]), std::stof(data[7]), std::stof(data[8]));

                gVec3 srcY(std::stof(data[9]), std::stof(data[10]), std::stof(data[11]));
                gVec3 tarY(std::stof(data[12]), std::stof(data[13]), std::stof(data[14]));

                gVec3 srcZ(std::stof(data[15]), std::stof(data[16]), std::stof(data[17]));
                gVec3 tarZ(std::stof(data[18]), std::stof(data[19]), std::stof(data[20]));


                SetJntRotDirOBJ(g_poseTrans_Avatar, buffer, (char*)data[1].c_str(), srcX, srcZ, srcY, (char*)data[2].c_str(), tarX, tarZ, tarY);
            }
            else{
                SetJntRotDirOBJ(g_poseTrans_Avatar, buffer, (char*)data[1].c_str(), (char*)data[2].c_str());
            }

            numLines++;
        }


        for (int i = 0; i < ee_ind.size() - 1; i++) {
            // Create an output string stream
            std::ostringstream oss;

            // Write the string and float to the output stream
            oss << "j" << i;

            std::ostringstream oss2;
            // Write the string and float to the output stream
            oss2 << "j" << i + 1;

            if (ee_ind[i] != 1) {
                g_poseTrans_Avatar->addDirectionObjective(oss.str().c_str(), oss2.str().c_str(), 1.0);
            }
        }

        //pelvis position
        g_poseTrans_Avatar->addDesiredObjective("j0", 10.0, gVec3(0, 0, 0));


        // Close the file
        inFile.close();

    }

    DLL_EXPORT int MAPPING_JOINTS(const char* mapped_name, const char* src_name, const char* tar_name) {

        //g_poseTrans_Avatar->addPoint(mapped_name, *g_poseTrans_Avatar->src->findLink(src_name), gVec3(0, 0, 0), *g_poseTrans_Avatar->tar->findLink(tar_name), gVec3(0, 0, 0));
        SetJntRotDirOBJ(g_poseTrans_Avatar, (char*)mapped_name, (char*)src_name, (char*)tar_name);
        //std::cout << (char*)src_name << (char*)tar_name << " tarpoints " << g_poseTrans_Avatar->tarPoints[0].body() << std::endl;
        //std::cout<<" tarpoints " << g_poseTrans_Avatar->tarPoints[0].body() << std::endl;
        //g_poseTrans_Avatar->tarPoints[0].updateKinematicsUptoPos();
        return g_poseTrans_Avatar->tarPoints.size();
    }
    DLL_EXPORT int MAPPING_JOINTS_withAXIS(const char* mapped_name, const char* src_name, const char* tar_name,
        gVec3 g_src_X, gVec3 g_src_Y, gVec3 g_src_Z,
        gVec3 g_tar_X, gVec3 g_tar_Y, gVec3 g_tar_Z) {

        //g_poseTrans_Avatar->addPoint(mapped_name, *g_poseTrans_Avatar->src->findLink(src_name), gVec3(0, 0, 0), *g_poseTrans_Avatar->tar->findLink(tar_name), gVec3(0, 0, 0));
        //SetJntRotDirOBJ(g_poseTrans_Avatar, (char*)mapped_name, (char*)src_name, (char*)tar_name);
        gVec3 src_x = mbs_src->findLink((char*)src_name)->frame().rotX();
        gVec3 src_y = mbs_src->findLink((char*)src_name)->frame().rotY();
        gVec3 src_z = mbs_src->findLink((char*)src_name)->frame().rotZ();

        //std::cout << (char*)src_name << (char*)tar_name << " src_x " <<  src_x << " tar rot x " << mat_tar.rotX() << " tar_x" << mat_tar.invMultVec3(src_x) << std::endl;
        //std::cout << (char*)src_name << (char*)tar_name << " src_y " << src_y << mat_tar.invMultVec3(src_y) << std::endl;
        //std::cout << (char*)src_name << (char*)tar_name << " src_z " << src_z << mat_tar.invMultVec3(src_z) << std::endl;

        SetJntRotDirOBJ(g_poseTrans_Avatar, (char*)mapped_name,
            (char*)src_name, g_src_X, g_src_Z, g_src_Y,
            (char*)tar_name, g_tar_X, g_tar_Z, g_tar_Y);

        //std::cout << (char*)src_name << (char*)tar_name << " tarpoints " << g_poseTrans_Avatar->tarPoints[0].body() << std::endl;
        //std::cout<<" tarpoints " << g_poseTrans_Avatar->tarPoints[0].body() << std::endl;
        //g_poseTrans_Avatar->tarPoints[0].updateKinematicsUptoPos();
        return g_poseTrans_Avatar->tarPoints.size();
    }
    DLL_EXPORT int MAPPING_DIRS(const char* j_start, const char* j_to, float weightDir) {
        //direction objectives
        g_poseTrans_Avatar->addDirectionObjective(j_start, j_to, weightDir);
        std::cout << " directions " << g_poseTrans_Avatar->directions.size() << " WEIGHT " << weightDir << " " << j_start << j_to << g_poseTrans_Avatar->namePoints[0] << std::endl;
        return 1;
    }
    DLL_EXPORT int MAPPING_POS(const char* j_to, float weightPos) {
        //pelvis position
        g_poseTrans_Avatar->addDesiredObjective(j_to, weightPos, gVec3(0, 0, 0));
        return 1;
    }
    
    /* Retargeting Using C++ LEVMAR */
    DLL_EXPORT void DO_RETARGET_OUTPUT(float x_offset, float y_offset, float z_offset) {
        //
        g_poseTrans_Avatar->src->updateKinematicsUptoPos();

        // desired positions update
        for (int p = 0; p < g_poseTrans_Avatar->srcPoints.size(); p++) {

            g_poseTrans_Avatar->tarPoints[p].updateKinematicsUptoPos();
            g_poseTrans_Avatar->srcPoints[p].updateKinematicsUptoPos();

        }

        //motion retargeting
        g_poseTrans_Avatar->desiredPoints[0].pos_desired = g_poseTrans_Avatar->scale * g_poseTrans_Avatar->src->link(0)->frame().trn(); //g_poseTrans->scale

        //std::cout << " let's retarget " << std::endl;
        gXMat offset; offset.setTrn(0, 0, 0);
        g_poseTrans_Avatar->transferPoseLevMar(offset);
        g_poseTrans_Avatar->tar->updateKinematicsUptoPos();

        gVec3 new_pelvis = g_poseTrans_Avatar->tar->link(0)->frame().trn();
        new_pelvis.setX(new_pelvis.x() + x_offset);
        new_pelvis.setY(new_pelvis.y() + y_offset);
        new_pelvis.setZ(new_pelvis.z() + z_offset);
        g_poseTrans_Avatar->tar->setBasePosition(new_pelvis);
        g_poseTrans_Avatar->tar->updateKinematicsUptoPos();

    }
    

    /* Retargeting Functions */
    DLL_EXPORT void UPDATE_SRC_POINTS() {
        for (int i = 0; i < g_poseTrans_Avatar->srcPoints.size(); ++i) {
            g_poseTrans_Avatar->srcPoints[i].updateKinematicsUptoPos();
        }
    }
    DLL_EXPORT const float* READ_SRC_POINTS(int joint_id) {

        static float data[] = { 9.20f, 2.19f, 11.23f };

        gVec3 trn = g_poseTrans_Avatar->srcPoints[joint_id].posWorld();
        //std::cout << " hi " << trn << std::endl;
        data[0] = trn.x(); data[1] = trn.y(); data[2] = trn.z();

        return data;
    }
    DLL_EXPORT void UPDATE_TAR_POINTS() {
        for (int i = 0; i < g_poseTrans_Avatar->tarPoints.size(); ++i) {
            g_poseTrans_Avatar->tarPoints[i].updateKinematicsUptoPos();
        }
    }
    DLL_EXPORT const float* READ_TAR_POINTS(int joint_id) {

        static float data[] = { 9.20f, 2.19f, 11.23f };

        gVec3 trn = g_poseTrans_Avatar->tarPoints[joint_id].posWorld();
        //std::cout << " hi " << trn << std::endl;
        data[0] = trn.x(); data[1] = trn.y(); data[2] = trn.z();

        return data;
    }
    DLL_EXPORT void SRC_STORECOORD() {
        mbs_src->storeCoord();
    }
    DLL_EXPORT void SRC_RESTORECOORD() {
        mbs_src->restoreCoord();
    }
    DLL_EXPORT void SET_DESIRED_BASE(float* base) {
        g_poseTrans_Avatar->desiredPoints[0].pos_desired = g_poseTrans_Avatar->scale * g_poseTrans_Avatar->src->link(0)->frame().trn(); //g_poseTrans->scale
    }
    DLL_EXPORT int READ_DIRECTION_SIZE() {
        return g_poseTrans_Avatar->directions.size();
    }
    DLL_EXPORT void GET_DIRECTIONS(int i, int* idx0, int* idx1) {
        *idx0 = g_poseTrans_Avatar->directions[i].idx0;
        *idx1 = g_poseTrans_Avatar->directions[i].idx1;
    }
    DLL_EXPORT const float* MEASURE_DIRECTION(int idx0, int idx1) {
        gVec3 dirSrc = g_poseTrans_Avatar->srcPoints[idx1].posWorld() - g_poseTrans_Avatar->srcPoints[idx0].posWorld();
        gVec3 dirTar = g_poseTrans_Avatar->tarPoints[idx1].posWorld() - g_poseTrans_Avatar->tarPoints[idx0].posWorld();
        dirSrc.normalize();
        dirTar.normalize();
        gVec3 del = dirSrc - dirTar;
        
        static float data[] = { 9.20f, 2.19f, 11.23f };
        data[0] = del.x(); data[1] = del.y(); data[2] = del.z();

        return data;
    }
    DLL_EXPORT const float* MEASURE_POSITION(int idx0, float* desPoints)
    {
        gVec3 posTar = g_poseTrans_Avatar->tarPoints[idx0].posWorld();
        gVec3 del = posTar - gVec3(desPoints[0],desPoints[1],desPoints[2]);
        
        static float data[] = { 9.20f, 2.19f, 11.23f };
        data[0] = del.x(); data[1] = del.y(); data[2] = del.z();

        return data;
    }

    /* Desired IK Using C++ LEVMAR */

    // load input MBS Data
    DLL_EXPORT int LOAD_MBS(LPCSTR chaSrcTXTFile) {
        MBSLoader tar_loader;
        mbs = new gMultibodySystem();
        tar_loader.loadModelUnity(chaSrcTXTFile, mbs, 1.0);
        for (int j = 0; j < mbs->numLinks(); j++)
            std::cout << mbs->link(j)->name() << mbs->link(j)->frame().trn() <<std::endl;
        //  std::cout << " joint " << src->link(j)->name() << " x " << src->link(j)->frame().rotX() << " y " <<
        //  src->link(j)->frame().rotY() << " z " << src->link(j)->frame().rotZ() << std::endl;
        return mbs->numLinks();
    }

    DLL_EXPORT void DO_POSE_IK() {
        //
        gXMat offset;
        mbs->updateKinematicsUptoPos();
        g_poseTrans_Avatar->DesiredLevMar(offset);

        arma::vec lca(mbs->dof());
        mbs->getCompactCoordArray(lca);
        mbs->setFromCompactCoordArray(lca);
        mbs->updateKinematicsUptoPos();

        /*for (int j = 0; j < mbs->numLinks(); j++)
            std::cout << mbs->link(j)->name() << mbs->link(j)->frame().trn() << std::endl;*/

        g_poseTrans_Avatar->tar->updateKinematicsUptoPos();
    }
    
    DLL_EXPORT int INIT_IK() {
        
        g_poseTrans_Avatar = new mgPoseIKSolver(mbs, mbs);

        return mbs->numLinks();
    }

    DLL_EXPORT LPCSTR INIT_JOINT_LIST(int i) {

        LPCSTR joint_name = mbs->link(i)->name();

        return joint_name;
    }
    DLL_EXPORT int SETUP_DES_DIR_JOINTS(LPCSTR mapped_name, LPCSTR joint_name) {
        g_poseTrans_Avatar->addDesiredJoint(mapped_name, *mbs->findLink(joint_name), gVec3(0, 0, 0));

        return g_poseTrans_Avatar->tarPoints.size();
    }
    DLL_EXPORT int SETTING_DESDIRJOINTS(LPCSTR mapped_name, LPCSTR src_name) {

        g_poseTrans_Avatar->addDesiredJointPoseDir(mapped_name, *mbs->findLink(src_name), gVec3(0, 0, 0),
            *mbs->findLink(src_name)->parent(), gVec3(0, 0, 0));

        return g_poseTrans_Avatar->tarPoints.size();
    }
    DLL_EXPORT int ADD_DESIRED_POINTS(LPCSTR mapped_name, float* desireds, float weight_pos) {
        
        gVec3 point;
        point.setX(-1 * desireds[0]);
        point.setY(desireds[1]);
        point.setZ(desireds[2]);
        
        g_poseTrans_Avatar->addDesiredObjective(mapped_name, weight_pos, point);

        return g_poseTrans_Avatar->desiredPoints.size();

    }
    DLL_EXPORT int ADD_DESIRED_DIR(LPCSTR mapped_name, float* desireds_dir, float weight_dir) {

        gVec3 direction;
        direction.setX(-1 * desireds_dir[0]);
        direction.setY(desireds_dir[1]);
        direction.setZ(desireds_dir[2]);

        g_poseTrans_Avatar->addDesiredDirObjective(mapped_name, weight_dir, direction);

        return g_poseTrans_Avatar->desiredDirs.size();
    }
    DLL_EXPORT int SET_DESIRED_POINTS(int i, float* desireds, float weight_pos) {

        gVec3 pos;
        pos.setX(-1 * desireds[0]);
        pos.setY(desireds[1]);
        pos.setZ(desireds[2]);

        g_poseTrans_Avatar->desiredPoints[i].pos_desired = pos;
        g_poseTrans_Avatar->desiredPoints[i].weight = weight_pos;

        return g_poseTrans_Avatar->desiredPoints.size();
    }
    DLL_EXPORT int SET_DESIRED_DIRS(int i, float* des_dir, float weight_dir) {

        gVec3 dir; 
        dir.setX(-1 * des_dir[0]);
        dir.setY(des_dir[1]);
        dir.setZ(des_dir[2]);

        //std::cout << " desiredDirs " << g_poseTrans_Avatar->desiredDirs.size() << std::endl;
        g_poseTrans_Avatar->desiredDirs[i].dir_desired = dir;
        g_poseTrans_Avatar->desiredDirs[i].weight_dir = weight_dir;

        return g_poseTrans_Avatar->desiredDirs.size();
    }

    /* Motion Data */
    DLL_EXPORT int INIT_DATA(int mbs_i, int Frames) {
        if (mbs_i == 0) {
            tarQuaternions = arma::mat(mbs_src->sizeSafeCoordArray(), Frames, arma::fill::zeros);
        }
        else if (mbs_i == 1) {
            tarQuaternions = arma::mat(mbs_tar->sizeSafeCoordArray(), Frames, arma::fill::zeros);
        }
        else if (mbs_i == 2) {
            tarQuaternions = arma::mat(mbs->sizeSafeCoordArray(), Frames, arma::fill::zeros);
        }
        std::cout << " Frames " << Frames << " DoF(quat) " << tarQuaternions.n_rows << std::endl;
        return tarQuaternions.n_cols;
    }
    DLL_EXPORT void UPDATE_POSE_fromData(int mbs_i, int iter) {
        //std::cout << " total frames " << g_refCoord.n_cols << std::endl;
        //for (int k = 0; k < 6; k++)
        //  std::cout << " hi " << g_refCoord.col(iter)[k] << std::endl;

        arma::vec pose = g_refCoord.col(iter);
        if (mbs_i == 0) {
            mbs_src->setFromCompactCoordArray(pose);
            mbs_src->updateKinematicsUptoPos();
        }
        else if(mbs_i == 1) {
            mbs_tar->setFromCompactCoordArray(pose);
            mbs_tar->updateKinematicsUptoPos();
        }
        else if (mbs_i == 2) {
            mbs->setFromCompactCoordArray(pose);
            mbs->updateKinematicsUptoPos();
        }
    }
    DLL_EXPORT void UPDATE_JOINTVEC(int mbs_i, int iter) {
        if(mbs_i == 0){
            arma::vec Quat_pose(mbs_src->numLinks()*4 + 3);
            mbs_src->getSafeCoordArray(Quat_pose);
            tarQuaternions.submat(0, iter, Quat_pose.n_elem - 1, iter) = Quat_pose;
            //tarQuaternions(0, iter, arma::SizeMat(Quat_pose.n_elem, 1)) = Quat_pose;
        }
        else if (mbs_i == 1) {
            arma::vec Quat_pose(mbs_tar->numLinks() * 4 + 3);
            mbs_tar->getSafeCoordArray(Quat_pose);
            tarQuaternions.submat(0, iter, Quat_pose.n_elem - 1, iter) = Quat_pose;
        }
        else if (mbs_i == 2) {
            arma::vec Quat_pose(mbs->numLinks() * 4 + 3);
            mbs->getSafeCoordArray(Quat_pose);
            tarQuaternions.submat(0, iter, Quat_pose.n_elem - 1, iter) = Quat_pose;
        }
        
    }
    
    /* Export Data */
    DLL_EXPORT void SAVE_BVH(LPCSTR bvh_file_path, int mbs_i, float frametime, float scale) {
        if(mbs_i == 0)
            g_saveBVH->saveBVHFile(mbs_src, bvh_file_path, tarQuaternions, mgBone::_AXISORDER::ZYX, frametime, scale);
        if (mbs_i == 1)
            g_saveBVH->saveBVHFile(mbs_tar, bvh_file_path, tarQuaternions, mgBone::_AXISORDER::ZYX, frametime, scale);
        if (mbs_i == 2)
            g_saveBVH->saveBVHFile(mbs, bvh_file_path, tarQuaternions, mgBone::_AXISORDER::ZYX, frametime, scale);
    }

    /* Unity MBS */
    DLL_EXPORT void UPDATE_BASE_POSITION_UnitytoMW(int mbs_i, float* input_pos) {

        gVec3 position(-1.0 * input_pos[0], input_pos[1], input_pos[2]);

        
        if (mbs_i == 0) {
            mbs_src->setBasePosition(position);
            mbs_src->updateKinematicsUptoPos();
        }
        else if (mbs_i == 1) {
            mbs_tar->setBasePosition(position);
            mbs_tar->updateKinematicsUptoPos();
        }
        else if (mbs_i == 2) {
            mbs->setBasePosition(position);
            mbs->updateKinematicsUptoPos();
        }

    }

    DLL_EXPORT void UPDATE_JOINT_QUATERNION_UnitytoMW(int mbs_i, int jointidx, float* input_Quat) {
        gQuat quat; quat.set(input_Quat[0], input_Quat[1], input_Quat[2], input_Quat[3]);
        gRotMat rot_LeftHanded = quat.inRotMatrix();
        gRotMat rot_RightHanded;
        double* convert_L2R = new double[9];
        convert_L2R[0] = rot_LeftHanded.e(0);    convert_L2R[3] = -1 * rot_LeftHanded.e(3); convert_L2R[6] = -1 * rot_LeftHanded.e(6);
        convert_L2R[1] = -1 * rot_LeftHanded.e(1); convert_L2R[4] = rot_LeftHanded.e(4);    convert_L2R[7] = rot_LeftHanded.e(7);
        convert_L2R[2] = -1 * rot_LeftHanded.e(2); convert_L2R[5] = rot_LeftHanded.e(5);    convert_L2R[8] = rot_LeftHanded.e(8);
        rot_RightHanded.set(convert_L2R);


        if (mbs_i == 0) {
            gXMat mat_G = mbs_src->link(jointidx)->localFrame();
            mat_G.setRot(rot_RightHanded);

            mbs_src->link(jointidx)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
            mbs_src->updateKinematicsUptoPos();
        }
        else if (mbs_i == 1) {
            gXMat mat_G = mbs_tar->link(jointidx)->localFrame();
            mat_G.setRot(rot_RightHanded);

            mbs_tar->link(jointidx)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
            mbs_tar->updateKinematicsUptoPos();
        }
        else if (mbs_i == 2) {
            gXMat mat_G = mbs->link(jointidx)->localFrame();
            mat_G.setRot(rot_RightHanded);

            mbs->link(jointidx)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
            mbs->updateKinematicsUptoPos();
        }

        

    }

    DLL_EXPORT void UPDATE_POSE_UnitytoMW(int mbs_i, float* input_Pose) {
        

        if (mbs_i == 0) {

            //std::cout << " 0 : " << input_Pose[0] << " 1 : " << input_Pose[1] <<" 2 : "<< input_Pose[2] << std::endl;

            for (int j = 0; j < mbs_src->numLinks(); j++) {

                if (j == 0) {
                    // base position
                    gVec3 position(-1.0 * input_Pose[0], input_Pose[1], input_Pose[2]);
                    gRotMat rot_righthanded = TranseRotUnitytoMW(input_Pose[3 + 4 * j + 0], input_Pose[3 + 4 * j + 1], input_Pose[3 + 4 * j + 2], input_Pose[3 + 4 * j + 3]);
                    
                    gXMat T; T.setTrn(position); T.setRot(rot_righthanded);
                    mbs_src->setBasePose(T);
                }
                else {
                    gRotMat rot_righthanded = TranseRotUnitytoMW(input_Pose[3 + 4 * j + 0], input_Pose[3 + 4 * j + 1], input_Pose[3 + 4 * j + 2], input_Pose[3 + 4 * j + 3]);
                    mbs_src->link(j)->setFromSafeCoordArray(rot_righthanded.inQuat().cptr());
                }
            }
            mbs_src->updateKinematicsUptoPos();
            gVec3 pos1 = mbs_src->link(0)->frame().trn();

        }
        else if (mbs_i == 1) {
            for (int j = 0; j < mbs_tar->numLinks(); j++) {

                if (j == 0) {
                    // base position
                    gVec3 position(-1.0 * input_Pose[0], input_Pose[1], input_Pose[2]);
                    mbs_tar->setBasePosition(position);
                }

                // rotation
                gRotMat rot_righthanded = TranseRotUnitytoMW(input_Pose[3 + 4 * j + 0], input_Pose[3 + 4 * j + 1], input_Pose[3 + 4 * j + 2], input_Pose[3 + 4 * j + 3]);
                gXMat mat_G = mbs_tar->link(j)->localFrame();
                mat_G.setRot(rot_righthanded);

                mbs_tar->link(j)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
            }
            mbs_tar->updateKinematicsUptoPos();
        
        }
        else if (mbs_i == 2) {
            for (int j = 0; j < mbs->numLinks(); j++) {

                if (j == 0) {
                    // base position
                    gVec3 position(-1.0 * input_Pose[0], input_Pose[1], input_Pose[2]);
                    mbs->setBasePosition(position);
                }

                // rotation
                gRotMat rot_righthanded = TranseRotUnitytoMW(input_Pose[3 + 4 * j + 0], input_Pose[3 + 4 * j + 1], input_Pose[3 + 4 * j + 2], input_Pose[3 + 4 * j + 3]);
                gXMat mat_G = mbs->link(j)->localFrame();
                mat_G.setRot(rot_righthanded);

                mbs->link(j)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
            }
            mbs->updateKinematicsUptoPos();

        }

    }
    
    DLL_EXPORT void OUTPUT_JOINT_POSE_UNITY(int mbs_i, float* output_POSE) {
        
        if (mbs_i == 0) {
            mbs_src->updateKinematicsUptoPos();
            //hip world position
            gVec3 pos = mbs_src->link(0)->frame().trn();
            
            //convert righthand quaternion to lefthand quaternion
            output_POSE[0] = -1 * pos.x(); output_POSE[1] = pos.y(); output_POSE[2] = pos.z();
            
            //joints local rotation
            for (int i = 0; i < mbs_src->numLinks(); i++) {
                
                arma::vec Quat(4);
                mbs_src->link(i)->getSafeCoordArray(Quat.memptr());

                gQuat quat_rH = gQuat(Quat[0],Quat[1], Quat[2], Quat[3]);
                gQuat quat_lH;
                //convert righthand quaternion to lefthand quaternion
                quat_lH.setX(quat_rH.x());
                quat_lH.setY(-1.0 * quat_rH.y());
                quat_lH.setZ(-1.0 * quat_rH.z());
                quat_lH.setW(quat_rH.w());

                //convert righthand quaternion to lefthand quaternion
                output_POSE[3 + 4 * i + 0] = quat_lH.x(); output_POSE[3 + 4 * i + 1] = quat_lH.y(); output_POSE[3 + 4 * i + 2] = quat_lH.z(); output_POSE[3 + 4 * i + 3] = quat_lH.w();
            }
        }
        if (mbs_i == 1) {
            //output_POSE = (float*)malloc((mbs_tar->numLinks()*4 + 3) * sizeof(float)); // �������� �迭 �Ҵ�

            mbs_tar->updateKinematicsUptoPos();
            //hip world position
            gVec3 pos = mbs_tar->link(0)->frame().trn();
            //convert righthand quaternion to lefthand quaternion
            output_POSE[0] = -1 * pos.x(); output_POSE[1] = pos.y(); output_POSE[2] = pos.z();
            //joints local rotation
            for (int i = 0; i < mbs_tar->numLinks(); i++) {
                arma::vec Quat(4);
                mbs_tar->link(i)->getSafeCoordArray(Quat.memptr());

                //std::cout << " DLL " << mbs_tar->link(i)->name() << std::endl;
                gQuat quat_rH = gQuat(Quat[0], Quat[1], Quat[2], Quat[3]);
                gQuat quat_lH;
                //convert righthand quaternion to lefthand quaternion
                quat_lH.setX(quat_rH.x());
                quat_lH.setY(-1.0 * quat_rH.y());
                quat_lH.setZ(-1.0 * quat_rH.z());
                quat_lH.setW(quat_rH.w());

                //convert righthand quaternion to lefthand quaternion
                output_POSE[3 + 4 * i + 0] = quat_lH.x(); output_POSE[3 + 4 * i + 1] = quat_lH.y(); output_POSE[3 + 4 * i + 2] = quat_lH.z(); output_POSE[3 + 4 * i + 3] = quat_lH.w();
            }
        }
        if (mbs_i == 2) {
            mbs->updateKinematicsUptoPos();
            //hip world position
            gVec3 pos = mbs->link(0)->frame().trn();
            //convert righthand quaternion to lefthand quaternion
            output_POSE[0] = -1 * pos.x(); output_POSE[1] = pos.y(); output_POSE[2] = pos.z();
            //joints local rotation
            for (int i = 0; i < mbs->numLinks(); i++) {
                arma::vec Quat(4);
                mbs->link(i)->getSafeCoordArray(Quat.memptr());

                //std::cout << " DLL " << mbs_tar->link(i)->name() << std::endl;
                gQuat quat_rH = gQuat(Quat[0], Quat[1], Quat[2], Quat[3]);
                gQuat quat_lH;
                //convert righthand quaternion to lefthand quaternion
                quat_lH.setX(quat_rH.x());
                quat_lH.setY(-1.0 * quat_rH.y());
                quat_lH.setZ(-1.0 * quat_rH.z());
                quat_lH.setW(quat_rH.w());

                //convert righthand quaternion to lefthand quaternion
                output_POSE[3 + 4 * i + 0] = quat_lH.x(); output_POSE[3 + 4 * i + 1] = quat_lH.y(); output_POSE[3 + 4 * i + 2] = quat_lH.z(); output_POSE[3 + 4 * i + 3] = quat_lH.w();
            }
        }
    }

    


    DLL_EXPORT void UPDATE_JOINT_QUATERNION_UnitytoMW_usingName(LPCSTR jointname, float* input_Quat) {
        gQuat quat; quat.set(input_Quat[0], input_Quat[1], input_Quat[2], input_Quat[3]);
        gRotMat rot_LeftHanded = quat.inRotMatrix();
        gRotMat rot_RightHanded;
        double* convert_L2R = new double[9];
        convert_L2R[0] = rot_LeftHanded.e(0);    convert_L2R[3] = -1 * rot_LeftHanded.e(3); convert_L2R[6] = -1 * rot_LeftHanded.e(6);
        convert_L2R[1] = -1 * rot_LeftHanded.e(1); convert_L2R[4] = rot_LeftHanded.e(4);    convert_L2R[7] = rot_LeftHanded.e(7);
        convert_L2R[2] = -1 * rot_LeftHanded.e(2); convert_L2R[5] = rot_LeftHanded.e(5);    convert_L2R[8] = rot_LeftHanded.e(8);
        rot_RightHanded.set(convert_L2R);

        gXMat mat_G = mbs->findLink(jointname)->localFrame();
        mat_G.setRot(rot_RightHanded);

        mbs->findLink(jointname)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
        mbs->updateKinematicsUptoPos();

    }

    DLL_EXPORT void OUTPUT_JOINT_QUATERNION_UNITY(int jointindex, float* output_Quat) {

        mbs->updateKinematicsUptoPos();

        gRotMat rot_MW = mbs->link(jointindex)->localFrame().rot();

        //
        gQuat quat_rH = rot_MW.inQuat();
        gQuat quat_lH;
        //convert righthand quaternion to lefthand quaternion
        quat_lH.setX(quat_rH.x());
        quat_lH.setY(-1.0 * quat_rH.y());
        quat_lH.setZ(-1.0 * quat_rH.z());
        quat_lH.setW(quat_rH.w());

        output_Quat[0] = quat_lH.x(); output_Quat[1] = quat_lH.y(); output_Quat[2] = quat_lH.z(); output_Quat[3] = quat_lH.w();
        //


    }

    


    
    
    

    

}

```

