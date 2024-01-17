import TCP as Core_tcp
from MBS.desiredIK import MBS_DesIK
import json
import ctypes

mbs = MBS_DesIK()


tcp = Core_tcp.TCP_Connection('143.248.6.198',80)


while True:
    tcp.recieve()

    if not tcp.data:
        break

    try :
        tcp.recieve_Json()
    except(json.JSONDecodeError) as e:
        print(f"예외 발생 : {e}")

    if(tcp.data_packet['text_indicator'] == "initMBS"):
        srcMBS = tcp.data_packet['text_mbs_txt'].encode('utf-8')
        mbs.initIK(srcMBS)
    
    if(tcp.data_packet['text_indicator'] == "SetDesPositons"):
        des_pos_arr = tcp.data_packet['floatArr_Position']
        des_weight_arr = tcp.data_packet['floatArr_WeightsP']
        des_desired_arr = tcp.data_packet['floatArr_DesP']
        
        des_weight_arr = (ctypes.c_float * len(des_weight_arr))(*des_weight_arr)
        des_desired_arr = (ctypes.c_float * len(des_desired_arr))(*des_desired_arr)
        
        mbs.SetDesPositionArray(des_desired_arr,des_pos_arr,des_weight_arr)
        
        print("set Desired Positions")
    
    if(tcp.data_packet['text_indicator'] == "SetDesiredPose"):
        des_point_arr = tcp.data_packet['floatArr_Position']
        des_desP_arr = tcp.data_packet['floatArr_DesP']
        des_weightP_arr = tcp.data_packet['floatArr_WeightsP']

        des_dir_arr = tcp.data_packet['floatArr_Direction']
        des_desD_arr = tcp.data_packet['floatArr_DesD']
        des_weightD_arr = tcp.data_packet['floatArr_WeightsD']
        
        
        des_weightP_arr = (ctypes.c_float * len(des_weightP_arr))(*des_weightP_arr)
        des_weightD_arr = (ctypes.c_float * len(des_weightD_arr))(*des_weightD_arr)
        
        des_desP_arr = (ctypes.c_float * len(des_desP_arr))(*des_desP_arr)
        des_desD_arr = (ctypes.c_float * len(des_desD_arr))(*des_desD_arr)
        
        mbs.SetDesDirectionArray(des_desD_arr,des_dir_arr,des_weightD_arr)
        mbs.SetDesPositionArray(des_desP_arr,des_point_arr,des_weightP_arr)
        
        #print("set Desired Pose")
    
    if(tcp.data_packet['text_indicator'] == "doIK"):

        # do IK
        mbs.doIK()
      
        # output mbs tar
        float_output_array = (ctypes.c_float * (mbs.numlinks*4+3))()
        mbs.exportUnityPoseFromMBS(2,float_output_array)

        # float 배열을 바이트로 변환
        tcp.send(float_output_array)

tcp.client_socket.close()