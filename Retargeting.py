import TCP as Core_tcp
from MBS.retarget import MBS_Retarget
import json
import ctypes

mbs = MBS_Retarget()
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
        srcMBS = tcp.data_packet['text_mbs_src_txt'].encode('utf-8')
        tarMBS = tcp.data_packet['text_mbs_tar_txt'].encode('utf-8')
        MappingTxt = tcp.data_packet['text_jointmapping_txt'].encode('utf-8')
        mbs.initRetarget(srcMBS,tarMBS,MappingTxt)
    
    if(tcp.data_packet['text_indicator'] == "doRetarget"):
        # float 배열 생성
        float_array = (ctypes.c_float * len(tcp.data_packet['floatArray']))(*tcp.data_packet['floatArray'])
        mbs.updateMBSPoseFromUnity(0,float_array)
        
        # do retarget
        base_offset = (ctypes.c_float * len(tcp.data_packet['base_offset']))(*tcp.data_packet['base_offset'])
        base_offset[0] = -base_offset[0] # left-hand coordinate to right-hand coordinate
        mbs.doRetarget(base_offset)

        # output mbs tar
        float_output_array = (ctypes.c_float * (mbs.numlinks*4+3))()
        mbs.exportUnityPoseFromMBS(1,float_output_array)

        # float 배열을 바이트로 변환
        tcp.send(float_output_array)

tcp.client_socket.close()