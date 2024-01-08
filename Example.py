import TCP as tcp_core
import json

tcp = tcp_core.TCP_Connection('143.248.6.198',80)

while True:
    tcp.recieve()

    if not tcp.data:
        break

    try :
        tcp.recieve_Json()
    except(json.JSONDecodeError) as e:
        print(f"예외 발생 : {e}")

    if(tcp.data_packet['text_indicator'] == "initMBS"):
        print("data_is_connected")