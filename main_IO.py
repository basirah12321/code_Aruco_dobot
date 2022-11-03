import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep
import numpy as np

current_actual = None

def connect_robot():
    try:
        ip = "192.168.1.6" #"192.168.5.1"
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004
        print("Connecting...")
        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print("Connection succeeded.")
        return dashboard, move, feed
    except Exception as e:
        print("Connection failed.")
        raise e

def get_feed(feed: DobotApi):
    global current_actual
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0

        a = np.frombuffer(data, dtype=MyType)
        if hex((a['test_value'][0])) == '0x123456789abcdef':
            # Refresh Properties
            print("============== Feed Back ===============")
            current_actual = a["tool_vector_actual"][0]
            print("tool_vector_actual: [X:{0}] , [Y:{1}] , [Z:{2}] , [RX:{3}] , [RY:{4}] , [RZ:{5}]".format(current_actual[0],current_actual[1],current_actual[2],current_actual[3],current_actual[4],current_actual[5]))
            

            CR_joint = a['q_target'][0]
            print("CR_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}] , [j5:{4}] , [j6:{5}]".format(CR_joint[0],CR_joint[1],CR_joint[2],CR_joint[3],CR_joint[4],CR_joint[5]))
            
            Digital_Input = a['digital_input_bits'][0]
            Digital_Input_array = [(int(Digital_Input)>>(8*i))&0xFF for i in range(7,-1,-1)]
            #print("Digital input: {0}".format(Digital_Input_array))
            
            DI_1 = 0
            DI_2 = 0
            if Digital_Input_array[3]&0x01 == 1:
                DI_1 = 1
            if Digital_Input_array[3]&0x02 == 2:
                DI_2 = 1
             
            print("Digital Input tools: DI_1:[{0}] , DI_2:[{1}]".format(DI_1,DI_2))
            print("========================================")
        sleep(0.001)

def wait_arrive(point_list):
    global current_actual
    while True:
        is_arrive = True
        if current_actual is not None:
            for index in range(len(current_actual)):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False
            if is_arrive:
                return
        sleep(0.001)

if __name__ == '__main__':
    dashboard, move, feed = connect_robot()
    print("Start power up.")
    dashboard.PowerOn()
    print("Please wait patiently,Robots are working hard to start.")
    count = 3
    while count > 0 :
        print(count)
        count = count - 1
        sleep(1)

    #Enable Robot
    print("Clear error.")
    dashboard.ClearError()
        
    print("Start enable.")
    dashboard.EnableRobot()
    print("Complete enable.")

    # Create Thread for reading feedback.
    feed_thread = threading.Thread(target=get_feed, args=(feed,))
    feed_thread.setDaemon(True) # ref. https://www.geeksforgeeks.org/python-daemon-threads/
    feed_thread.start()

    try:
        while True:   
            # open 100%
            dashboard.ToolDO(1,0)
            dashboard.ToolDO(2,0)
            sleep(1)
            
            # open 80%
            dashboard.ToolDO(1,1)
            dashboard.ToolDO(2,0)
            sleep(1)
            
            # open 20%
            dashboard.ToolDO(1,0)
            dashboard.ToolDO(2,1)
            sleep(1)
            
            # open 0%
            dashboard.ToolDO(1,1)
            dashboard.ToolDO(2,1)
            sleep(1)
                
    except KeyboardInterrupt:
        dashboard.DisableRobot()
        print("Clear error.")
        dashboard.ClearError()
