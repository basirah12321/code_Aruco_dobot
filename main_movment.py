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

def run_point_MOVL(move: DobotApiMove, point_list: list):
    # MovL(self, x, y, z, rx, ry, rz):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5])

def run_point_MOVJ(move: DobotApiMove, point_list: list):
    # MovJ(self, x, y, z, rx, ry, rz):
    move.MovJ(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5])

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

    print("Loop execution.")
    point_a = [-300, -400, 500, -150, -10, 150]
    point_b = [-500, -200, 600, -150, -20, 140]
    point_c = [-472.6138,-140.8185,470.1134,179.9868,0.1351,89.8574]
    point_d = [-472.6138,-140.8185,350,179.9868,0.1351,89.8574]
    
    point_RX_Tool1 = [-472.6138,-140.8185,350,120,0.1351,89.8574]
    point_RX_Tool2 = [-472.6138,-140.8185,350,-120,0.1351,89.8574]
    
    point_RY_Tool1 = [-472.6138,-140.8185,350,179.9868,50,89.8574]
    point_RY_Tool2 = [-472.6138,-140.8185,350,179.9868,-50,89.8574]

    try:
        while True:   
            run_point_MOVL(move, point_a)
            wait_arrive(point_a)
            run_point_MOVL(move, point_b)
            wait_arrive(point_b)
            
            run_point_MOVJ(move, point_a)
            wait_arrive(point_a)
            run_point_MOVJ(move, point_b)
            wait_arrive(point_b) 
            
            run_point_MOVJ(move, point_c)
            wait_arrive(point_c)
            
            run_point_MOVJ(move, point_d)
            wait_arrive(point_d)
            
            run_point_MOVJ(move, point_RX_Tool1)
            wait_arrive(point_RX_Tool1)
            
            run_point_MOVJ(move, point_d)
            wait_arrive(point_d)
            
            run_point_MOVJ(move, point_RX_Tool2)
            wait_arrive(point_RX_Tool2)
            
            run_point_MOVJ(move, point_d)
            wait_arrive(point_d)
            
            run_point_MOVJ(move, point_RY_Tool1)
            wait_arrive(point_RY_Tool1)
            
            run_point_MOVJ(move, point_d)
            wait_arrive(point_d)
            
            run_point_MOVJ(move, point_RY_Tool2)
            wait_arrive(point_RY_Tool2)
            
            run_point_MOVJ(move, point_d)
            wait_arrive(point_d)
                
    except KeyboardInterrupt:
        dashboard.DisableRobot()
        print("Clear error.")
        dashboard.ClearError()
