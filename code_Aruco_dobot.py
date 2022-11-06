from logging import exception
import cv2
import cv2.aruco as aruco # pip3 install opencv-contrib-python
import pyrealsense2 as rs # pip3 install pyrealsense2
import numpy as np
from scipy.spatial import distance as dist
import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep




current_actual = None

def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):

    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(x, y)  # Get the depth corresponding to the pixel
    # print ('depth: ',dis) #  The unit of depth is m
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    #print ('camera_coordinate: ',camera_coordinate)
    
    return dis, camera_coordinate

def get_aligned_images():
    frames = pipeline.wait_for_frames()  # Wait for image frame , Get the frameset of color and depth
    aligned_frames = align.process(frames)  # Get alignment frame , Align the depth box with the color box

    aligned_depth_frame = aligned_frames.get_depth_frame()  # Gets the in the alignment frame depth frame
    aligned_color_frame = aligned_frames.get_color_frame()  # Gets the in the alignment frame color frame

    ####  Get camera parameters  ####
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # Get the depth parameter （ Pixel coordinate system to camera coordinate system will use ）
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # Get camera internal parameters

    ####  take images To numpy arrays ####
    img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB chart
    img_depth = np.asanyarray(aligned_depth_frame.get_data())  # Depth map （ Default 16 position ）

    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame

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
            #print("============== Feed Back ===============")
            current_actual = a["tool_vector_actual"][0]
            #print("tool_vector_actual: [X:{0}] , [Y:{1}] , [Z:{2}] , [RX:{3}] , [RY:{4}] , [RZ:{5}]".format(current_actual[0],current_actual[1],current_actual[2],current_actual[3],current_actual[4],current_actual[5]))
            

            CR_joint = a['q_target'][0]
            #print("CR_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}] , [j5:{4}] , [j6:{5}]".format(CR_joint[0],CR_joint[1],CR_joint[2],CR_joint[3],CR_joint[4],CR_joint[5]))
            #print("========================================")
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

#Start realsense pipeline 
pipeline= rs.pipeline()
config= rs.config()

#Eneble device id for more than 1 realsense
#config_1.enable_device('018322070394')

#confing resolution of realsense
rs_w=640
rs_h=480
fps=15

#eneble video stream color and depth
config.enable_stream(rs.stream.depth, rs_w, rs_h, rs.format.z16, fps)
config.enable_stream(rs.stream.color, rs_w, rs_h, rs.format.bgr8, fps)
pipeline.start(config)

align_to = rs.stream.color  # align_to  Is the stream type for which the depth frame is scheduled to be aligned
align = rs.align(align_to)  # rs.align  Align the depth frame with other frames

#funtion for easily concate video or image
def concat_tile(im_list_2d):
	return cv2.vconcat([cv2.hconcat(im_list_h) for im_list_h in im_list_2d])

#name window that show output
cv2.namedWindow("Aruco", cv2.WINDOW_AUTOSIZE)

#aruco funtion to call Ditionary 5x5
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
parameters = aruco.DetectorParameters_create()

parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 400

marker = aruco.drawMarker(aruco_dict, 200, 200)
marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

showLive=True

#set font of all text
font=cv2.FONT_HERSHEY_SIMPLEX

if __name__ == '__main__':
    # Robot Setup
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

    try:
        #main function
        while (showLive):
            
            #wait for realsense frame input. if not it will crash out
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()


            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())


            #rearrange depth_image to color image
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_JET)

            Frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            #get info of aruco 
            corners, ids, rejected = aruco.detectMarkers(Frame, aruco_dict, parameters=parameters)

            #Put text of aruco into image
            for (i, b) in enumerate(corners):

                #get coordinate center of aruco  to put text
                c1 = (b[0][0][0], b[0][0][1])
                c2 = (b[0][1][0], b[0][1][1])
                c3 = (b[0][2][0], b[0][2][1])
                c4 = (b[0][3][0], b[0][3][1])

                x_cam1 = int((c1[0] + c2[0] + c3[0] + c4[0]) / 4)
                print(x_cam1)
                y_cam1 = int((c1[1] + c2[1] + c3[1] + c4[1]) / 4)
                print(y_cam1)
                z_cam1 = depth_frame.get_distance(int(x_cam1),int(y_cam1))
                print(z_cam1)

                color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()
                dis, camera_coordinate = get_3d_camera_coordinate((x_cam1,y_cam1), aligned_depth_frame, depth_intrin)
                x_cam=camera_coordinate[0]*100
                y_cam=camera_coordinate[1]*100
                z_cam=camera_coordinate[2]*100

                data_string = "ID:" + str(ids[i]) + ",(" + str(x_cam) + "," + str(y_cam) + "," + str("{:.2f}".format(z_cam)) + ")"
                frame = cv2.putText(color_image, data_string, (x_cam1-30, y_cam1), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2, cv2.LINE_AA)

                # from experiment
                x_robot = (9.790*y_cam) + 668.79
                print(x_robot)
                y_robot = (9.928*x_cam) - 179.88
                print(y_robot)
                z_robot = (-9.987*z_cam) + 950.40
                print(z_robot)

                point1 = [x_robot, y_robot, z_robot+20, -179, 0, -90]
                
                point_home = [473.58, -140.29, 469.22, -179, 0.27, -90]
                run_point_MOVJ(move, point1)
                wait_arrive(point1)
                sleep(1)
                run_point_MOVJ(move, point_home)
                wait_arrive(point_home)

            #concate color image and depth image
            show=concat_tile([[color_image,depth_colormap]])

            #show output image
            cv2.imshow('TEST', show)

            #wait key for exit
            key = cv2.waitKey(10)
            if key & 0xFF == ord('q') or key == 27:
                showLive = False
                break


            pipeline.stop()
            cv2.destroyAllWindows()

    except:
        dashboard.DisableRobot()
        print(exception)
        print("Clear error.")
        dashboard.ClearError()