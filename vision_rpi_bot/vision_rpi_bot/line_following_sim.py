import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class camera_sub(Node):

    def __init__(self):
        super().__init__('qr_maze_solving_node')
        self.camera_sub = self.create_subscription(Image, '/vision_rpi_bot_camera/image_raw', self.camera_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_msg=Twist()
        self.bridge = CvBridge()    

    # def Motor_Steer (self, speed, steering):
    #     if steering == 0:
    #         self.vel_msg.linear.x = speed
    #         return 
    #     elif steering > 0:
    #         steering = 100 - steering

    #         self.vel_msg.linear.x = speed
    #         self.vel_msg.angular.z = -(speed*steering/100)

    #         return
    #     elif steering < 0:
    #         steering = steering * -1
    #         steering = 100 - steering

    #         self.vel_msg.linear.x = speed
    #         self.vel_msg.angular.z = (speed*steering/100)

    #         return 

        

    def camera_cb(self, data):
        x_last = 320
        y_last = 180

        kp = 1.75
        ap = 3

        camera = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        # camera = cv2.resize(camera, (320, 200))
        # print("Camera resolution: {} x {}".format(camera.shape[1], camera.shape[0]))
        Blackline = cv2.inRange(camera, (0,0,0), (60,60,60))
        Greensign = cv2.inRange(camera, (0, 65, 0), (100, 200, 100))
        kernel = np.ones((3, 3), np.uint8)
        Blackline = cv2.erode(Blackline, kernel, iterations=5)
        Blackline = cv2.dilate(Blackline, kernel, iterations=9)
        Greensign = cv2.erode(Greensign, kernel, iterations=5)
        Greensign = cv2.dilate(Greensign, kernel, iterations=9)	
        # img_blk,contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        contours = []
        if int(cv2.__version__[0]) >= 4:
            contours, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, contours, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours_blk = []
        if int(cv2.__version__[0]) >= 4:
            contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours_grn = []
        if int(cv2.__version__[0]) >= 4:
            contours_grn, _ = cv2.findContours(Greensign.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, contours_grn, _ = cv2.findContours(Greensign.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        Greendected = False  # Initialize the variable

        if len(contours_grn) > 0:
            Greendected = True	
            x_grn, y_grn, w_grn, h_grn = cv2.boundingRect(contours_grn[0])
            centerx_grn = x_grn + (w_grn/2)	   
            cv2.line(camera, (int(centerx_grn), 200), (int(centerx_grn), 250), (0, 0, 255), 3)

        contours_blk_len = len(contours_blk)
        if len(contours_blk) > 0:
            x_blk, y_blk, w_blk, h_blk = cv2.boundingRect(contours_blk[0])
            centerx_blk = x_blk + (w_blk/2)

            #3rd addition
            if contours_blk_len == 1:
                blackbox = cv2.minAreaRect(contours_blk[0])
            else:
                canditates = []
                off_bottom = 0
                for con_num in range(contours_blk_len):
                    blackbox = cv2.minAreaRect(contours_blk[con_num])
                    (x_min, y_min), (w_min, h_min), ang = blackbox
                    box = cv2.boxPoints(blackbox)
                    (x_box,y_box) = box[0]
                    if y_box > 358 :
                        off_bottom += 1
                    canditates.append((y_box,con_num,x_min,y_min))	
                canditates = sorted(canditates)

                if off_bottom > 1:
                    canditates_off_bottom=[]
                    for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
                        (y_highest,con_highest,x_min, y_min) = canditates[con_num]
                        total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
                        canditates_off_bottom.append((total_distance,con_highest))
                    canditates_off_bottom = sorted(canditates_off_bottom)
                    (total_distance,con_highest) = canditates_off_bottom[0] 
                    blackbox = cv2.minAreaRect(contours_blk[con_highest])	
                else:
                    (y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]
                    blackbox = cv2.minAreaRect(contours_blk[con_highest])
            
            (x_min, y_min), (w_min, h_min), ang = blackbox
            x_last = x_min
            y_last = y_min
            if ang < -45 :
                ang = 90 + ang
            if w_min < h_min and ang > 0:	
                ang = (90-ang)*-1
            if w_min > h_min and ang < 0:
                ang = 90 + ang
            setpoint = 320
            error = int(x_min - setpoint) 
            ang = int(ang)
            ang = int(ang)	 
            if (ang == 90) :
                ang = ang - 1
            if (ang < 90):
                ang = 90 - ang
            if (ang > 90) :
                ang = 180 - ang
        
            speed = 0.6
            steering = (error*kp) + (ang * ap)

            if steering == 0:
                self.vel_msg.linear.x = speed #straight
             
            elif steering > 0:
                steering = 100 - steering

                self.vel_msg.linear.x = speed
                self.vel_msg.angular.z = (speed*steering/100) #z is for turning

                
            elif steering < 0:
                steering = steering * -1
                steering = 100 - steering

                self.vel_msg.linear.x = speed
                self.vel_msg.angular.z = -(speed*steering/100)

            self.cmd_vel_pub.publish(self.vel_msg)
            
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            cv2.drawContours(camera,[box],0,(0,0,255),3)	 
            cv2.putText(camera,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(camera,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.line(camera, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)	
            #2nd addition 	   

        if Greendected: 
            if centerx_grn > centerx_blk:
                cv2.putText(camera, "Turn Right", (350, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            else:
                cv2.putText(camera, "Turn Left", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
        # else:       
        #     setpoint = 320
        #     error = centerx_blk - setpoint
        #     centertext = "Error = " + str(error)
        #     cv2.putText(camera, centertext, (200, 340), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3)

        cv2.imshow("original with line", camera)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    sensor_sub = camera_sub()

    rclpy.spin(sensor_sub)
    sensor_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class CameraSub(Node):

#     def __init__(self):
#         super().__init__('qr_maze_solving_node')
#         self.camera_sub = self.create_subscription(Image, '/vision_rpi_bot_camera/image_raw', self.camera_cb, 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.vel_msg = Twist()
#         self.bridge = CvBridge()
#         self.frames = []  # stores the video sequence for the demo

#     def camera_cb(self, data):
#         frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
#         h, w = gray.shape
#         start_height = h - 5  # Scan index row 235

#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

#         signed_thresh = thresh[start_height].astype(np.int16)
#         diff = np.diff(signed_thresh)

#         points = np.where(np.logical_or(diff > 200, diff < -200))

#         if len(points) > 0 and len(points[0]) > 1:
#             middle = (points[0][0] + points[0][1]) / 2

#             error = middle - w / 2
#             print("Error -> ", error)

#             if error < 0:
#                 self.vel_msg.angular.z = -0.5
#             else:
#                 self.vel_msg.angular.z = 0.5

#             self.vel_msg.linear.x = 0.4
#             self.cmd_vel_pub.publish(self.vel_msg)

#         cv2.line(frame_rgb, (0, start_height), (w, start_height), (0, 255, 0), 1)

#         cv2.imshow('Frame', frame)
#         cv2.imshow('Threshold', thresh)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     sensor_sub = CameraSub()
#     rclpy.spin(sensor_sub)
#     sensor_sub.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class CameraSub(Node):

#     def __init__(self):
#         super().__init__('line_following_node')
#         self.camera_sub = self.create_subscription(Image, '/vision_rpi_bot_camera/image_raw', self.camera_cb, 10)
#         self.bridge = CvBridge()
#         self.frames = []

#     def camera_cb(self, data):
#         frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         h, w = gray.shape
#         bytes_per_frame = w * h

#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

#         signed_thresh = thresh[-5].astype(np.int16)
#         diff = np.diff(signed_thresh)

#         points = np.where(np.logical_or(diff > 200, diff < -200))

#         if len(points) > 0 and len(points[0]) > 1:
#             middle = (points[0][0] + points[0][1]) / 2

#             # Implement your motor control logic here
#             print("Implement motor control logic with the calculated 'middle' value")

#         else:
#             # Adjust the following lines according to your requirements
#             # start_height -= 5
#             # start_height = start_height % h
#             print("Line lost, implement necessary logic here")

#         frames.append(frame_rgb)
#         frames.append(thresh)
#         if psutil.virtual_memory().percent >= 85:
#             del frames[0]

# def main(args=None):
#     rclpy.init(args=args)
#     sensor_sub = Came(total_distance,con_highest) = canditates_off_bottom[0] raSub()
#     rclpy.spin(sensor_sub)
#     sensor_sub.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2

# class camera_sub(Node):

#     def __init__(self):
#         super().__init__('qr_maze_solving_node')
#         self.camera_sub = self.create_subscription(Image,'/vision_rpi_bot_camera/image_raw',self.camera_cb,10)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.vel_msg = Twist()
#         self.bridge = CvBridge()

#     def detect_right_turn(self, white_index):
#         if len(white_index) > 3:  # Adjust threshold as needed
#             consecutive_diff = [white_index[i+1] - white_index[i] for i in range(len(white_index)-1)]
#             if all(diff > 50 for diff in consecutive_diff):  # Adjust threshold as needed
#                 return True
#         return False

#     def camera_cb(self, data):
#         frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
#         frame = frame[290:479, 130:400]  # First is Y and second is X
#         edged = cv2.Canny(frame, 60, 100)

#         white_index = [index for index, value in enumerate(edged[172]) if value == 255]
#         mid_point_line = 0  # Initialize mid_point_line

#         if len(white_index) == 2:
#             mid_point_line = int((white_index[0] + white_index[1]) / 2)
#             cv2.circle(img=edged, center=(white_index[0], 172), radius=2, color=(255, 0, 0), thickness=1)
#             cv2.circle(img=edged, center=(white_index[1], 172), radius=2, color=(255, 0, 0), thickness=1)
#             cv2.circle(img=edged, center=(mid_point_line, 172), radius=3, color=(255, 0, 0), thickness=2)

#         mid_point_robot = [135, 172]
#         cv2.circle(img=edged, center=(mid_point_robot[0], mid_point_robot[1]), radius=5, color=(255, 0, 0), thickness=2)
#         error = mid_point_robot[0] - mid_point_line if mid_point_line != 0 else 0

#         if self.detect_right_turn(white_index):
#             self.vel_msg.angular.z = 0.5
#             self.vel_msg.linear.x = 0.2  # Adjust linear velocity during turns
#         else:
#             if error < 0:
#                 self.vel_msg.angular.z = -0.5
#             else:
#                 self.vel_msg.angular.z = 0.5
#             self.vel_msg.linear.x = 0.4

#         self.cmd_vel_pub.publish(self.vel_msg)

#         cv2.imshow('Frame', frame)
#         cv2.imshow('Canny Output', edged)
#         cv2.waitKey(1)


# def main(args=None):
#     rclpy.init(args=args)

#     sensor_sub = camera_sub()

#     rclpy.spin(sensor_sub)
#     sensor_sub.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
