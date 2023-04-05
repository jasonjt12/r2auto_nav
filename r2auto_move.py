import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Bool
import numpy as np
import pickle
import cmath
# import RPI.GPIO as GPIO
import paho.mqtt.client as mqtt
with open("waypoints.pickle","rb") as handle:
    waypoints = pickle.load(handle)

print(waypoints)
# mapfile = 'map.txt'
rotatechange = 0.3
speedchange = 0.1
angle_error = 2
route = {1:[1, 0],2:[2, 0],3:[2, 0],4:[3,0],5:[4,0],6:[5, 6, 10,  0]}
# print("in in in ")
count = 0

# quad_1 = range(0, 0.5 * pi)
# quad_2 = range (0.5 * pi, pi)
# quad_3 = range(pi, -0.5 * pi)
# quad_4 = range(-0.5 * pi, -pi)

# waypoints = {1:[-2.5769601779175844, -1.23103603909631],2:[-2.1849758576303553, -0.5775387034088549]}

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    orientation = math.atan2(t3, t4)

    return orientation # in radians

class Auto_Mover(Node):
    def __init__(self):
        self.table = 0
        #self.rot_q = 0.0
        self.orien = 0.0
        #count = 0
        self.front = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.mapbase = None
        super().__init__('auto_mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel',5)
        self.userinput = self.create_subscription(String,'TableNo', self.user_sub, 5)
        self.map2base_sub = self.create_subscription(Pose, 'map2base', self.map2base_callback, 5)
        self.lidarsub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)  
        self.cansub = self.create_subscription(Bool, 'can', self.can_callback, 2)

    def map2base_callback(self, msg):
            # self.get_logger().info('In map2basecallback')
            self.mapbase = msg.position
            self.orien = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)    
        
    def user_sub(self, msg):
        self.table = int(msg.data)
        # print("in subcriber user")    

    def scan_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        positive_range = laser_range[-20:-1]
        # print(laser_range[0])
        # taken_range = np.add(taken_range, laser_range[0:16])
        other_range = (laser_range[0:20])
        taken_range = np.append(other_range , positive_range)
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        # self.front = laser_range[0]
        lr2i = np.nanargmin(taken_range)
        self.front = taken_range[lr2i]
 
    def can_callback(self, msg):
        self.can = msg.state
    
    def move_to_point(self, point):
        print("tavelling to waypoint")
        # points_char = int(input("enter waypoint to travel: "))
        twist = geometry_msgs.msg.Twist()
        # print("qewagdsfnc")
        
        # rclpy.init_node("speed_controller")
        # r = rclpy.Rate(4)
        self.goal_x = waypoints[point][0][0]
        self.goal_y = waypoints[point][0][1]
        theta = (math.atan2(self.goal_y-self.y,self.goal_x-self.x))
        inc_x = 1000
        degree_to_turn = math.degrees(theta - self.orien )
        sign = np.sign(degree_to_turn)
        try:
            while self.front > 2:
                while abs(int(degree_to_turn*100)) >= 10:
                    theta = (math.atan2(self.goal_y-self.y,self.goal_x-self.x))
                    degree_to_turn = math.degrees(theta - self.orien)
                    sign = np.sign(degree_to_turn)
                    if prev != 0.5 * sign:
                        twist.angular.z = 0.5 * sign
                        twist.linear.x = 0.0
                        self.publisher_.publish(twist)
                        prev = 0.5 * sign
                    else:
                        continue
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            if prev_lin != 0.1:
                twist.angular.z = 0.0
                twist.linear.x = 0.1 #or 0.3
                self.publisher_.publish(twist)
                prev_lin = 0.1
        finally:
            # stop moving   
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
    
    def sequential(self,route):
        for point in route:
            if point == 0 or point == 5:
                self.move_to_point(point)
            else:
                self.move_to_point(point)
                if tableno == 3:
                    #face behind
                elif tableno == 6:
                    #special case
                else:
                    #face forward

                while self.front > 2:


                   
            if tableno == 6:

                self.move_to_point(point)
    
    # def path(self, point):
    #     twist = geometry_msgs.msg.Twist()
    #     # table = 2
    #     # print("table",Table)
    #     try:
    #         while self.front>0.2:
    #             degree_to_turn = math.degrees(theta - self.orien)
    #             while abs(degree_to_turn) >= 0.1:
    #                 theta = (math.atan2(self.goal_y-self.y,self.goal_x-self.x))
    #                 degree_to_turn = math.degrees(theta - self.orien)
    #                 sign = np.sign(degree_to_turn)
    #                 twist.angular.z = 0.5 * sign
    #             twist.linear.x = 0.3 #or 0.1
    #         count = 0
    #         can_in0 = True
    #         can_in1 = True
    #         can_in2 = True
    #         while can_in2 == True and count<=15:
    #             can_in0 = self.can
    #             can_in1 = can_in0
    #             can_in2 = can_in1
    #             count += 1
    #             time.sleep(1)
            
                


            
            
            
            
            
            
            
            
    #         if self.table == 1:
    #             while self.front > 0.2:
    #                     twist.linear.x = 0.3
    #                     twist.angular.z = 0.0
    #                     self.publisher_.publish(twist)
    #             if self.front <= 0.2:
    #                 twist.linear.x =0.0

    #         # if self.table == 4 or 5:
    #         #     self.travelling_point(0)
    #         #     self.goal_y4 = waypoints[3][1][1]
    #         #     while ((int(abs(self.goal_y4)*100) -2 != int(abs(self.y)*100))) :
    #         #         rclpy.spin_once(self)
    #         #         print("moving")
    #         #         print("current y", self.y)
    #         #         print("goal", self.goal_y)
    #         #         twist.linear.x = 0.1
    #         #         twist.angular.z = 0.0    
    #         #         self.publisher_.publish(twist)  
    #         #         if (int(abs(self.goal_y4)*100) -3 == int(abs(self.y)*100)):
    #         #             twist.linear.x = 0.0
    #         #             twist.angular.z = 0.0 
    #         #             self.publisher_.publish(twist) 
    #         #             break

    #         # if self.table == 2 or 3 or 4 or 5 :
    #         #     for points in paths[self.table]:
    #         #         self.travelling_point(points)
    #         #         # if self.table == 4 or 5:
    #         #         #     print("in 4")
    #         #         #     while (abs(int(abs(self.goal_y)*100)-int(abs(self.y)*100 ))>= 3):
    #         #         #         rclpy.spin_once(self)
    #         #         #         twist.linear.x = 0.1
    #         #         #         twist.angular.z = 0.0 
    #         #         #         self.publisher_.publish(twist)
    #         #         #         if (abs(int(abs(self.goal_y)*100)-int(abs(self.y)*100 ))<= 2):
    #         #         #             twist.linear.x = 0.0
    #         #         #             twist.angular.z = 0.0 
    #         #         #             self.publisher_.publish(twist)
    #         #         #             break
                        
    #         #     if self.table == 3:
    #         #         while abs(int(self.orien*100)) <=299 :
    #         #             rclpy.spin_once(self)
    #         #             twist.angular.z = 0.3
    #         #             self.publisher_.publish(twist)
    #         #             print("Turning to table 3")
    #         #             print(math.degrees(self.orien))
    #         #             print( abs(int(self.orien*100)))
    #         #     else:
    #         #         while abs(int(self.orien*100)) >=2 :
    #         #             print("Turning to table")
    #         #             rclpy.spin_once(self)
    #         #             print(math.degrees(self.orien))
    #         #             twist.angular.z = 0.3
    #         #             self.publisher_.publish(twist)
    #         #     while self.front > 0.3:
    #         #         rclpy.spin_once(self)
    #         #         print("moving to table")
    #         #         twist.linear.x = 0.4
    #         #         twist.angular.z = 0.0
    #         #         self.publisher_.publish(twist)
    #         #         if self.front <= 0.2:
    #         #             print("stopping at table")
    #         #             twist.linear.x =0.0
    #         #     new_path = paths[self.table][::-1]
    #         #     if self.table == (4 or 5):
    #         #         new_path[-1] = 7
    #         #     print(new_path)
    #         #     self.run_combi(new_path)
                    
                    
    #         # self.publisher_.publish(twist)   

    #         # if self.table == 0:
    #         #     print("FAILED TO SUBSCIBE TO USER")
    #     finally:
    #         # stop moving   
    #         twist.linear.x = 0.0
    #         twist.angular.z = 0.0
    #         self.publisher_.publish(twist)
    # def on_table_num(client, userdata, msg):
    #     global table_num 
    #     table_num = int(self.userinput)
    #     print(table_num) # added cuz without this IT WONT WORK

    


def main(args = None):

    try:
        rclpy.init(args = args)
        auto_move = Auto_Mover()
        rclpy.spin_once(auto_move)
        while True:
            user_sub()
            tableno = self.table
            if tableno != -1:
                sequential(route[tableno])
                self.
         #print (table_num)
        #  if(table_num != -1):
        #      # send message to esp32 to tell it that the robot has un-docked and is moving to the table
        #      client.publish("esp32/input", "0")
        #      #navigation.moveToTable(table_num)
        #      navigation.dock()
        #      table_num = -1
         
        pass
    except KeyboardInterrupt:
        auto_move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
