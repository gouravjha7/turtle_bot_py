# Setup:
#  roscore
#  rosrun turtlesim turtlesim_node
#  . ~/catkin_ws/devel/setup.bash 
#  rosrun turtle_boy_py main.py

#Goal1:
#     1. Uncomment theturtle_spawn
#     2.  Uncomment move_to_desire_location()
#     3.  Uncomment x = turtle_PID() and x.statrt()

# #Goal2:
#     1. Uncomment move_in_circle
#     2. Uncomment move_turtle




import rospy
import math
from math import atan2, sqrt
from std_msgs.msg import Float32
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import time
import sys
class turtle_PID():           
    def callback(self,data):                                 #Update value of poses
        self.position=data
    
    def start(self):
        rospy.init_node('goal_nav')
        self.position=Pose()
        self.velocity_pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.rate=rospy.Rate(10)                         
        self.pose_sub=rospy.Subscriber('/turtle1/pose',Pose,self.callback)	
        goal=Pose()
        goal.x=float(sys.argv[1])
        goal.y=float(sys.argv[2])
        start_time=time.time()
        vel=Twist()
        Kp=1
        Kd=0.1
        Ki=0.0001
        dist_prev=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
        error=0           										#Initialise error summation terms for integral control
        steer_sum=0
        steer_prev=(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)
        while(True):
            error=error+dist_prev
            dist=sqrt((self.position.x-goal.x)**2+(self.position.y-goal.y)**2)
            dedt=dist-dist_prev	
            vel.linear.x=Kp*(dist)+Kd*dedt+Ki*error							#PID Implementation
            steer=(atan2(goal.y - self.position.y, goal.x - self.position.x)-self.position.theta)   #Angle needed to turn
            dsteer=steer-steer_prev
            vel.angular.z=6*Kp*steer+Kd*dsteer+Ki*steer_sum						#PID implementation
            self.velocity_pub.publish(vel)
            self.rate.sleep()
            dist_prev=dist
            steer_prev=steer
            steer_sum=steer_sum+steer_prev
            if(dist<0.5):
                break

def demo_move_turtle(velocity_pub,line_vel,ang_vel):
    rate = rospy.Rate(10)
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = line_vel
        vel.linear.y = 0
        vel.linear.z = 0		
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        velocity_pub.publish(vel)
        rate.sleep()	

def poseCallback(pose_message):
    global x
    global y, yaw 
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

def move_to_desire_location(velocity_pub, x_coordinate, y_coordinate):
    global x 
    global y, yaw 

    velocity_mess = Twist()

    while not rospy.is_shutdown():
        k_linear = 0.5
        distance = abs(math.sqrt(((x_coordinate-x)**2) + ((y_coordinate-y)**2)))
        linear_speed = distance * k_linear

        k_angular = 4.0 
        desire_angle = math.atan2(y_coordinate-y,x_coordinate-x)
        angular_speed = (desire_angle-yaw)*k_angular

        velocity_mess.linear.x = linear_speed
        velocity_mess.angular.z = angular_speed


        velocity_pub.publish(velocity_mess)
        print('x=',x,', y=',y, 'distance to goal: ',distance)

        if(distance<0.01):
            break

def move_in_circle(velocity_pub,radius,speed,is_clockwise):
    vel= Twist()
    global x,y 
    x0 = x
    y0 = y  

    if(is_clockwise):
        vel.linear.x = abs(speed)
    else:
        vel.linear.x = -abs(speed)
    while not rospy.is_shutdown():
        vel.linear.x = radius
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = speed
        rospy.loginfo("Speed = %f", speed)		
        velocity_pub.publish(vel)
        time.sleep(1)

def rotate(velocity_pub,angular_speed_degree,relative_angle_degree, clockwise):

    velocity_mess = Twist()
    angular_speed = math.radians(abs(angular_speed_degree))
    global x, y 
    x0 = x 
    y0 = y 

    if(clockwise):
        velocity_mess.angular.z = -abs(angular_speed)
    else:
        velocity_mess.angular.x = abs(angular_speed)


    angle_moved = 0.0	  
    loop_rate = rospy.Rate(100)
    # cmd_vel_topic = 'turtle1/cmd_vel'
    # velocity_pub = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        rospy.loginfo("rotating")
        velocity_pub.publish(velocity_mess)

        t1 = rospy.Time.now().to_sec()
        current_angle_d = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if(current_angle_d > relative_angle_degree):
            # rospy.loginfo("reached")
            break

    velocity_mess.angular.z = 0
    velocity_pub.publish(velocity_mess)

def turtle_spawn():
    rospy.init_node('turtle_spawn')
    rospy.wait_for_service('/spawn')
    try:
        add_turtle = rospy.ServiceProxy('/spawn', Spawn)
        response = add_turtle(2.0, 2.0, 0.0, "turtle2")
        return response.name
    except rospy.ServiceException as e:
        print("ailed: %s"%e)

def move(velocity_pub, speed, distance, is_forward):
    velocity_mess= Twist()
    global x,y 
    x0 = x
    y0 = y  

    if(is_forward):
        velocity_mess.linear.x = abs(speed)
    else:
        velocity_mess.linear.x = -abs(speed)
    
    distance_moved = 0.0
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Turtlesim Moves forward")
        velocity_pub.publish(velocity_mess)
        loop_rate.sleep()
        distance_moved = abs(math.sqrt(((x-x0) **2)+ ((y-y0) ** 2)))
        print(distance_moved)
        if not(distance_moved<=distance):
            rospy.loginfo("reached")
            break
    velocity_mess.linear.x = 0
    velocity_pub.publish(velocity_mess)

def move_turtle(velocity_pub, speed, distance, is_forward):
    velocity_mess= Twist()
    global x,y 
    x0 = x
    y0 = y  

    if(is_forward):
        velocity_mess.linear.x = abs(speed)
    else:
        velocity_mess.linear.x = -abs(speed)
    
    distance_moved = 0.0
    loop_rate = rospy.rate(10)

    while True:
        rospy.loginfo("Turtlesim Moves forward")
        velocity_pub.publisher(velocity_mess)
        loop_rate.sleep()
        distance_moved = abs(math.sqrt(((x-x0) **2)+ ((y-y0) ** 2)))
        print(distance_moved)
        if not(distance_moved<distance):
            rospy.info("reached")
            break
    velocity_mess.linear.x = 0
    velocity_pub.publisher.publish(velocity_mess)

if __name__ == '__main__' :	
    try:
        # rospy.init_node('turtlesim_motion_pose',anonymous=True)		
        # velocity_pub  = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)		
        # pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose,poseCallback)
        time.sleep(1)
        x = turtle_PID()
        
        #move(velocity_pub,3.0,4.0,False)
        #rotate(velocity_pub,90,90,True)
        #move_to_desire_location(velocity_pub,1,1)
        #move_in_circle(velocity_pub,float(sys.argv[1]),float(sys.argv[2], True)) #GOAL 2
        #demo_move_turtle(velocity_pub,2,6)
        #turtle_spawn()
        #turtlesim_follower()
        x.start()
    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated')

