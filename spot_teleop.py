#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

############### spot_msgs ###############
from spot_msgs.srv import SetObstacleParams, SetObstacleParamsResponse
from spot_msgs.srv import SetVelocity, SetVelocityResponse
from spot_msgs.srv import SetSwingHeight, SetSwingHeightResponse
from spot_msgs.srv import SetLocomotion, SetLocomotionResponse
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from std_msgs.msg import Bool

from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import spot_driver.msg
import spot_driver.srv


import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class keyboard_teleop:

    def __init__(self, config):

        # self.lin_vel = float(config.lin_vel)
        # self.ang_vel = float(config.ang_vel)
        # self.height_up = float(config.height_up)
        # self.height_down = float(config.height_down)

        rospy.init_node('keyboard_teleop')
        self.rate = rospy.Rate(60)  # 60hz is standard for Boston Dynamics robots

        # Initialize service proxies

        # ######################################### Locomotion Services #########################################

        # rospy.Service("self_right", Trigger, self.handle_self_right)
        # rospy.Service("sit", Trigger, self.handle_sit)
        # rospy.Service("stand", Trigger, self.handle_stand)
        # rospy.Service("power_on", Trigger, self.handle_power_on)
        # rospy.Service("power_off", Trigger, self.handle_safe_power_off)
        # rospy.Service("estop/hard", Trigger, self.handle_estop_hard)
        # rospy.Service("estop/gentle", Trigger, self.handle_estop_soft)
        # rospy.Service("estop/release", Trigger, self.handle_estop_disengage)
        # rospy.Service("allow_motion", SetBool, self.handle_allow_motion)
        # rospy.Service("stair_mode", SetBool, self.handle_stair_mode)
        # rospy.Service("locomotion_mode", SetLocomotion, self.handle_locomotion_mode)
        # rospy.Service("swing_height", SetSwingHeight, self.handle_swing_height)
        # rospy.Service("velocity_limit", SetVelocity, self.handle_vel_limit)
        # rospy.Service("obstacle_params", SetObstacleParams, self.handle_obstacle_params)
        # rospy.Service("roll_over_right", Trigger, self.handle_roll_over_right)
        # rospy.Service("roll_over_left", Trigger, self.handle_roll_over_left)

        # # rospy.Service("posed_stand", PosedStand, self.handle_posed_stand)
        # # rospy.Service("terrain_params", SetTerrainParams, self.handle_terrain_params)
        # # rospy.Service("list_graph", ListGraph, self.handle_list_graph)

        #define service proxies
        self.obstacle_params_srv_pub = rospy.ServiceProxy("obstacle_params", spot_driver.srv.SetObstacleParams)

        #define service requests
        self.obstacle_params_srv_req = spot_driver.srv.SetObstacleParamsRequest()


    def obstacle_params_service(self, key_pressed): 
        #req is key pressed: o = 'on', p = 'no_padding', l = 'nothing'

        obstacle_parameters = spot_command_pb2.ObstacleParams()

        if key_pressed == 'o':
            obstacle_parameters.disable_vision_foot_obstacle_avoidance = True
            obstacle_parameters.disable_vision_foot_constraint_avoidance = True
            obstacle_parameters.disable_vision_body_obstacle_avoidance = True
            obstacle_parameters.obstacle_avoidance_padding = 0.5
            obstacle_parameters.disable_vision_foot_obstacle_body_assist = True
            obstacle_parameters.disable_vision_negative_obstacles = True
        
        elif key_pressed == 'p':
            obstacle_parameters.disable_vision_foot_obstacle_avoidance = True
            obstacle_parameters.disable_vision_foot_constraint_avoidance = True
            obstacle_parameters.disable_vision_body_obstacle_avoidance = True
            obstacle_parameters.obstacle_avoidance_padding = 0.0
            obstacle_parameters.disable_vision_foot_obstacle_body_assist = True
            obstacle_parameters.disable_vision_negative_obstacles = True

        elif key_pressed == 'l':
            obstacle_parameters.disable_vision_foot_obstacle_avoidance = False
            obstacle_parameters.disable_vision_foot_constraint_avoidance = False
            obstacle_parameters.disable_vision_body_obstacle_avoidance = False
            obstacle_parameters.obstacle_avoidance_padding = 0.0
            obstacle_parameters.disable_vision_foot_obstacle_body_assist = False
            obstacle_parameters.disable_vision_negative_obstacles = False


        self.obstacle_params_srv_req = obstacle_parameters

        try:
            rospy.wait_for_service("obstacle_params", timeout=2.0)
            self.obstacle_params_srv_pub(self.obstacle_params_srv_req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    settings = keyboard_teleop()

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key in 'opl':
                settings.obstacle_params_service(key)
                print("Obstacle params profile changed to: {}".format(key))
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
