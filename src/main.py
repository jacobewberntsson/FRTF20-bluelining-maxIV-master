#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from FRTF20blueliningmaxIV.srv import laser_service, action_service
# For testing the action service
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# We need to establish directions, simple test drive forwards/left 
# and see how position of robot changes in laser coordinates
def init():
    print("SEEKING COORDINATE TRANSFORM")

    # Get position of robot from laser service.
    laserService = rospy.ServiceProxy('laser_service', laser_service)
    response = laserService()
    positionOld = response.position

    # Move forward in X axis 1 second
    msg = Twist()
    msg.linear.x = 1
    pub.publish(msg)
    rospy.sleep(1)
    msg.linear.x = 0
    pub.publish(msg)
    rospy.sleep(0.5)

    # Get new position from laser service
    response = laserService()
    positionNew = response.position

    # Derive the movement
    movement = positionNew.position.x - positionOld.position.x
    if movement > 0:
        signs[0] = 1
    else:
        signs[0] = -1

    # Get position of robot from laser service.
    success = False
    while not success:
    	response = laserService()
        success = response.success
    	positionOld = response.position

    # Move forward in X axis 1 second
    msg = Twist()
    msg.linear.y = 1
    msg.angular.z = 0.15
    pub.publish(msg)
    rospy.sleep(1)
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.sleep(0.5)

    # Get new position from laser service
    response = laserService()
    positionNew = response.position

    # Derive the movement
    movement = positionNew.position.z - positionOld.position.z
    if movement > 0:
        signs[1] = 1
    else:
        signs[1] = -1

    # Calculate rotation
    imuOld = rospy.wait_for_message("/imu", Imu)
    # Rotate positive
    msg = Twist()
    msg.angular.z = 0.1
    pub.publish(msg)
    rospy.sleep(1)
    msg.angular.z = 0
    pub.publish(msg)
    rospy.sleep(0.5)

    imuNew = rospy.wait_for_message("/imu", Imu)

    # Calculate the rotation
    rotation = imuNew.orientation.w - imuOld.orientation.w
    if rotation > 0:
        rotSign = 1
    else:
        rotSign = -1

    return signs, rotSign

rospy.init_node('main', anonymous=True)
rospy.wait_for_service('action_service')
rospy.wait_for_service('laser_service')

# Initial orentation
imuInit = rospy.wait_for_message("/imu", Imu)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(10)

# Signs for forward/right
signs = [1, 1]
# Signs for rotation
rotSign = 1

# signs, rotSign = init()

# For testing our action server without laser
# xs = [1000, 500, 300, 200, -100, 0]
# zs = [700, 700, 400, 100, -50, 0]
# index = 0
# time = 0

# start stop topic
start = False
def start_callback(msg): # empty callback
    global start
    start = msg.data
startSub = rospy.Subscriber("/start", Bool, start_callback)

signs, rotSign = init()
speed = 10
while not rospy.is_shutdown():
    msg = Twist() # Message to move the robot
    if start:

        #laserReq = laser_serviceRequest()
        laserService = rospy.ServiceProxy('laser_service', laser_service)
	response = laserService()

        # response = True
        if not response.success: # this should be in when we have laser
            rospy.logdebug("Laser connection seems to be down")
        
        else:
            actionService = rospy.ServiceProxy('action_service', action_service)
            actionReq = response.distanceToGo

        if (actionReq.position.x**2 + actionReq.position.z**2)**0.5 > 300:
            speed = 10
        elif (actionReq.position.x**2 + actionReq.position.z**2)**0.5 > 100:
            speed = 5
        else:
            speed = 1
        # actionReq.position.x = xs[index]
        # actionReq.position.z = zs[index]
        action = actionService(actionReq)

        # time = 1 + time
        # rospy.loginfo(time)

        # if index < 5 and time % 100 == 0:
        #     index = 1 + index

        action = action.action

        # Check if robot has moved to close to the laser
        # position = response.position
        # if (position.position.x**2 + position.position.z**2)**0.5 < 1 # Needs to be a value that represents the closest distance the robot is allowed to go to the laser. Probably in meters or mm
        #       action = "stop"

        # Get orentation
        imu = rospy.wait_for_message("/imu", Imu)
        rotation = imu.orientation.w - imuInit.orientation.w 


        if rotation > 0 and abs(rotation) > 0.01:
            msg.angular.z = rotSign * -0.1
        elif rotation < 0 and abs(rotation) > 0.01:
            msg.angular.z = rotSign * 0.1
        elif action == "forward":
            msg.linear.x = signs[0] * 0.1 * speed
        elif action == "backward":
            msg.linear.x = signs[0] * -0.1 * speed
        elif action == "left":
            msg.linear.y = signs[1] * 0.1 * speed
            msg.angular.z = signs[1] * 0.015 * speed
        elif action == "right":
            msg.linear.y = signs[1] * -0.1 * speed
            msg.angular.z = signs[1] * -0.015 * speed

    pub.publish(msg)

    rate.sleep()
