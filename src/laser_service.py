#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from FRTF20blueliningmaxIV.srv import laser_serviceResponse, laser_service
import socket

class UDP_connect:
    def __init__(self, ip, port, buffersize):
        self._ip = ip
        self._port = port
        self._buffersize = buffersize

        
    def get_message(self):
        UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        UDPServerSocket.bind((self._ip, self._port))
        UDPServerSocket.settimeout(5)

        try:
            bytesAddressPair = UDPServerSocket.recvfrom(self._buffersize)
        except socket.timeout:
            return False 
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        return [message, address]

def callback(msg):
    response = laser_serviceResponse()
    response.success = True

    distancesMessage = distancesUDP.get_message()
    positionMessage = positionUDP.get_message()
    if not distancesMessage or not positionMessage:
        response.success = False

    else:
        # TODO: Might need to consider timestamps but we think it should work without
        # X,9.6879549857494169e+002,Y,-2.1040698523077076e+003,Z,-6.7986554237869018e+002 example line from laser
        distancesSplit = distancesMessage[0].split(",")
        distanceX = float(distancesSplit[1])
        distanceZ = float(distancesSplit[3])

        distances = Pose()
        distances.position.x = distanceX
        distances.position.z = distanceZ

        positionSplit = positionMessage[0].split(",")
        positionX = float(positionSplit[1][0:4])
        positionZ = float(positionSplit[5][0:4])

        position = Pose()
        position.position.x = positionX
        position.position.z = positionZ

        response.distanceToGo = distances
        response.position = position

    return response

rospy.init_node('laser_service', anonymous=True)
srv = rospy.Service('laser_service', laser_service, callback)

localIP     = "192.168.100.102"
localPortDistances = 65432
localPortPosition = 65433
bufferSize  = 1024

# Distances are the error in different axis to the point we are going to
distancesUDP = UDP_connect(localIP, localPortDistances, bufferSize)
# Position will be the coordinates we are currently at according to the laser
positionUDP = UDP_connect(localIP, localPortPosition, bufferSize)

rospy.spin()
