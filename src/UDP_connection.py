#!/usr/bin/env python

import socket

class UDP_connect:
    def __init__(self, ip, port, buffersize):
        self._ip = ip
        self._port = port
        self._buffersize = buffersize

        self._UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self._UDPServerSocket.bind((self._ip, self._port))

    def get_message(self):
        bytesAddressPair = self._UDPServerSocket.recvfrom(self._buffersize)
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        return [message, address]