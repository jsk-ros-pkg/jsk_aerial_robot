#!/usr/bin/env python

from __future__ import print_function
import socket
from contextlib import closing

host = '192.168.25.100'
port = 12345
bufsize = 4096

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((host, port))
while True:
    #print(sock.recv(bufsize))
    print(int.from_bytes(sock.recv(bufsize), 'little'))

