import socket

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

    s.connect(('192.168.25.238', 5005))
    s.sendall(b'hellohello')
    data = s.recv(1024)
    print(repr(data))
