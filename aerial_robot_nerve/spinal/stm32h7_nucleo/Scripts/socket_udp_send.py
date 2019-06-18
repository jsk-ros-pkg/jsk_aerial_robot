import socket
import time

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:

    s.settimeout(1)

    try:
        while True:
            s.sendto(b'hellohello',("192.168.25.238", 5005))
            try:
                data, addr = s.recvfrom(4096)
            except socket.timeout:
                print("caught a timeout at first echo")

            print(repr(data))

            try:
                data, addr = s.recvfrom(4096)
            except socket.timeout:
                print("caught a timeout at second echo")

            print(int.from_bytes(data, 'little'))

            time.sleep(0.01)

    except KeyboardInterrupt:
        exit()
