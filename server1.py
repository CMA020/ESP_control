import socket
import time
s = socket.socket()

s.bind(('0.0.0.0', 8090))
s.listen(0)

while True:

    client, addr = s.accept()

    while True:
        content = client.recv(32)
        content_str = content.decode('utf-8')
        # print(content)
        # time.sleep(0.5)
        if len(content_str) == 0:
            break

        else:

            print(content_str)

    print("Closing connection")
    client.close()