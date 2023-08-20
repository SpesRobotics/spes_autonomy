import socket
import struct
from datetime import datetime, timedelta


UDP_IP = "127.0.0.1"
UDP_PORT = 5000
PACK_STRING = ">HIHHHHBqqHH"


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 0))
    sock.settimeout(1.0)

    pack_string_size = struct.calcsize(PACK_STRING)

    while True:
        # Sending heartbeat (for hole punching) and receiving messages
        sock.sendto(b"1", (UDP_IP, UDP_PORT))
        data = None
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue

        num_detections = struct.unpack(">H", data[:2])[0]
        data = data[2:]
        for i in range(num_detections):
            tokens = struct.unpack(PACK_STRING, data)
            data = data[pack_string_size:]

            class_ = tokens[0]
            instance = tokens[1]
            center_x = tokens[2]
            center_y = tokens[3]
            height = tokens[4]
            width = tokens[5]
            precision = tokens[6]
            timestamp_sec = tokens[7]
            timestamp_nsec = tokens[8]
            image_width = tokens[9]
            image_height = tokens[10]

            dt = datetime.fromtimestamp(timestamp_sec)
            dt_with_ns = dt + timedelta(microseconds=timestamp_nsec // 1000)
            formatted_time = dt_with_ns.strftime("%H:%M:%S.%f")[:-3]

            print(
                f"class = {class_}, instance = {instance}, center_x = {center_x}, center_y = {center_y}, height = {height}, width = {width}, precision = {precision/100} , timestamp_sec={formatted_time}, image_width={image_width}, image_height={image_height}\n"
            )


if __name__ == "__main__":
    main()
