from ultralytics import YOLO
from collections import defaultdict
import cv2
import argparse
import numpy as np
import socket
import threading
import struct
import time
import os


class UDPStreamer:
    BUFFER_SIZE = 1024

    def __init__(self, port):
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__sock.bind(('0.0.0.0', port))
        self.__client = None

        self.__thread = threading.Thread(target=self.__listen)
        self.__running = True
        self.__thread.start()

    def __listen(self):
        while self.__running:
            _, client = self.__sock.recvfrom(1024)
            if self.__client != client:
                self.__client = client
                print('Client connected')

    def send(self, data):
        if self.__client is None:
            return
        self.__sock.sendto(data, self.__client)

    def send_yolo_detections(self, results, timestamp_sec, timestamp_nsec):
        boxes = results[0].boxes.xywh.cpu()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        classes = results[0].boxes.cls.int().cpu().tolist()
        confs = results[0].boxes.conf.cpu().tolist()

        message = struct.pack('>H', len(boxes))

        for box, track_id, class_, conf in zip(boxes, track_ids, classes, confs):
            x, y, w, h = box
            class_ = int(class_)
            instance = int(class_ * 1e6) + int(track_id)
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            width = int(w)
            height = int(h)
            conf = int(conf * 100)
            message += struct.pack(
                '>HIHHHHBqq',
                class_,
                instance,
                center_x,
                center_y,
                width,
                height,
                conf,
                timestamp_sec,
                timestamp_nsec,
            )

            print(
                f'Detected {results[0].names[class_]}[{instance}] at ({center_x}, {center_y})'
            )

        self.send(message)

    def close(self):
        self.__running = False
        self.__thread.join()
        self.__sock.close()


def get_annotated_frame(results, track_history):
    boxes = results[0].boxes.xywh.cpu()
    track_ids = results[0].boxes.id.int().cpu().tolist()
    annotated_frame = results[0].plot()

    for box, track_id in zip(boxes, track_ids):
        x, y, w, h = box
        track = track_history[track_id]
        track.append((float(x), float(y)))
        if len(track) > 30:
            track.pop(0)

        points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(
            annotated_frame,
            [points],
            isClosed=False,
            color=(230, 230, 230),
            thickness=10,
        )

    return annotated_frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--model', type=str, default='yolov8n.pt', help='Path to a model'
    )
    parser.add_argument('--source', type=int, default=0, help='Camera index')
    parser.add_argument(
        '--show-detections', type=bool, default=False, help='Show detections'
    )
    parser.add_argument('--port', type=int, default=5000, help='UDP port')
    parser.add_argument('--verbose', type=bool, default=False, help='Verbose')
    args = parser.parse_args()

    track_history = defaultdict(lambda: [])

    model = YOLO(args.model)

    streamer = UDPStreamer(args.port)

    capture = cv2.VideoCapture(args.source)
    if not capture.isOpened():
        print('Unable to open camera')
        exit(1)

    while True:
        timestamp = time.time()
        timestamp_sec = int(time.time())
        timestamp_nsec = int((timestamp - timestamp_sec) * 1e9)

        success, image = capture.read()
        if not success:
            break

        results = model.track(
            image,
            persist=True,
            verbose=args.verbose,
            tracker=os.path.join(
                os.path.dirname(os.path.realpath(__file__)), 'tracker.yaml'
            ),
        )
        if results[0].boxes.id is None:
            continue

        streamer.send_yolo_detections(
            results,
            timestamp_sec=timestamp_sec,
            timestamp_nsec=timestamp_nsec,
        )

        if args.show_detections:
            annotated_frame = get_annotated_frame(results, track_history)
            cv2.imshow('image', annotated_frame)
            if cv2.waitKey(1) == ord('q'):
                break

    capture.release()
    streamer.close()

    if args.show_detections:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
