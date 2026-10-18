#!/usr/bin/env python3
"""Record the 3D demo cameras (video_cams.CAMERAS) during a Gazebo run. Runs inside the `gazebo` container:

    docker compose exec -T gazebo /entrypoint.sh python3 /ws/tools/metrics/record_cams.py /ws/runs/<id>/video [cams]

Spawns one static camera model per camera into the running world and reads the images straight from gz-transport
(no ROS bridge: 1080p frames through DDS would cost more CPU than rendering them). Writes per camera ``<cam>.avi``
(MJPEG, quality 95: an intermediate for make_demo_video.py) and ``<cam>.csv`` (frame index, sim time). Cameras render
in SIM time, so a capped real-time factor slows the recording, not the video. Stops on SIGTERM/SIGINT.
"""
import csv
import os
import queue
import signal
import sys
import threading
import time

import cv2
import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from video_cams import CAMERAS, camera_sdf  # noqa: E402

WORLD = os.environ.get("SWARMFLOW_LAYOUT", "warehouse_v2")


def main():
    out_dir = sys.argv[1]
    cams = sys.argv[2].split(",") if len(sys.argv) > 2 else list(CAMERAS)
    os.makedirs(out_dir, exist_ok=True)
    node = Node()
    frames = queue.Queue(maxsize=40)      # transport callbacks only enqueue; one thread encodes
    writers, logs, counts = {}, {}, {}
    for c in cams:
        cam = CAMERAS[c]
        w = cv2.VideoWriter(os.path.join(out_dir, f"{c}.avi"), cv2.VideoWriter_fourcc(*"MJPG"), cam.rate_hz,
                            (cam.width, cam.height))
        w.set(cv2.VIDEOWRITER_PROP_QUALITY, 95)
        writers[c], counts[c] = w, 0
        f = open(os.path.join(out_dir, f"{c}.csv"), "w", newline="")
        logs[c] = (f, csv.writer(f))
        logs[c][1].writerow(["frame", "t"])
        if not node.subscribe(Image, f"/video/{c}", lambda m, c=c: frames.put((c, m))):
            sys.exit(f"subscribe /video/{c} failed")
        ok, rep = node.request(f"/world/{WORLD}/create", EntityFactory(sdf=camera_sdf(cam, f"/video/{c}")),
                               EntityFactory, Boolean, 5000)
        print(f"spawned {c}: {ok and rep.data}", flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    last_report = time.time()
    while not (stop.is_set() and frames.empty()):
        try:
            c, m = frames.get(timeout=0.2)
        except queue.Empty:
            continue
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * 3]
        writers[c].write(cv2.cvtColor(img.reshape(m.height, m.width, 3), cv2.COLOR_RGB2BGR))
        logs[c][1].writerow([counts[c], f"{m.header.stamp.sec + m.header.stamp.nsec * 1e-9:.3f}"])
        counts[c] += 1
        if time.time() - last_report > 30:
            last_report = time.time()
            print(" ".join(f"{k}:{v}" for k, v in counts.items()), f"queue:{frames.qsize()}", flush=True)
    for c in cams:
        writers[c].release()
        logs[c][0].close()
    print("done", " ".join(f"{k}:{v}" for k, v in counts.items()), flush=True)


if __name__ == "__main__":
    main()
