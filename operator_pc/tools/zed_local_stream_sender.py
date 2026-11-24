#!/usr/bin/env python3
"""Stand-in for the Vision PC `zed_stream`: stream a USB ZED camera from this PC.

Lets the whole Operator PC image chain (run_zed_sdk_ros_bridge_local, HMD overlay, intent recogniser)
be developed with a local ZED Mini exactly as with the robot head camera; point the receivers at
127.0.0.1:<port> via RHPS1_ZED_STREAM_ENDPOINT.

    python3 operator_pc/tools/zed_local_stream_sender.py            # HD1080 @ 15 fps, port 30000
"""
import argparse
import signal
import sys

import pyzed.sl as sl

RESOLUTIONS = {"HD2K": sl.RESOLUTION.HD2K, "HD1080": sl.RESOLUTION.HD1080,
               "HD720": sl.RESOLUTION.HD720, "VGA": sl.RESOLUTION.VGA}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=30000, help="even port number (ZED SDK requirement)")
    parser.add_argument("--resolution", choices=sorted(RESOLUTIONS), default="HD1080")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--bitrate", type=int, default=8000, help="[kbit/s]")
    args = parser.parse_args()

    init = sl.InitParameters()
    init.camera_resolution = RESOLUTIONS[args.resolution]
    init.camera_fps = args.fps
    init.depth_mode = sl.DEPTH_MODE.NONE   # depth is computed by the receiver, as with the Vision PC
    cam = sl.Camera()
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"cannot open the USB ZED camera: {status}", file=sys.stderr)
        return 1
    stream = sl.StreamingParameters()
    stream.codec = sl.STREAMING_CODEC.H264
    stream.port = args.port
    stream.bitrate = args.bitrate
    status = cam.enable_streaming(stream)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"cannot enable streaming: {status}", file=sys.stderr)
        cam.close()
        return 1
    info = cam.get_camera_information()
    print(f"Streaming {info.camera_model} S/N {info.serial_number} {args.resolution}@{args.fps} on port {args.port}",
          flush=True)

    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rt = sl.RuntimeParameters()
    frames = 0
    while running:
        if cam.grab(rt) == sl.ERROR_CODE.SUCCESS:
            frames += 1
            if frames % 300 == 0:
                print(f"streamed {frames} frames", flush=True)
    cam.disable_streaming()
    cam.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
