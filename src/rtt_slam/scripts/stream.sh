#!/bin/bash
# Stream the RealSense colour image as H.264 over RTP/UDP.
# usage: stream.sh [host] [port]   (defaults to the previous fixed target)

HOST=${1:-145.126.41.72}
PORT=${2:-4500}

gst-launch-1.0 \
  rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! video/x-raw,format=I420 ! \
  x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! \
  rtph264pay pt=96 ! udpsink host="$HOST" port="$PORT"
