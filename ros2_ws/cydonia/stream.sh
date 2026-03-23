#!/bin/bash

gst-launch-1.0 \
  rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! video/x-raw,format=I420 ! \
  x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! \
  rtph264pay pt=96 ! udpsink host=145.126.41.151 port=4500
