#!/bin/bash

# Iterate over possible video devices /dev/video0 to /dev/video4
for video_device in /dev/video{0..4}; do
    # Check if the device exists
    if [ -e "$video_device" ]; then
        # Check if the device supports the YUYV format
        if v4l2-ctl --device=$video_device --list-formats-ext | grep -q "YUYV"; then
            echo "Found YUYV format on $video_device"
            # Run the GStreamer pipeline with the selected device
            gst-launch-1.0 v4l2src device=$video_device ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.41.151
            exit 0
        fi
   fi
done

# If no device with YUYV format is found
echo "No video device with YUYV format found."
exit 1

