# syntax=docker/dockerfile:1.7-labs
# needed until COPY --parents is available in stable

FROM osrf/ros:kilted-desktop-full
WORKDIR /ws

COPY --parents src/*/package.xml .
RUN apt update && rosdep install --from-path -iy /ws/src && apt clean && rm -rf /var/lib/apt/lists/*

CMD ["sleep", "inf"]
