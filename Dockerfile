FROM docker.io/osrf/ros:humble-desktop-full@sha256:b0302b39a2321079950b6a91a13eafe62e60bd42ffe61c3cea3d5e96886d0a87
WORKDIR /ws

RUN apt-get update && apt-get upgrade -y
RUN rosdep update

# install project dependencies
COPY --parents src/*/package.xml .
COPY --parents src/*/*/package.xml .
RUN rosdep install --from-path --ignore-src -y /ws/src

CMD ["sleep", "inf"]
