FROM docker.io/osrf/ros:humble-desktop-full@sha256:7323d816878a2c47000cd4e49d49642ffa677c4b1d96c5cf67c27d1434394ba2
WORKDIR /ws

RUN apt-get update && apt-get upgrade -y
RUN rosdep update

# install project dependencies
COPY --parents src/*/package.xml .
COPY --parents src/*/*/package.xml .
RUN rosdep install --from-path --ignore-src -y /ws/src

CMD ["sleep", "inf"]
