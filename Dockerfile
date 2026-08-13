FROM docker.io/osrf/ros:humble-desktop-full@sha256:3178f4192a0220b20cad468e4b4c91c5d7c3d8c77cfbea85358d97db5af469c6
WORKDIR /ws

RUN apt-get update && apt-get upgrade -y
RUN rosdep update

# install project dependencies
COPY --parents src/*/package.xml .
COPY --parents src/*/*/package.xml .
RUN rosdep install --from-path --ignore-src -y /ws/src

CMD ["sleep", "inf"]
