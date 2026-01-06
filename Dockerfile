FROM osrf/ros:humble-desktop-full
WORKDIR /ws

RUN apt-get update && apt-get upgrade -y

# install project dependencies
COPY --parents src/*/package.xml .
RUN rosdep install --from-path --ignore-src -y /ws/src

CMD ["sleep", "inf"]
