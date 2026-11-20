FROM docker.io/osrf/ros:jazzy-desktop-full
WORKDIR /ws

RUN apt-get update
RUN apt-get upgrade -y
RUN rosdep update

# install project dependencies
RUN --mount=dst=/ws \
  rosdep install --from-path --ignore-src -y src

CMD ["sleep", "inf"]
