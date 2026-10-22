FROM docker.io/osrf/ros:humble-desktop-full@sha256:1db1e4e941d4f77fab55bcd479158a75273329335e86ec468bff280859e8178c
WORKDIR /ws

RUN apt-get update
RUN apt-get upgrade -y
RUN rosdep update

# install project dependencies
RUN --mount=dst=/ws \
  rosdep install --from-path --ignore-src -y src

CMD ["sleep", "inf"]
