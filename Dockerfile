FROM docker.io/osrf/ros:jazzy-desktop-full@sha256:ae7ad3ac243da1dfd8bd402a2fa08e149ffbf7a385013bd8ef798d0800accdc8
WORKDIR /ws

RUN apt-get update
RUN apt-get upgrade -y
RUN rosdep update

# install project dependencies
RUN --mount=dst=/ws \
  rosdep install --from-path --ignore-src -y src

CMD ["sleep", "inf"]
