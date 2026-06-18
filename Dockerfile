FROM docker.io/osrf/ros:humble-desktop-full@sha256:90b4f334f3c23a1d5e0af55ebe17ee7a6a2b8b9b0a2fda0ced2e85fe9d7c23e1
WORKDIR /ws

RUN apt-get update && apt-get upgrade -y
RUN rosdep update

# install project dependencies
COPY --parents src/*/package.xml .
COPY --parents src/*/*/package.xml .
RUN rosdep install --from-path --ignore-src -y /ws/src

CMD ["sleep", "inf"]
