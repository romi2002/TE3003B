FROM ros:humble
RUN apt-get update && \
	apt-get install -y curl wget ros-humble-foxglove-bridge software-properties-common && \
	rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y cmake build-essential libprotobuf-dev autoconf libtool pkg-config golang-go protobuf-compiler libabsl-dev && \
    git clone --recurse-submodules -b v1.30.2 --depth 1 --shallow-submodules https://github.com/grpc/grpc && \
    cd grpc && mkdir -p cmake/build && cd cmake/build && \
    cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
    -DCMAKE_BUILD_TYPE=Release       \
    -DgRPC_ABSL_PROVIDER=package     \
    -DgRPC_PROTOBUF_PROVIDER=package \
      ../.. && \
    make -j12 && \
    make install && \
    rm -rf /var/lib/apt/lists

RUN apt-get update && \
    apt-get install -y ros-humble-tf2-geometry-msgs && \
    rm -rf /var/lib/apt/lists/*

COPY library-scripts/desktop-lite-debian.sh /tmp/library-scripts/
RUN apt-get update && bash /tmp/library-scripts/desktop-lite-debian.sh
ENV DBUS_SESSION_BUS_ADDRESS="autolaunch:" \
    VNC_RESOLUTION="1440x768x16" \
    VNC_DPI="96" \
    VNC_PORT="5901" \
    NOVNC_PORT="6080" \
    DISPLAY=":1" \
    LANG="en_US.UTF-8" \
    LANGUAGE="en_US.UTF-8"
ENTRYPOINT ["/usr/local/share/desktop-init.sh"]
CMD ["sleep", "infinity"]

#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
#	bash -c "wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -" && \
#	apt-get update && \
#	apt-get install -y gazebo11 libgazebo11-dev && \
#	rm -rf /var/lib/apt/lists/*
