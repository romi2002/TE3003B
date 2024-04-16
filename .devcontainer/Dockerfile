FROM ros:noetic
RUN apt-get update && \
	apt-get install -y curl ros-noetic-desktop wget ros-noetic-foxglove-bridge ros-noetic-rqt-multiplot && \
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
