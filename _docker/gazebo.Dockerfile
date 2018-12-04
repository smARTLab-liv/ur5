FROM ros:kinetic-robot

RUN apt-get update
RUN apt-get -y install wget

RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -


RUN apt-get update
# RUN apt-get install -y ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
RUN apt-get -y install ros-kinetic-gazebo9-*

COPY ./docker/gazebo_entry.sh /gazebo_entry.sh
RUN chmod +x /gazebo_entry.sh
CMD ["/gazebo_entry.sh"]