FROM ros:kinetic-robot

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN pwd

RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py

COPY ./docker/workspace_entry.sh /workspace_entry.sh
RUN chmod +x /workspace_entry.sh
CMD ["/workspace_entry.sh"]