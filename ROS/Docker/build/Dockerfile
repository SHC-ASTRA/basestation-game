FROM ros:humble-ros-core
WORKDIR /temp

RUN apt update && \
    apt install -y \
    software-properties-common \
    ros-dev-tools \
    ros-humble-rosbridge-server

ENV MYUSER=astra
ENV MYUID=1000
ENV MYGID=1000

RUN addgroup --gid ${MYGID} ${MYUSER}


RUN adduser --gecos '' --disabled-password --uid ${MYUID} --gid ${MYGID} ${MYUSER} && \
    adduser ${MYUSER} adm

COPY astra_msgs /usr/local/astra_msgs
RUN chown -R ${MYUSER} /usr/local/astra_msgs
RUN chmod -R 777 /usr/local/astra_msgs

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

COPY rosbridge_websocket_launch.xml /opt/ros/humble/share/rosbridge_server/launch/rosbridge_websocket_launch.xml

CMD ["/bin/bash", "/usr/local/bin/entrypoint.sh"]

USER ${MYUSER}
