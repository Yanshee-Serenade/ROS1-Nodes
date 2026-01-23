FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"

# Create a non-root user with the same UID/GID as the host
ARG USERNAME=dockeruser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create user and group
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

# Build ORB-SLAM3
# COPY orb_slam3_ros ./src/orb_slam3_ros
# RUN catkin build orb_slam3_ros

COPY ubt_msgs ./src/ubt_msgs
RUN catkin build ubt_msgs

RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc
COPY serenade ./src/serenade
COPY ros_entrypoint.sh /ros_entrypoint.sh
COPY bridge.yaml /bridge.yaml
COPY start_bridge.launch /start_bridge.launch
COPY init.sh /init.sh
