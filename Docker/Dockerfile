FROM arm64v8/ros:humble-perception
SHELL ["/bin/bash", "-c"]

ARG USER_ID=1000
ARG GROUP_ID=1000
ENV USERNAME turtle
RUN adduser --disabled-password --gecos '' $USERNAME && \
    usermod  --uid ${USER_ID} $USERNAME && \
    groupmod --gid ${GROUP_ID} $USERNAME && \
    usermod --shell /bin/bash $USERNAME && \
    adduser $USERNAME sudo && \
    adduser $USERNAME dialout && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER $USERNAME

RUN sudo apt-get update && \
    rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    sudo apt-get upgrade -y && \
    sudo rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source="./utils/install.sh",target="/tmp/install.sh" bash /tmp/install.sh
RUN --mount=type=bind,source="./utils/build.sh",target="/tmp/build.sh" source /opt/ros/${ROS_DISTRO}/setup.bash && bash /tmp/build.sh

WORKDIR /home/$USERNAME
