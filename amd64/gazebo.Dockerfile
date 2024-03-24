FROM gazebo:latest

ARG USER_ID=1000
ARG GROUP_ID=1000

ENV USERNAME gazebo
RUN adduser --disabled-password --gecos '' $USERNAME && \
    usermod  --uid ${USER_ID} $USERNAME && \
    groupmod --gid ${GROUP_ID} $USERNAME && \
    usermod --shell /bin/bash $USERNAME && \
    adduser $USERNAME sudo && \
    adduser $USERNAME dialout && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER $USERNAME

RUN mkdir -p /home/$USERNAME
WORKDIR /home/$USERNAME