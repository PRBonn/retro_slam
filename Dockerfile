FROM osrf/ros:humble-desktop
LABEL maintainer="Saurabh Gupta <sgupta2@uni-bonn.de>"

ENV USER_NAME="user"
ENV GROUP_ID=1000
ENV USER_ID=1000
ENV ROS_DISTRO humble

# Use local mirrors for faster deployment
RUN sed -i -e 's/http:\/\/archive\.ubuntu\.com\/ubuntu\//mirror:\/\/mirrors\.ubuntu\.com\/mirrors\.txt/' /etc/apt/sources.list

# Install bare-minumin tools to bootstrap the installation
RUN apt update && apt upgrade -y && apt install -y \
    curl \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Add normal sudo-user to container, passwordless
RUN addgroup --gid $GROUP_ID $USER_NAME \
&& adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID $USER_NAME \
&& adduser $USER_NAME sudo \
&& echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
&& sed -i 's/required/sufficient/' /etc/pam.d/chsh \
&& touch /home/$USER_NAME/.sudo_as_admin_successful

# Add ROS apt repository
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && apt install -y /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y \
    wget \
    gpg \
    && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc \
       2>/dev/null | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main" \
       > /etc/apt/sources.list.d/kitware.list \
    && apt-get update \
    && apt-get install -y cmake \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-tf2-eigen \
    iproute2 \
    ccache \
    fontconfig \
    locales \
    python3-pip \
    curl \
    rsync \
    git \
    tmuxinator \
    && rm -rf /var/lib/apt/lists/*


# Switch to local user
USER $USER_NAME

# Some custom setup
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/benemer/dotfiles/main/tools/install.sh)" "" --full --skip-decrypt
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.zsh 2>/dev/null || true' > /home/$USER_NAME/.zshrc_local
RUN echo 'source /home/$USER_NAME/ros_ws/install/setup.zsh 2>/dev/null || true' >> /home/$USER_NAME/.zshrc_local
RUN echo 'export HISTFILE=/home/$USER_NAME/ros_ws/.zsh_history' >> /home/$USER_NAME/.zshrc_local

# Zenoh
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add ROS entrypoint for zsh
WORKDIR /home/$USER_NAME/ros_ws
COPY --chmod=755 ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
