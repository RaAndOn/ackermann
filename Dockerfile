FROM ros:melodic-ros-base

ENV HOME=/root

RUN apt -y update && apt -y upgrade

RUN apt install -y git \
                   curl \
                   wget \
                   bash-completion \
                   openssh-client
                   clang-tools \
                   clang-tidy

# Add git tools
RUN curl https://raw.githubusercontent.com/git/git/master/contrib/completion/git-completion.bash -o ~/.git-completion.bash && \
   echo source ~/.git-completion.bash >> ~/.bashrc

# Install Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > \
    /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt update && apt install -y gazebo9 ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros-control

# Install ROS Packages
RUN apt install -y python3-catkin-tools \
                   ros-$ROS_DISTRO-tf2-geometry-msgs \
                   ros-$ROS_DISTRO-xacro \
                   ros-$ROS_DISTRO-rviz \
                   ros-$ROS_DISTRO-robot-state-publisher

WORKDIR $HOME/

RUN git config --system user.name "Joshua Ra'anan" && \
    git config --system user.email "joshua.raanan@gmail.com" && \
    # This is unsafe outside a docker  container
    git config --system --add safe.directory /root

CMD ["tail", "-f", "/dev/null"]
