FROM ros:noetic-ros-base

ENV HOME=/root

RUN apt -y update && apt -y upgrade

RUN apt install -y git \
                   curl \
                   wget \
                   bash-completion \
                   openssh-client

# Add git tools
RUN curl https://raw.githubusercontent.com/git/git/master/contrib/completion/git-completion.bash -o ~/.git-completion.bash && \
   echo source ~/.git-completion.bash >> ~/.bashrc

# Install Catkin Tools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
    > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt update && apt-get install -y python3-catkin-tools

# Install Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > \
    /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt update && apt install -y gazebo11 ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Install ROS Packages
RUN apt install -y ros-noetic-tf2-geometry-msgs

WORKDIR $HOME/

RUN git config --system user.name "Joshua Ra'anan" && \
    git config --system user.email "joshua.raanan@gmail.com" && \
    # This is unsafe outside a docker  container
    git config --system --add safe.directory /root

CMD ["tail", "-f", "/dev/null"]
