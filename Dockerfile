FROM public.ecr.aws/artefacts/nav2:humble-fortress

WORKDIR /ws

COPY deps.repos /tmp/deps.repos
RUN apt update && apt install -y python3-vcstool && rm -rf /var/lib/apt/lists/*
RUN mkdir -p src && vcs import --input /tmp/deps.repos src
RUN apt update -y && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install

COPY src /ws/src
COPY README.md /ws/README.md
COPY how_to_run /ws/how_to_run
RUN apt update -y && apt install -y ros-humble-rosbag2-storage-mcap && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install
RUN pip install -r /ws/src/sam_bot_nav2_gz/requirements.txt

CMD ["bash"]
