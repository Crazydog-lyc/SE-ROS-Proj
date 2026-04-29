FROM public.ecr.aws/artefacts/nav2:humble-fortress

WORKDIR /ws

COPY src/course_interfaces /ws/src/course_interfaces
COPY src/nav2_mission_manager /ws/src/nav2_mission_manager

RUN apt update -y && rosdep install --from-paths src --ignore-src -r -y && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

CMD ["/bin/bash", "-lc", "source /ws/install/setup.bash && colcon test --event-handlers console_direct+ && colcon test-result --verbose"]
