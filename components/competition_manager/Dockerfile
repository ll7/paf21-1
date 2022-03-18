FROM ros:noetic

# set up the default shell used by RUN tags
SHELL ["/bin/bash", "-c"]

# install git and alias python3 as python
RUN apt-get update && apt-get install -y \
        git python-is-python3 && \
    rm -rf /var/lib/apt/lists/*

# copy / download source code
WORKDIR /app/src/competition_manager
ARG CM_GITHUB_URL=https://github.com/ll7/paf_competition_manager
RUN git clone $CM_GITHUB_URL .

# register a startup procedure to launch the ROS node
ADD ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
