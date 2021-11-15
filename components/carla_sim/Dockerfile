# snippet source: https://git.rz.uni-augsburg.de/luttkule/psaf-utils/-/blob/master/docker/carla-build/Dockerfile
# this fixes the 'xdg-user-dir not found' error message, preventing the official CARLA image from crashing

ARG CARLA_VERSION
FROM carlasim/carla:$CARLA_VERSION
USER root
RUN apt-get update && apt-get install -y xdg-user-dirs xdg-utils && apt-get clean
USER carla

ENTRYPOINT ["/bin/bash", "./CarlaUE4.sh"]
CMD ["-carla-server", "-opengl"]
