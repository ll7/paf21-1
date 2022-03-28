# snippet source: https://git.rz.uni-augsburg.de/luttkule/psaf-utils/-/blob/master/docker/carla-build/Dockerfile
# this fixes the 'xdg-user-dir not found' error message, preventing the official CARLA image from crashing

ARG CARLA_VERSION=0.9.10.1
FROM carlasim/carla:$CARLA_VERSION

USER root
RUN apt-get update && apt-get install -y wget xdg-user-dirs xdg-utils && apt-get clean
WORKDIR /home/carla/Import
RUN  wget -q https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.10.tar.gz
WORKDIR /home/carla
RUN ./ImportAssets.sh
USER carla

ENTRYPOINT ["/bin/bash", "./CarlaUE4.sh"]
CMD ["-carla-server"]
