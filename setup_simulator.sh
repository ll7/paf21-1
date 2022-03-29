#!/bin/bash
set -e

if [ ! -d "./simulator" ]; then
    echo 'downloading the CARLA simulator ...'
    CARLA_SIM_URL=https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.1.tar.gz
    mkdir ./simulator
    wget -q -O- $CARLA_SIM_URL | tar xzf - -C ./simulator
    echo 'download successful!'

    echo 'downloading additional CARLA simulator assets (some maps)'
    ADD_ASSETS_URL=https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.10.tar.gz
    cd simulator/Import && wget -q $ADD_ASSETS_URL && cd ..
    echo 'download successful!'
    echo 'importing additional assets ...'
    ./ImportAssets.sh > /dev/null 2>&1 && rm -rf Import/*.tar.gz
    echo 'import successful!'
fi

echo 'simulator is ready for use'
