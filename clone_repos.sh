#!/bin/bash
git submodule update --init --recursive &
wait $!

echo "Download QGroundControl"
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage \
    && chmod +x ./QGroundControl.AppImage

