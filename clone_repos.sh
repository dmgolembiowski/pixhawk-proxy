#!/bin/bash

git clone https://github.com/GaloisInc/lmcp_sentinelizer.git
git clone https://github.com/GaloisInc/uxas_attribute_message.git
git clone https://github.com/GaloisInc/mavlink2protobuf_service.git
# git clone https://github.com/GaloisInc/pixhawk-proxy.git <- this already exists

echo "Clone PX4 Firmware"
git clone -b uxas_master https://github.com/GaloisInc/Firmware.git

echo "Download QGroundControl"
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage && chmod +x ./QGroundControl.AppImage

echo "Clone UxAS (Galois fork)"
git clone https://github.com/GaloisInc/OpenUxAS
git clone https://github.com/afrl-rq/LmcpGen.git

echo "Clone and build OpenAMASE"
git clone https://github.com/afrl-rq/OpenAMASE.git


