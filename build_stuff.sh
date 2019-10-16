#!/bin/bash

echo "Build non-UxAS stuff"
cd LmcpGen && ant jar && cd ..
cd OpenAMASE/OpenAMASE && ant jar && cd ../..

echo "Install UxAS prerequisites"
# Install pkg-config for finding link arguments: in terminal
sudo apt -y install pkg-config
# Install git: in terminal
sudo apt -y install git
sudo apt -y install gitk
# Install opengl development headers: in terminal
sudo apt -y install libglu1-mesa-dev
# Install unique ID creation library: in terminal
sudo apt -y install uuid-dev
# Install Boost libraries (**optional but recommended**; see external dependencies section): in terminal
sudo apt-get install libboost-filesystem-dev libboost-regex-dev libboost-system-dev
# Install pip3: in terminal
sudo apt -y install python3-pip
sudo -H pip3 install --upgrade pip
# Install ninja build system: in terminal
sudo -H pip3 install ninja
# Install meson build configuration: in terminal
sudo -H pip3 install meson==0.42.1
# Install python plotting capabilities (optional): in terminal
sudo apt -y install python3-tk
sudo -H pip3 install matplotlib
sudo -H pip3 install pandas
# Install Oracle JDK
sudo apt -y install default-jre
# Install ant for command line build of java programs
sudo apt -y install ant
echo "Dependencies installed!"

echo "Configuring UxAS plotting utilities ..."
sudo python3 OpenUxAS/src/Utilities/localcoords/setup.py install

echo "Generate lmcp"
bash OpenUxAS/RunLmcpGen.sh

echo "Build OpenUxAS"
cd OpenUxAS
./rm-external
./prepare
meson build --buildtype=release
meson build_debug --buildtype=debug
ninja -C build all

cd ../

echo "Done building stuff"