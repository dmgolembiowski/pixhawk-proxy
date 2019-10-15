#!/bin/bash

# Setup gazebo repo connection - http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Setup java repo connection
sudo add-apt-repository ppa:webupd8team/java

# Update apt after adding new repos.
sudo apt update

# installs packages             will prompt for sudo password
ansible-playbook configure.yml --ask-become-pass

bash install_rust.sh

cd ..
bash clone_repos.sh

bash build_stuff.sh
