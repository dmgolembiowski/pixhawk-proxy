#!/bin/bash

bash install_rust.sh

ansible-playbook configure.yml

cd ..
bash clone_repos.sh

echo "Build stuff"
cd OpenUxAS && git checkout pixhawk && cd ..
cd LmcpGen && ant jar && cd ..
cd OpenAMASE/OpenAMASE && ant jar && cd ../..

echo "Build UxAS - this is the problematic portion"
echo "    Install prerequisites and generate lmcp"
cd OpenUxAS

bash install_prerequisites.sh

bash RunLmcpGen.sh

echo "    Build OpenUxAS"
./rm-external
./prepare
meson build --buildtype=release
ninja -C build all
cd ../

echo "Done building stuff"

