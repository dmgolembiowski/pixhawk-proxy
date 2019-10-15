#!/bin/bash

echo "Build non-UxAS stuff"
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
meson build --buildtype=debug # TODO: change back to release
ninja -C build all
cd ../

echo "Done building stuff"