#!/bin/bash

# Install a few random packages.
# curl              used below to pull install scripts from web
# build-essential   Installs make, g++, gcc
# clang-format      Formats generated protobuf files
# meson             Creates build scripts for ninja
# ninja             Builds UxAS
# ant               Builds java Apps
# rustc             Rust support for a couple repos.
sudo apt install -y curl
sudo apt install -y build-essential
sudo apt install -y clang-format
sudo apt install -y meson
sudo apt install -y ninja
sudo apt install -y ant

# Only run this one once. It will warn you before running twice that second runs
# may mess it up.
curl -sSL http://get.gazebosim.org | sh

# Install rustup with will provide rust and utilities.
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs > install_rust.sh
bash install_rust.sh --default-toolchain stable --profile default -y
rm install_rust.sh

# rustc, rustup & cargo are in ~/.profile but we'd need to log in again to get it sourced.
# So, just manually source profile. Note: this will be required for every terminal if you want
# to use rust until you've logged out and in.
source ~/.profile

# Must be nightly because developer used features only availible in the nightly branch
# NOTE: Nightly features either make it into the main rust branch or they are dropped.
#       If the rust programs can't compile at a later date it may be that the features
#       he used were dropped rather than integrated into the stable Rust release.
rustup install nightly
rustup default nightly
rustup update nightly

cd .. # Don't put line in README.md - break instructions here.


#bash pixhawk-proxy/build_stuff.sh
