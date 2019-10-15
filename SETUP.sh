#!/bin/bash

# Install a few random packages.
# curl              used below to pull install scripts from web
# build-essential   Installs make, g++, gcc
# clang-format      Used by a later install script
# meson             Creates build scripts for ninja
# ninja             Builds UxAS
# ant               Builds java Apps
# rustc             Rust support for a couple repos.
sudo apt install -y curl build-essential clang-format meson ninja ant rustc

# Only run this one once. It will warn you before running twice that second runs
# may mess it up.
curl -sSL http://get.gazebosim.org | sh

# Must be nightly because developer used features only availible in the nightly branch
# NOTE: Nightly features either make it into the main rust branch or they are dropped.
#       If the rust programs can't compile at a later date it may be that the features
#       he used were dropped rather than integrated into the stable Rust release.
rustup install nightly
rustup default nightly
rustup update nightly

cd ..
#bash pixhawk-proxy/clone_repos.sh
#
#bash pixhawk-proxy/build_stuff.sh
