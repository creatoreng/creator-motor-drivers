#!/bin/bash
#
# Install build dependencies into an ubuntu system

set -e

# apt update
sudo apt-get update -q

# apt update and other common tools for installing packages
sudo apt-get install --no-install-recommends --upgrade -y -q \
		apt-utils \
		software-properties-common \
		curl \
		wget \
		unzip

# install gcc and common build tools
sudo apt-get install --no-install-recommends --upgrade -y -q \
		make \
		automake \
		autoconf \
		libtool \
		pkg-config \
		gcc \
		g++ \
		git \
		gpg

# install CMake
## get the kitware gpg key
wget -nv --show-progress --progress=bar:force:noscroll --tries=2 -O - https://apt.kitware.com/keys/kitware-archive-latest.asc \
	| gpg --dearmor - \
	| sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
## add kitware apt repository
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
## apt update and install
sudo apt-get update -q
sudo apt-get install --no-install-recommends --upgrade -y -q cmake

# install boost
sudo apt-get install --no-install-recommends -y -q libboost-all-dev

exit $?
