#!/bin/bash

set -x

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu precise main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -

sudo add-apt-repository -y ppa:hrg/daily
sudo apt-get update -qq
sudo apt-get install -qq -y hrpsys-base libsdl1.2-dev drcsim-hydro libomniorb4-dev python-pip graphviz

sudo pip install Sphinx sphinx-intl sphinx-rdt-theme breathe

source /opt/ros/hydro/setup.bash
source /usr/share/drcsim/setup.sh

cmake .
make

cd doc
make html


