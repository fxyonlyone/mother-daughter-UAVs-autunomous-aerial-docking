#! /bin/bash
sudo apt install -y ninja-build exiftool python-argparse python-empy python-toml python-numpy python-yaml python-dev python-pip ninja-build protobuf-compiler libeigen3-dev genromfs xmlstarlet libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libnewmat10-dev

pip2 install pandas jinja2 pyserial cerberus pyulog numpy toml pyquaternion

pip3 install packaging numpy empy toml pyyaml jinja2

sudo apt install ros-melodic-mavros ros-melodic-mavros-extras     # for ros-melodic

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

sudo chmod a+x ./install_geographiclib_datasets.sh

sudo ./install_geographiclib_datasets.sh
