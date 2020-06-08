#!/bin/bash

git clone https://github.com/RoboticsYY/opencv_deb_install.git

find ./opencv_deb_install -name "OpenCV-3.4.5-x86_64-*.deb" | xargs sudo dpkg -i