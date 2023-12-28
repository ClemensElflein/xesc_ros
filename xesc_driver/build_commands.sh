#!/bin/bash
/usr/bin/cmake /home/clemens/Dev/rosws/src/xesc_ros/xesc_driver -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_INSTALL_PREFIX=/home/clemens/Dev/rosws/install/xesc_driver
/usr/bin/cmake --build /home/clemens/Dev/rosws/build/xesc_driver -- -j12 -l12
/usr/bin/cmake --install /home/clemens/Dev/rosws/build/xesc_driver
