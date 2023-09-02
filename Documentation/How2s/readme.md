This folder contains how-tos with information on how to run applications, scripts, and demos produced in this project. The how-tos also include instructions to install the requirements to run the demos.

Please notice that the project also has a Wiki pages. Make sure that this information is also available in those wiki-pages.








 make sure you are now at /atr_rl
```
cd Code/ros_ws
colcon build  --symlink-install --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```