# ROS2 in Conda
Since this project will be using a lot of python packages, it's a good idea to use conda environment to manage different packages.  

I followed [this tutorial](https://robostack.github.io/GettingStarted.html) to install ros2 dependencies in a conda environment, and there's several tips.

  - After finishing installing everything in the ros_env, rememer **deactivate** first then activte it again even you are using a Linux machine. In the tutorial it only reminds to do so "on Windows". Then you can do rosdep init and update.
  - If you have a pre-built workspace, you will then need to build everything again. For c++ 
  - Need to install spdlog