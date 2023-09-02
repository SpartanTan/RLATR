# ProjectName

This repository contains the material of the master thesis or Design Project "Reinforcement learning for robust mobile robot navigation control"

## Background

The ATRs work on a dynamic and stochastic factory floor, which requires the agent to be able to react to potential collisions e.g. with a walking passenger or a toolbox on its way. Moreover, the end time of delivery should also be strictly controlled otherwise the
whole production line might pause due to a lack of spares at some stages.

Another big challenge comes from the sensors equipped by ATRs. Robots rely heavily on sensors e.g. IMU, odometer or scanners. The guidance and control system might easily fail when one or more sensors break down or simply provide low-accurate measurements.

Students:

- Zhicun Tan zhicun@chalmers.se

Advisors:

- Emmanuel Dean deane@chalmers.se
- Karinne Ramirez-Amaro karinne@chalmers.se
- Jens Christian Andersen jcan@dtu.dk

## Problem and Goal

This thesis will propose a robust navigation controller using reinforcement learning meth-ods. The controller should be able to control the ATR to follow the desired trajectory, avoid collisions and keep the delivery deadline. The obstacles could be static or dynamic.  

The learning step will be performed online. 

The experiment will be implemented in a simulation environment that mimics different
scenarios in factories and warehouses. Besides, the performance difference between the
RL-based controller and the MPC-based controller will be evaluated.

The project is in collaboration with Volvo GTO.

## Git repo structure

In general, the solutions generated within this project will be integrated within a larger scenario. Therefore, it is important to maintain the modularity and reusability of the obtained solutions.

In general, this project can produce source code (C++, Python, ROS, Arduino, VHDL), diagrams, designs, and circuits that may be re-used and integrated with other projects. Most probably, you will need to implement additional code, routines, or tests to develop your solutions. For example, if you are training a NN, you will need to pre-process and label the training data. This is material needed to develop your solution but it is not part of the solution. Therefore, it should not be included directly in the resultant material.

For this reason, we propose the following file system for this repo:

```
.
├── Code (Folder that contains all the code generated in this project)
│   └── python_tools (Folder that contain all the python code and additional tools to support your development)
|   └── ros_ws (Folder containing ros workspaces and/or ros packages). NOTE: Do not include binary and log folders, e.g. build/ devel/ log/
├── Diagrams (This folder contains circuit diagrams, e.g., schematics (.sch), boards (.brd), CAD (.stl, .cda), etc. generated and used in this project)
├── Documentation (This folder contains all the documentation generated and used in this project)
│   ├── How2s (Folder with mini-how-tos providing instructions on how to run demos or applications developed within this project)
│   ├── PlanningReport (Folder containing material for the planning report)
│   ├── Resources (Folder containing material from third party authors, e.g. reference papers (PDF), figures, manuals, etc.)
│   ├── Thesis/Report (Folder containing the final report or thesis)
│   └── Workdiary (Folder with a work diary template to report the daily activities)
└── README.md (Main project's readme file, with these general instructions and documentation's links to understand this project.)
```

You can add more folders to improve the file system but the general structure should be preserved. The idea is to provide a homogeneous structure that is preserved through all the connected projects.

## Getting started

### Agile Project Management (APM) with Gitlab

We are planning to use APM to manage the project (<https://kanbanize.com/agile/project-management>).
​

We have prepared a set of videos that explain how to use Github to implement APM. Please, follow the videos.

<https://www.youtube.com/playlist?list=PLObANLLE5pkmDIPhaBsUbmjp01qI8KxvU>

### Ubuntu tutorials

The Linux command line for beginners: <https://ubuntu.com/tutorials/command-line-for-beginners#1-overview>

### Ros version: ROS Noetic

Ubuntu install of ROS Noetic: <http://wiki.ros.org/noetic/Installation/Ubuntu>

### Ros tutorials

<http://wiki.ros.org/ROS/Tutorials> (general ros tutorials)

<https://gitlab.com/deanch/ros_course_template> (Dean's tutorial, more specific)

## Code Standards and Formats

### ROS style and naming conventions

ROS provides a comprehensive guide on naming conventions and coding standards.

You can find the guidelines for python here:

<http://wiki.ros.org/PyStyleGuide>

And for C++, here

<http://wiki.ros.org/CppStyleGuide>

The guidelines for naming convention and units can be found here:

<http://wiki.ros.org/ROS/Patterns/Conventions>

Finally, here are some instructions on how to format C++ code using ROS format

<https://github.com/PickNikRobotics/roscpp_code_format>

### Python

We would appreciate if you write the code in this repo in accordance with the following standards:

- black formatting <https://black.readthedocs.io/en/stable/>
- pylint <https://pylint.org/>
- pydocstyle <http://www.pydocstyle.org/en/stable/>

Please use the standard settings for these packages.
They are easily installed with pip (preferable on a conda virtual environment):

- pip install black
- pip install pylint
- pip install pydocstyle.

Once installed, you can check your python scripts (e.g. test.py) through

- black test.py (add flags --check --diff to check the file without making any changes)
- pylint test.py
- pydocstyle test.py

These commands also work on entire folders.
However, please get familiar with these tools before reformatting large amounts of code!

Furthermore, these tools work well in VSCode (surely in other IDEs as well).

#### How to activate black formatting in VSCode

<https://dev.to/adamlombard/how-to-use-the-black-python-code-formatter-in-vscode-3lo0>

#### How to activate linting (e.g. pylint) in VSCode

<https://code.visualstudio.com/docs/python/linting>

Make sure to also visit settings (through ctrl+,) and search for "python.linting.pylintArgs" and add "--enable=F,E,W" as an item.
Otherwise any pylint warnings will be suppressed.

#### How to activate pydocstyle in VSCode

Open settings with ctrl+, and search for "python.linting.pydocstyleEnabled". Enable for both user and workspace.
