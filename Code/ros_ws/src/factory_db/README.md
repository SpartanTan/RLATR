# factory_db (Vanilla)

This git repo provides general configuration files and parameters of the factory needed by different ros2 packages and nodes. It provides a common access point to centralize configurations and parameters.

## Description

There are some factory parameters that will be shared among multiple ros packages (nodes). These parameters are related to common information of the factory. Examples are the number of ATR available in the factory, their corresponding ATR_IDs, the number of cameras, their intrinsic and extrinsic parameters, etc.

One specific use case for this package is the communication between the **atr_demo** and the **atr_factory_state** nodes. Both nodes require a list of available ATRs with their IDs. **atr_demo** spawns the ATRs and **atr_factory_sate** subscribes to the topics of each ATR. Then, both nodes require exactly the same information. The package **factory_db** provides this information for both nodes. If we need to change this information, then we just need to modify a single file in **factory_db**.

### Input

None

### Output

Different configuration files organized in folders. The structure of the package is:

```bash
├── factory_db
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── atr         ---> configuration files related to the ATRs
│   │   └── perception  ---> configuration files related to perception
│   ├── launch
│   │   └── README.md
│   ├── package.xml
│   └── README.md
```

### Common methods

For the moment, this package only contains configuration files

## How to use

The idea is to add your global configuration files in a subfolder inside config folder. Then, load and parse the configuration files in the launch files of the different ros nodes that require the parameters defined in that configuration file.

In general, we are using yaml files as configuration files (standard format for ROS). Then, we need to use a YAML parser.

To use the yaml parser in python, you need to install the following package:

```bash
pip install PyYAML
```

Once the yaml parser is installed, you can use it in your launch file (python script). The following snippet shows a typical way to use this parser.

```python
from launch import LaunchDescription
import yaml
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  
  # Create the launch description and populate it
  ld = LaunchDescription()
  
  # find the global path to the factory_db package
  factory_db_dir = FindPackageShare("factory_db").find("factory_db")

  # generate the global path to the target configuration file (yaml file)
  atr_param_yaml = os.path.join(factory_db_dir, "config/atr", "atr.param.yaml")

  # parse the yaml file
  with open(atr_param_yaml, "r") as stream:
        try:
            config_parameters = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

  # Get the value of the parameter atr_id_list from the yaml file
  atr_id_list = config_parameters["atr_params"]["ros__parameters"]["atr_id_list"]

  # run a node from my_awesome_pkg ros package using the global parameter atr_id_list
  my_awesome_pkg_dir = FindPackageShare("my_awesome_pkg").find("my_awesome_pkg")

  my_awesome_pkg = Node(
      package="my_awesome_pkg",
      name="my_awesome_pkg",
      executable="my_awesome_pkg_node",
      parameters=[{"atr_id_list": atr_id_list}]
  )
  ld.add_action(my_awesome_pkg)

  return ld
```

In this snippet, we are using this configuration file: (<https://gitlab.com/volvo_gto/gpss_mvp/shared/factory_db/-/blob/vanilla/config/atr/atr.param.yaml>) as an example.

## TODO

For the moment, this package only provides a common access point for other ros packages to get configuration files with shared information.

The next step is to develop a DB server which provides these parameters dynamically and on-demand. Then, the nodes requiring global parameters can request that information from the DB server in run-time.
