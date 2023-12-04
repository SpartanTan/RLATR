import gymnasium
import matplotlib.pyplot as plt
import argparse
import subprocess
import os
import time
from importlib.metadata import distribution

def strtobool(val):
    """
    Convert a string representation of truth to true (1) or false (0).
    
    True values are 'y', 'yes', 't', 'true', 'on', and '1'.
    False values are 'n', 'no', 'f', 'false', 'off', and '0'.
    Raises ValueError if 'val' is anything else.
    """
    val = val.lower()
    if val in ('y', 'yes', 't', 'true', 'on', '1'):
        return 1
    elif val in ('n', 'no', 'f', 'false', 'off', '0'):
        return 0
    else:
        raise ValueError("Invalid truth value %r" % (val,))

def is_package_installed(package_name):
    # Check if package is installed
    try:
        distribution(package_name)
        return True
    except:
        return False
    
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--install', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle')
    args = parser.parse_args()   
    return args

if __name__ == "__main__":
    args = parse_args()
    package_name = "FactoryEnv"
    if is_package_installed(package_name):
        print(f"{package_name} is already installed.")
    else:
        print(f"{package_name} is not installed. Installing now...")
        if args.install:
            print("You've selected to install the package")
            try:
                current_directory = os.getcwd()
                install_directory = os.path.join(current_directory, "../..")
                os.chdir(install_directory)
                subprocess.check_call(["pip", "install", "-e", package_name])
                print(f"{package_name} has been installed successfully.")
            except subprocess.CalledProcessError:
                print(f"Failed to install {package_name}.")
        else:
            print("You've selected not to install the package. Check the box if you want to do so.") 
    
    from factory_env.envs.parameters import env_param
    params = env_param()
    env = gymnasium.make("training-factory-v0", params=params, render_mode="rgb_array")
    print("Environment is installed. You shall see the plot.")
    env.reset()
    env.render()
    plt.show()