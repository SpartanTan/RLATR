import sys
import os
import json
import factory_env

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QFileDialog, QLabel, QTextEdit, QHBoxLayout, QCheckBox, QTabWidget, QComboBox, QGridLayout
from PyQt5.QtWidgets import QLineEdit, QGridLayout
from PyQt5.QtGui import QIntValidator, QDoubleValidator
from PyQt5.QtCore import QProcess

from setuptools import find_packages
from datetime import datetime

class InputTextEdit(QTextEdit):
    def __init__(self, parent=None):
        super(InputTextEdit, self).__init__(parent)
        self.process = None  # Initialize an attribute to hold the QProcess instance

    def setProcess(self, process):
        self.process = process  # Use this method to set the QProcess instance when you start the process

    def keyPressEvent(self, event):
        super(InputTextEdit, self).keyPressEvent(event)
        if self.process:  # if a process is running
            # Send the captured key to the process's stdin
            self.process.write(bytes(event.text(), 'utf-8'))

class App(QWidget):

    def __init__(self):
        super().__init__()
        self.title = 'RL runner'
        self.param_inputs = {}
        self.reward_params_input = {}
        self.PPO_arguments_input = {}

        self.factory_env_dir = os.path.dirname(factory_env.__file__)

        self.initUI()
        self.process = None
        self.tensorboard_process = None

    def initUI(self):
        self.setWindowTitle(self.title)
        layout = QVBoxLayout()
        self.tab_widget = QTabWidget()
        
        # Create install tab
        tab1 = self.create_install_tab()
        self.tab_widget.addTab(tab1, "Install")
        
        # Create parameters tab
        tab2 = self.create_parameters_tab()
        self.tab_widget.addTab(tab2, "Parameters")

        # Create test tab
        tab3 = self.create_test_tab()
        self.tab_widget.addTab(tab3, "Test run")

        # Create test tab
        tab4 = self.create_training_tab()
        self.tab_widget.addTab(tab4, "Training")

        # Add print console        
        self.output_display = QTextEdit(self)
        self.output_display.setReadOnly(True)

        # Add stop button
        self.stop_button = QPushButton('Stop script', self)
        self.stop_button.clicked.connect(self.stop_script)

        # Whole layout
        layout.addWidget(self.tab_widget)
        layout.addWidget(self.output_display)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)

        # Toggle parameters layout visibility off at start
        # self.toggle_parameters_visibility()
        self.show()
    
    '''
       ___         _        _ _   _____     _    
      |_ _|_ _  __| |_ __ _| | | |_   _|_ _| |__ 
       | || ' \(_-<  _/ _` | | |   | |/ _` | '_ \
      |___|_||_/__/\__\__,_|_|_|   |_|\__,_|_.__/
                                                 
    '''
    def create_install_tab(self):
        # Create install tab
        tab1 = QWidget()
        tab1_layout = QVBoxLayout()

        # Add info
        tab1_layout.addWidget(QLabel("This tab is for installing and testing the environment.\nCheck the --install box below if you want to install the training environment."))
        # Add args options
        self.boolean_arg_checkbox = QCheckBox(" --install", tab1)
        tab1_layout.addWidget(self.boolean_arg_checkbox)
        # Add button
        button1_is_env_installed = QPushButton('Is environment installed?', tab1)
        button1_is_env_installed.clicked.connect(self.run_script_is_env_installed)
        tab1_layout.addWidget(button1_is_env_installed)

        tab1.setLayout(tab1_layout)
        return tab1
    
    def run_script_is_env_installed(self):
        self.output_display.clear()  # Clear the QTextEdit widget

        args = []
        if self.boolean_arg_checkbox.isChecked():
            args.extend(["--install", "true"])
        else:
            args.extend(["--install", "false"])
        
        # Use QProcess to run the script
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        self.process.start("python3", ["-u", "is_env_registered.py"] + args)

    '''
       ___                         _                _____     _    
      | _ \__ _ _ _ __ _ _ __  ___| |_ ___ _ _ ___ |_   _|_ _| |__ 
      |  _/ _` | '_/ _` | '  \/ -_)  _/ -_) '_(_-<   | |/ _` | '_ \
      |_| \__,_|_| \__,_|_|_|_\___|\__\___|_| /__/   |_|\__,_|_.__/
                                                                   
    '''
    def create_parameters_tab(self):
        tab2 = QWidget()
        tab2_layout = QVBoxLayout()

        # Add info
        tab2_layout.addWidget(QLabel("This tab is for parametesr."))
        
        # Path parameter
        toggle_path_params_button = QPushButton("Path Parameters", self)
        toggle_path_params_button.clicked.connect(lambda: self.toggle_parameters('path_params'))
        tab2_layout.addWidget(toggle_path_params_button)
        self.path_params_layout = self.setup_path_parameter_inputs(tab2)
        tab2_layout.addLayout(self.path_params_layout)

        # ATR parameter
        toggle_atr_params_button = QPushButton("ATR Parameters", self)
        toggle_atr_params_button.clicked.connect(lambda: self.toggle_parameters('atr_params'))
        tab2_layout.addWidget(toggle_atr_params_button)
        self.atr_params_layout = self.setup_atr_parameter_inputs(tab2)
        tab2_layout.addLayout(self.atr_params_layout)

        # env parameter layout
        toggle_env_params_button = QPushButton("Env Parameters", self)
        toggle_env_params_button.clicked.connect(lambda: self.toggle_parameters('env_params'))
        tab2_layout.addWidget(toggle_env_params_button)
        self.env_params_layout = self.setup_env_parameter_inputs(tab2)
        tab2_layout.addLayout(self.env_params_layout)

        # sensor parameter layout
        toggle_sensor_params_button = QPushButton("Sensor Parameters", self)
        toggle_sensor_params_button.clicked.connect(lambda: self.toggle_parameters('sensor_params'))
        tab2_layout.addWidget(toggle_sensor_params_button)
        self.sensor_params_layout = self.setup_sensor_parameter_inputs(tab2)
        tab2_layout.addLayout(self.sensor_params_layout)

        # reward parameter layout
        toggle_reward_params_button = QPushButton("Reward Parameters", self)
        toggle_reward_params_button.clicked.connect(lambda: self.toggle_parameters('reward_params'))
        tab2_layout.addWidget(toggle_reward_params_button)
        self.reward_params_layout = self.setup_reward_parameter_inputs(tab2)
        tab2_layout.addLayout(self.reward_params_layout)

        self.parameter_layouts = {
            'path_params': {
                'layout': self.path_params_layout,
                'visible': True,  # Set initial visibility state here
            },
            'atr_params': {
                'layout': self.atr_params_layout,
                'visible': False,  # Set initial visibility state here
            },
            'env_params': {
                'layout': self.env_params_layout,
                'visible': False,  # Set initial visibility state here
            },
            'sensor_params': {
                'layout': self.sensor_params_layout,
                'visible': False,  # Set initial visibility state here
            },
            'reward_params': {
                'layout': self.reward_params_layout,
                'visible': False,  # Set initial visibility state here
            },
        }

        # Hide the widgets in each layout based on the visibility state set in the dictionary
        for param_type, param_info in self.parameter_layouts.items():
            layout = param_info['layout']
            visible = param_info['visible']
            if not visible:
                self.toggle_parameters_visibility(layout, False)

        tab2.setLayout(tab2_layout)
        return tab2

    def setup_path_parameter_inputs(self, parent):
        # Define tooltips for parameters
        tooltips = {
            'No': 'Number of obstacles',
            'Nw': 'Number of waypoints',
            'Lp': 'Path length',
            'mu_r': 'Friction coefficient',
            'sigma_d': 'Standard deviation of the distance between the obstacle and the path',
            'shift_distance': 'Distance between the path and the walls',
            'extend_length': 'How much should the wall extended at the two ends of the path',
            'look_ahead_distance': 'How far should the robot look ahead',
            'target_finishing_time': 'How long should the robot finish the path [s], used for setting ghost trajectory',
            'without_walls': 'Generate a path without walls',
            "allowed_jump_index": "number of points the robot can jump on the path",
            'how_many_points_forward': 'How many points should be considered for the path following reward',
            'max_ep_steps': 'Maximum number of steps in an episode',
            'obs_mean_vel': 'Mean velocity of the obstacles',
            'obs_std_vel': 'Standard deviation of the velocity of the obstacles',
            'flip_chance': 'chance of flipping the velocity of the obstacles',
            'dynamic_obstacles_r': 'Whether to use dynamic raidus for the obstacles',
            'dynamic_obstacles': 'Whether the obstacles are dynamic or not',
            'static_map': '0: run, 1: save, 2: load',
            "consider_width": "0: consider width, 1: not consider width",
        }

        # Setup parameter inputs and grid layout
        validators = {
            'No': QIntValidator(1, 100, self),
            'Nw': QIntValidator(1, 100, self),
            'Lp': QIntValidator(1, 100, self),
            'mu_r': QDoubleValidator(0.0, 1.0, 2, self),
            'sigma_d': QDoubleValidator(0.0, 1.0, 2, self),
            'shift_distance': QDoubleValidator(0.0, 2.0, 2, self),
            'extend_length': QDoubleValidator(0.0, 2.0, 2, self),
            'look_ahead_distance': QDoubleValidator(0.0, 2.0, 2, self),
            'target_finishing_time': QDoubleValidator(0.0, 100.0, 2, self),
            'without_walls': QIntValidator(0, 1, self),
            'allowed_jump_index': QIntValidator(0, 100, self),
            'how_many_points_forward': QIntValidator(1, 100, self),
            'max_ep_steps': QIntValidator(1, 1000, self),
            'obs_mean_vel': QDoubleValidator(0.0, 10.0, 2, self),
            'obs_std_vel': QDoubleValidator(0.0, 10.0, 2, self),
            'flip_chance': QDoubleValidator(0.0, 1.0, 2, self),
            'dynamic_obstacles_r': QIntValidator(0, 1, self),
            'dynamic_obstacles': QIntValidator(0, 1, self),
            'static_map': QIntValidator(0, 2, self),
            'consider_width': QIntValidator(0, 1, self),
        }
        with open(os.path.join(self.factory_env_dir, 'envs/default_params', 'path_params.json'), 'r') as file:
            default_values = json.load(file)

        params_layout = QGridLayout()
        row, col = 0, 0
        for param, validator in validators.items():
            label = QLabel(f"{param}:", parent)
            label.setToolTip(tooltips.get(param, ""))  # Set tooltip, or empty string if not found
            line_edit = QLineEdit(parent)
            line_edit.setValidator(validator)
            line_edit.setText(default_values[param])
            self.param_inputs[param] = line_edit  # Store reference to the input

            params_layout.addWidget(label, row, col)
            params_layout.addWidget(line_edit, row, col + 1)

            if col < 2:
                col += 2
            else:
                col = 0
                row += 1

        return params_layout
    
    def setup_atr_parameter_inputs(self, parent):
        # Define tooltips for parameters
        tooltips = {
            'dt': 'Time step for simulation or control loop',
            'wheel_radius': 'Radius of the robot wheel',
            'track_width': 'Distance between the centers of the two wheels of the robot',
            'atr_linear_vel_max': 'Maximum linear velocity of the robot',
        }

        validators = {
            'dt': QDoubleValidator(0.01, 1.0, 2, self),
            'wheel_radius': QDoubleValidator(0.01, 1.0, 3, self),
            'track_width': QDoubleValidator(0.1, 2.0, 2, self),
            'atr_linear_vel_max': QDoubleValidator(0.1, 1.0, 2, self),
        }
        with open(os.path.join(self.factory_env_dir, 'envs/default_params', 'atr_params.json'), 'r') as file:
            default_values = json.load(file)
        
        params_layout = QGridLayout()
        row, col = 0, 0

        for param, validator in validators.items():
            label = QLabel(f"{param}:", parent)
            label.setToolTip(tooltips.get(param, ""))  # Set tooltip, or empty string if not found
            line_edit = QLineEdit(parent)
            line_edit.setValidator(validator)
            line_edit.setText(default_values[param])
            self.param_inputs[param] = line_edit  # Store reference to the input

            params_layout.addWidget(label, row, col)
            params_layout.addWidget(line_edit, row, col + 1)

            if col < 2:
                col += 2
            else:
                col = 0
                row += 1
        return params_layout
    
    def setup_env_parameter_inputs(self, parent):
        # Define tooltips for parameters
        tooltips = {
            'full_plot': 'Whether to plot the dashboard',
            'rangefinder': 'Whether to use rangefinder',
        }

        validators = {
            'full_plot': QIntValidator(0, 1, self),
            'rangefinder': QIntValidator(0, 1, self),
        }

        with open(os.path.join(self.factory_env_dir, 'envs/default_params', 'env_params.json'), 'r') as file:
            default_values = json.load(file)

        params_layout = QGridLayout()
        row, col = 0, 0

        for param, validator in validators.items():
            label = QLabel(f"{param}:", parent)
            label.setToolTip(tooltips.get(param, ""))  # Set tooltip, or empty string if not found
            line_edit = QLineEdit(parent)
            line_edit.setValidator(validator)
            line_edit.setText(default_values[param])
            self.param_inputs[param] = line_edit  # Store reference to the input

            params_layout.addWidget(label, row, col)
            params_layout.addWidget(line_edit, row, col + 1)

            if col < 2:
                col += 2
            else:
                col = 0
                row += 1

        return params_layout
    
    def setup_sensor_parameter_inputs(self, parent):
        # Define tooltips for parameters
        tooltips = {
            'distance_threshold': 'how far the robot can see',
            'sensor_angle': 'total angle of the sensor',
            'nsectors': 'how many sectors to be divide default [20]',
            'num_points_on_walls': 'how many points on the walls should be considered',
            'num_points_on_obstacles': 'how many points on the obstacles should be considered',
            'narrow_angle': 'angles on the circle, how much from the angle difference to the starting bound',
            'angle_inbetween': 'angles on the circle, how much between the starting bound and the ending bound',
            'resolution': 'resolution of the rangefinder',
            'Sr': 'maximum rangefinder distance',
        }

        validators = {
            'distance_threshold': QDoubleValidator(0.1, 2.0, 2, self),
            'sensor_angle': QIntValidator(0, 360, self),
            'nsectors': QIntValidator(0, 100, self),
            'num_points_on_walls': QIntValidator(0, 100, self),
            'num_points_on_obstacles': QIntValidator(0, 100, self),
            'narrow_angle': QIntValidator(0, 360, self),
            'angle_inbetween': QIntValidator(0, 360, self),
            'resolution': QDoubleValidator(0.0, 20.0, 2, self),
            'Sr': QDoubleValidator(0.0, 10.0, 2, self),
        }

        with open(os.path.join(self.factory_env_dir, 'envs/default_params', 'sensor_params.json'), 'r') as file:
            default_values = json.load(file)
        
        params_layout = QGridLayout()
        row, col = 0, 0

        for param, validator in validators.items():
            label = QLabel(f"{param}:", parent)
            label.setToolTip(tooltips.get(param, ""))  # Set tooltip, or empty string if not found
            line_edit = QLineEdit(parent)
            line_edit.setValidator(validator)
            line_edit.setText(default_values[param])
            self.param_inputs[param] = line_edit  # Store reference to the input

            params_layout.addWidget(label, row, col)
            params_layout.addWidget(line_edit, row, col + 1)

            if col < 2:
                col += 2
            else:
                col = 0
                row += 1

        return params_layout

    def setup_reward_parameter_inputs(self, parent):
        # Define tooltips for parameters
        tooltips = {
            'alpha_lambda': 'trade-off paramter',
            'beta_lambda': 'trade-off paramter',
            'gamma_theta': 'obstacle avoidance reward parameter',
            'gamma_x': 'obstacle avoidance reward parameter',
            "epsilon_x": "deadzone distance of obstacle avoidance reward",
            'gamma_e': 'path following reward parameter',
            'alpha_r': 'existance reward parameter',
            'r_collision': 'collision reward',
            'r_arrived': 'arrived reward',
            'r_terminated': 'terminated reward',
            'max_total_reward': 'maximum total reward',
            'lagging_frac': 'lagging reward fraction',
            'r_oa_frac': 'obstacle avoidance reward fraction',
            "angle_resolution": "step size for the angle",
            "distance_resolution": "step size for the distance",
            "oa_result_resolution": "step size for the result of the obstacle avoidance reward",
        }

        validators = {
            'alpha_lambda': QDoubleValidator(0.0, 10.0, 2, self),
            'beta_lambda': QDoubleValidator(0.0, 10.0, 2, self),
            'gamma_theta': QDoubleValidator(0.0, 10.0, 2, self),
            'gamma_x': QDoubleValidator(0.0, 1.0, 2, self),
            "epsilon_x": QDoubleValidator(0.0, 1.0, 2, self),
            'gamma_e': QDoubleValidator(0.0, 1.0, 2, self),
            'alpha_r': QDoubleValidator(0.0, 1.0, 2, self),
            'r_collision': QDoubleValidator(-50000.0, 50000.0, 2, self),
            'r_arrived': QDoubleValidator(-50000.0, 50000.0, 2, self),
            'r_terminated': QDoubleValidator(-50000.0, 50000.0, 2, self),
            'max_total_reward': QDoubleValidator(-50000.0, 50000.0, 2, self),
            'lagging_frac': QDoubleValidator(0.0, 100.0, 2, self),
            'r_oa_frac': QDoubleValidator(0.0, 100.0, 2, self),
            "angle_resolution": QDoubleValidator(0.0, 0.5, 2, self),
            "distance_resolution": QDoubleValidator(0.0, 10.0, 2, self),
            "oa_result_resolution": QDoubleValidator(0.0, 10.0, 2, self),
        }

        params_file_path = os.path.join(self.factory_env_dir, 'envs/default_params', 'reward_params.json')
        with open(params_file_path, 'r') as file:
            default_values = json.load(file)

        params_layout = QGridLayout()
        row, col = 0, 0
        # Add Save button
        self.save_params_button = QPushButton("Save Parameters", parent)
        self.save_params_button.clicked.connect(self.save_parameters)

        # Add Load button
        self.load_params_button = QPushButton("Load Parameters", parent)
        self.load_params_button.clicked.connect(self.load_parameters)

        self.reward_dir_combo = QComboBox(parent)
        self.reward_dir_combo.addItem('default_params')
        self.reward_dir_combo.addItem('exp_params')
        self.reward_dir_combo.currentIndexChanged.connect(self.update_reward_params_files)  # When the index changes, update the file list
        self.reward_dir_combo.setCurrentIndex(0)  # Set default value

        self.reward_params_combo = QComboBox(parent)
        self.update_reward_params_files()

        for param, validator in validators.items():
            label = QLabel(f"{param}:", parent)
            label.setToolTip(tooltips.get(param, ""))  # Set tooltip, or empty string if not found
            line_edit = QLineEdit(parent)
            line_edit.setValidator(validator)
            line_edit.setText(default_values[param])
            self.reward_params_input[param] = line_edit  # Store reference to the input

            params_layout.addWidget(label, row, col)
            params_layout.addWidget(line_edit, row, col + 1)

            if col < 2:
                col += 2
            else:
                col = 0
                row += 1
        # Place buttons on the layout
        params_layout.addWidget(self.save_params_button, row + 1, 0)
        params_layout.addWidget(self.load_params_button, row + 1, 1)
        params_layout.addWidget(self.reward_dir_combo, row + 1, 2)
        params_layout.addWidget(self.reward_params_combo, row + 1, 3)

        return params_layout
    
    def update_reward_params_files(self):
        # Clear the current items
        self.reward_params_combo.clear()
        selected_dir = self.reward_dir_combo.currentText()
        params_dir_path = os.path.join(self.factory_env_dir, 'envs', selected_dir)
        reward_files = os.listdir(params_dir_path)

        # Filter out non .pth files if needed
        reward_files = [f for f in reward_files if f.endswith('.json')]
        # Add files to the combo box
        for file_name in reward_files:
            self.reward_params_combo.addItem(file_name)

    def save_parameters(self):
        params = {param: line_edit.text() for param, line_edit in self.reward_params_input.items()}

        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"reward_params_{current_time}.json"
        params_file_path = os.path.join(self.factory_env_dir, 'envs/exp_params', filename)

        with open(params_file_path, 'w') as file:
            json.dump(params, file, indent=4)
        # print("Parameters saved to reward_params.json")
        self.output_display.append(f"Parameters saved to {params_file_path}")

    def load_parameters(self):
        selected_dir = self.reward_dir_combo.currentText()
        selected_file = self.reward_params_combo.currentText()
        try:
            params_file_path = os.path.join(self.factory_env_dir, 'envs', selected_dir, selected_file)
            with open(params_file_path, 'r') as file:
                params = json.load(file)
            for param, value in params.items():
                if param in self.reward_params_input:
                    self.reward_params_input[param].setText(value)
            self.output_display.append(f"Load {selected_dir}/{selected_file}")
        except FileNotFoundError:
            self.output_display.append("No saved parameters file found.")
        except json.JSONDecodeError:
            self.output_display.append("Error decoding JSON from saved parameters file.")

    def toggle_parameters(self, param_type):
        # Toggle the visibility state in the dictionary
        self.parameter_layouts[param_type]['visible'] = not self.parameter_layouts[param_type]['visible']
        # Pass the layout to the toggle visibility function
        self.toggle_parameters_visibility(self.parameter_layouts[param_type]['layout'], self.parameter_layouts[param_type]['visible'])

    def toggle_parameters_visibility(self, layout, visible):
        for i in range(layout.count()):
            item = layout.itemAt(i)
            if item.widget():
                item.widget().setVisible(visible)

        # Force the layout and its parent widget to update and adjust to the changes
        layout.activate()  # This will force the layout to reapply its layout rules
        parent_widget = layout.parentWidget()
        if parent_widget:
            parent_widget.update()  # Update the parent widget
            parent_widget.repaint()  # Repaint the parent widget
            parent_widget.adjustSize()  # Adjust size if necessary
        # # Refresh the parent widget or layout to reflect the changes
        # layout.update()  # This will cause the layout to re-arrange its items
        # layout.parentWidget().adjustSize()  # This adjusts the size of the parent widget
        # layout.parentWidget().update()  # This updates the parent widget immediately

    def toggle_control_buttons_display(self, index):
        is_manual = (self.which_test_combo.currentData() == 0)
        is_benchmark = (self.which_test_combo.currentData() == 1)
        is_agent = (self.which_test_combo.currentData() == 2)
        for i in range(self.teleop_buttons_layout.count()):
            widget = self.teleop_buttons_layout.itemAt(i).widget()
            if is_manual:
                widget.show()
            else:
                widget.hide()
        for i in range(self.agent_selector_layout.count()):
            widget = self.agent_selector_layout.itemAt(i).widget()
            if is_agent or is_benchmark:
                widget.show()
            else:
                widget.hide()
    
    '''
       _____       _     _____     _    
      |_   _|__ __| |_  |_   _|_ _| |__ 
        | |/ -_|_-<  _|   | |/ _` | '_ \
        |_|\___/__/\__|   |_|\__,_|_.__/
                                        
    '''
    def create_test_tab(self):
        # Create test tab
        tab3 = QWidget()
        tab3_layout = QVBoxLayout()

        # Add info
        tab3_layout.addWidget(QLabel("This page is for testing the environment."
                                     "You can:\n"
                                     "- manually control the robot\n"
                                     "- use RL agent\n"
                                     "- use lattice planner\n"
                                     "to drive the robot in the simulation env."))
        ## Add args options
        args_layout = self.setup_args_inputs(tab3)
        tab3_layout.addLayout(args_layout)

        ## Add teleop buttons
        self.teleop_buttons_layout = self.setup_teleop_buttons(tab3)
        tab3_layout.addLayout(self.teleop_buttons_layout)

        self.agent_selector_layout = self.setup_agent_file_selector(tab3)
        tab3_layout.addLayout(self.agent_selector_layout)
        # # Add input console
        # self.input_display = InputTextEdit(self)
        # tab3_layout.addWidget(self.input_display)

        ## Add button
        button2_test = QPushButton('Run test', tab3)
        button2_test.clicked.connect(self.run_test)
        tab3_layout.addWidget(button2_test)

        tab3.setLayout(tab3_layout)        
        return tab3
    
    def setup_args_inputs(self, parent):
        # Create the layout and widgets for selecting the test and option
        args_layout = QGridLayout()
        
        which_test_label = QLabel("Which test to perform?", parent)
        self.which_test_combo = QComboBox(parent)
        self.which_test_combo.addItem("manual", 0)
        self.which_test_combo.addItem("benchmark", 1)
        self.which_test_combo.addItem("test agent", 2)
        self.which_test_combo.currentIndexChanged.connect(self.toggle_control_buttons_display)
        self.which_test_combo.setCurrentIndex(0)  # Set default value
        
        which_option_label = QLabel("Which env to play?", parent)
        self.which_option_combo = QComboBox(parent)
        self.which_option_combo.addItem("sim", 0)
        self.which_option_combo.addItem("real", 1)
        self.which_option_combo.setCurrentIndex(0)  # Set default value
        
        args_layout.addWidget(which_test_label, 0, 0)
        args_layout.addWidget(self.which_test_combo, 0, 1)
        args_layout.addWidget(which_option_label, 1, 0)
        args_layout.addWidget(self.which_option_combo, 1, 1)

        return args_layout
    
    def setup_agent_file_selector(self, parent):
        agent_selector_layout = QGridLayout()
        which_agent_label = QLabel("Which agent to test?", parent)

        # Create the directory combo box and populate it
        self.agent_dir_combo = QComboBox(parent)
        self.agent_dir_combo.addItem('agents')
        self.agent_dir_combo.addItem('good_agents')
        self.agent_dir_combo.currentIndexChanged.connect(self.update_agent_files)  # When the index changes, update the file list
        self.agent_dir_combo.setCurrentIndex(0)  # Set default value

        # Create the combo box and populate it with file names
        self.agent_file_combo = QComboBox(parent)
        self.update_agent_files()

        self.refresh_button = QPushButton("Refresh File List", parent)
        self.refresh_button.clicked.connect(self.update_agent_files)
        
        agent_selector_layout.addWidget(QLabel("Select Directory:"), 0, 0)
        agent_selector_layout.addWidget(self.agent_dir_combo, 0, 1)
        agent_selector_layout.addWidget(which_agent_label, 1, 0)
        agent_selector_layout.addWidget(self.agent_file_combo, 1, 1)
        agent_selector_layout.addWidget(self.refresh_button, 1, 2)
        
        
        # Hide the layout at start
        for i in range(agent_selector_layout.count()):
            widget = agent_selector_layout.itemAt(i).widget()
            widget.hide()
        return agent_selector_layout
    
    def update_agent_files(self):
        # Clear the current items
        self.agent_file_combo.clear()

        # List all files in the 'agents' folder
        selected_dir = self.agent_dir_combo.currentText()
        agent_files = os.listdir(selected_dir)

        # Filter out non .pth files if needed
        agent_files = [f for f in agent_files if f.endswith('.pth')]

        # Add files to the combo box
        for file_name in agent_files:
            self.agent_file_combo.addItem(file_name)

    # teleop
    def setup_teleop_buttons(self, parent):
        # Create the layout and widgets for teleoperation
        teleop_buttons_layout = QGridLayout()
        
        # Create and add buttons to the layout
        upButton = QPushButton("Up", parent)
        rotleftButton = QPushButton("Rot Left", parent)
        rotrightButton = QPushButton("Rot Right", parent)
        stopButton = QPushButton("Stop", parent)
        turnleftButton = QPushButton("Turn Left", parent)
        turnrightButton = QPushButton("Turn Right", parent)
        backleftButton = QPushButton("Back Left", parent)
        backrightButton = QPushButton("Back Right", parent)

        # Connect buttons to functions
        upButton.clicked.connect(lambda: self.send_key('w'))
        rotleftButton.clicked.connect(lambda: self.send_key('a'))
        rotrightButton.clicked.connect(lambda: self.send_key('d'))
        stopButton.clicked.connect(lambda: self.send_key('e'))  # Assuming 's' is the stop key

        turnleftButton.clicked.connect(lambda: self.send_key('j'))
        turnrightButton.clicked.connect(lambda: self.send_key('k'))
        backleftButton.clicked.connect(lambda: self.send_key('n'))
        backrightButton.clicked.connect(lambda: self.send_key('m'))

        teleop_buttons_layout.addWidget(upButton, 0, 0)
        teleop_buttons_layout.addWidget(stopButton, 0, 1)
        teleop_buttons_layout.addWidget(rotleftButton, 1, 0)
        teleop_buttons_layout.addWidget(rotrightButton, 1, 1)
        teleop_buttons_layout.addWidget(turnleftButton, 2, 0)
        teleop_buttons_layout.addWidget(turnrightButton, 2, 1)
        teleop_buttons_layout.addWidget(backleftButton, 3, 0)
        teleop_buttons_layout.addWidget(backrightButton, 3, 1)

        return teleop_buttons_layout

    def send_key(self, key):
        self.process.write(key.encode('utf-8'))

    def run_test(self):
        self.output_display.clear()  # Clear the QTextEdit widget
        args = []

        which_test = self.which_test_combo.currentData()
        which_option = self.which_option_combo.currentData()
        
        args.append("--which_test")
        args.append(str(which_test))
        
        if which_test == 2 or which_test == 1:
            args.append("--agent_dir")
            args.append(self.agent_dir_combo.currentText())
            args.append("--agent_file")
            args.append(self.agent_file_combo.currentText())
        
        args.append("--which_env")
        args.append(str(which_option))

        for param, line_edit in self.param_inputs.items():
            value = line_edit.text()
            args.append(f"--{param}")
            args.append(value)
        for param, line_edit in self.reward_params_input.items():
            value = line_edit.text()
            args.append(f"--{param}")
            args.append(value)
        # Use QProcess to run the script
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        # self.input_display.setProcess(self.process)
        self.process.start("python3", ["-u", "test.py"] + args)
    
    '''
       _____         _      _             _____     _    
      |_   _| _ __ _(_)_ _ (_)_ _  __ _  |_   _|_ _| |__ 
        | || '_/ _` | | ' \| | ' \/ _` |   | |/ _` | '_ \
        |_||_| \__,_|_|_||_|_|_||_\__, |   |_|\__,_|_.__/
                                  |___/                  
    '''
    def create_training_tab(self):
        # Create test tab
        tab = QWidget()
        tab_layout = QVBoxLayout()
        # Add info
        tab_layout.addWidget(QLabel("This page is for training the agent."))

        # Path parameter layout
        tab_layout.addWidget(QLabel("Arguments"))
        PPO_arguments_layout = self.setup_PPO_arguments_inputs(tab)
        tab_layout.addLayout(PPO_arguments_layout)


        ## Add args options
        PPO_bool_args_layout = self.setup_PPO_bool_args_inputs(tab)
        tab_layout.addLayout(PPO_bool_args_layout)

        ## Add button
        button_PPO = QPushButton('Run PPO', tab)
        button_PPO.clicked.connect(self.run_PPO)
        tab_layout.addWidget(button_PPO)

        ## Launch Tensorboard
        launch_tensorboard_layout = self.setup_launch_tensorboard(tab)
        tab_layout.addLayout(launch_tensorboard_layout)

        tab.setLayout(tab_layout)        
        return tab
    
    def setup_PPO_arguments_inputs(self, parent):
        # Define tooltips for parameters
        tooltips = {
            'exp-name': 'Experiment name, default: the filename where it is run without the .py extension',
            'gym-id': 'Gym environment ID, default: training-factory-v0',
            'learning-rate': 'Learning rate, default: 2e-4',
            'seed': 'Random seed, default: 1',
            'total-timesteps': 'Total timesteps, default: 200000',
            'torch-deterministic': 'Set PyTorch\'s CUDNN backend to deterministic, default: True',
            'cuda': 'Use CUDA (GPU), default: False',
            'track': 'Be tracked by Weights and Biases, default: False',
            'wandb-project-name': 'Weights and Biases project name, default: ATR-RL',
            'wandb-entity': 'Weights and Biases entity, default: None',
            'capture-video': 'Capture video of the training process, default: False',
            'num-envs': 'Number of parallel game environments, default: 4',
            'num-steps': 'Number of steps per game environment, default: 1024',
            'anneal-lr': 'Toggle learning rate annealing, default: True',
            'gae': 'Use GAE for advantage estimation, default: True',
            'gamma': 'Discount factor gamma, default: 0.999',
            'gae-lambda': 'Lambda for GAE, default: 0.95',
            'num-minibatches': 'Number of mini-batches per update, default: 32',
            'update-epochs': 'Number of epochs to update the policy, default: 10',
            'norm-adv': 'Toggle advantage normalization, default: True',
            'clip-coef': 'Surrogate clipping coefficient, default: 0.2',
            'clip-vloss': 'Toggle clipping of the value function loss, default: True',
            'ent-coef': 'Coefficient of the entropy loss, default: 0.0',
            'vf-coef': 'Coefficient of the value function loss, default: 0.5',
            'max-grad-norm': 'Maximum norm for gradient clipping, default: 0.5',
            'target-kl': 'Target KL divergence threshold, default: None',
        }

        validators = {
            'learning-rate': QDoubleValidator(1e-5, 1.0, 5, self),
            'seed': QIntValidator(1, 99999, self),
            'total-timesteps': QIntValidator(1, int(1e9), self),
            'num-envs': QIntValidator(1, 100, self),
            'num-steps': QIntValidator(1, 10000, self),
            'gamma': QDoubleValidator(0.0, 1.0, 3, self),
            'gae-lambda': QDoubleValidator(0.0, 1.0, 3, self),
            'num-minibatches': QIntValidator(1, 100, self),
            'update-epochs': QIntValidator(1, 100, self),
            'clip-coef': QDoubleValidator(0.0, 1.0, 3, self),
            'ent-coef': QDoubleValidator(0.0, 10.0, 3, self),
            'vf-coef': QDoubleValidator(0.0, 10.0, 3, self),
            'max-grad-norm': QDoubleValidator(0.0, 10.0, 3, self),
            # 'target-kl' does not have a default and hence may not need a validator
        }

        default_values = {
            'learning-rate': '2e-4',
            'seed': '1',
            'total-timesteps': '200000',
            'num-envs': '4',
            'num-steps': '1024',
            'gamma': '0.999',
            'gae-lambda': '0.95',
            'num-minibatches': '32',
            'update-epochs': '10',
            'clip-coef': '0.2',
            'ent-coef': '0.0',
            'vf-coef': '0.5',
            'max-grad-norm': '0.5',
            # 'target-kl' does not have a default and hence may not be included here
        }

        
        args_layout = QGridLayout()
        row, col = 0, 0

        for param, validator in validators.items():
            label = QLabel(f"{param}:", parent)
            label.setToolTip(tooltips.get(param, ""))  # Set tooltip, or empty string if not found
            line_edit = QLineEdit(parent)
            line_edit.setValidator(validator)
            line_edit.setText(default_values[param])
            self.PPO_arguments_input[param] = line_edit  # Store reference to the input

            args_layout.addWidget(label, row, col)
            args_layout.addWidget(line_edit, row, col + 1)

            if col < 2:
                col += 2
            else:
                col = 0
                row += 1

        return args_layout
    
    def setup_PPO_bool_args_inputs(self, parent):
        # Create the layout for the boolean arguments
        args_layout = QGridLayout()

        # Pair 1: torch-deterministic and cuda
        self.torch_deterministic_combo = self.create_bool_combo_box(True)
        self.cuda_combo = self.create_bool_combo_box(False)
        args_layout.addWidget(QLabel("torch-deterministic?"), 0, 0)
        args_layout.addWidget(self.torch_deterministic_combo, 0, 1)
        args_layout.addWidget(QLabel("cuda?"), 0, 2)
        args_layout.addWidget(self.cuda_combo, 0, 3)

        # Pair 2: track and capture-video
        self.track_combo = self.create_bool_combo_box(False)
        self.capture_video_combo = self.create_bool_combo_box(False)
        args_layout.addWidget(QLabel("track?"), 1, 0)
        args_layout.addWidget(self.track_combo, 1, 1)
        args_layout.addWidget(QLabel("capture-video?"), 1, 2)
        args_layout.addWidget(self.capture_video_combo, 1, 3)

        # Pair 3: anneal-lr and gae
        self.anneal_lr_combo = self.create_bool_combo_box(True)
        self.gae_combo = self.create_bool_combo_box(True)
        args_layout.addWidget(QLabel("anneal-lr?"), 2, 0)
        args_layout.addWidget(self.anneal_lr_combo, 2, 1)
        args_layout.addWidget(QLabel("gae?"), 2, 2)
        args_layout.addWidget(self.gae_combo, 2, 3)

        # Pair 4: norm-adv and clip-vloss
        self.norm_adv_combo = self.create_bool_combo_box(True)
        self.clip_vloss_combo = self.create_bool_combo_box(True)
        args_layout.addWidget(QLabel("norm-adv?"), 3, 0)
        args_layout.addWidget(self.norm_adv_combo, 3, 1)
        args_layout.addWidget(QLabel("clip-vloss?"), 3, 2)
        args_layout.addWidget(self.clip_vloss_combo, 3, 3)

        return args_layout

    def create_bool_combo_box(self, default_value):
        combo_box = QComboBox(self)
        combo_box.addItem("False", False)
        combo_box.addItem("True", True)
        combo_box.setCurrentIndex(combo_box.findData(default_value))
        return combo_box

    def run_PPO(self):
        self.output_display.clear()  # Clear the QTextEdit widget
        args = []

        # Assume you have self.which_test_combo and self.which_option_combo defined somewhere in your class
        which_test = self.which_test_combo.currentData()
        which_option = self.which_option_combo.currentData()

        # Add the boolean arguments to the args list
        args.append("--torch-deterministic")
        args.append(str(self.torch_deterministic_combo.currentData()))

        args.append("--cuda")
        args.append(str(self.cuda_combo.currentData()))

        args.append("--track")
        args.append(str(self.track_combo.currentData()))

        args.append("--capture-video")
        args.append(str(self.capture_video_combo.currentData()))

        args.append("--anneal-lr")
        args.append(str(self.anneal_lr_combo.currentData()))

        args.append("--gae")
        args.append(str(self.gae_combo.currentData()))

        args.append("--norm-adv")
        args.append(str(self.norm_adv_combo.currentData()))

        args.append("--clip-vloss")
        args.append(str(self.clip_vloss_combo.currentData()))

        # Add PPO arguments
        for param, line_edit in self.PPO_arguments_input.items():
            value = line_edit.text()
            if value:  # Add the argument only if it has a value
                args.append(f"--{param}")
                args.append(value)
        
        # Add Environment arguments(parameters)
        for param, line_edit in self.param_inputs.items():
            value = line_edit.text()
            args.append(f"--{param}")
            args.append(value)
        for param, line_edit in self.reward_params_input.items():
            value = line_edit.text()
            args.append(f"--{param}")
            args.append(value)

        # Use QProcess to run the script
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        # self.process.finished.connect(self.process_finished)  # Handle the process finished event
        self.process.start("python3", ["-u", "PPO.py"] + args)
    
    def setup_launch_tensorboard(self, parent):
        args_layout = QGridLayout()
        button_tensorboard = QPushButton('Lunch Tensorboard', parent)
        button_tensorboard.clicked.connect(self.run_tensorboard)

        self.which_exps_dir = QComboBox(parent)
        self.which_exps_dir.addItem("All exps", 0)
        self.which_exps_dir.addItem("Good exps", 1)
        self.which_exps_dir.setCurrentIndex(0)  # Set default value

        args_layout.addWidget(button_tensorboard, 0, 0)
        args_layout.addWidget(self.which_exps_dir, 0, 1)
        return args_layout
    
    def run_tensorboard(self):
        self.tensorboard_process = QProcess(self)
        self.tensorboard_process.readyReadStandardOutput.connect(self.handle_stdout)
        self.tensorboard_process.readyReadStandardError.connect(self.handle_stderr)
        if self.which_exps_dir.currentData() == 0:
            self.tensorboard_process.start("tensorboard", ["--logdir", "runs"])
        elif self.which_exps_dir.currentData() == 1:
            self.tensorboard_process.start("tensorboard", ["--logdir", "good_runs"])

    '''
        ___                  _     
       / __|___ _ _  ___ ___| |___ 
      | (__/ _ \ ' \(_-</ _ \ / -_)
       \___\___/_||_/__/\___/_\___|
                                   
    '''
    def handle_stdout(self):
        if self.process is not None:
            data = self.process.readAllStandardOutput().data().decode()
            self.output_display.append(data.strip())
        if self.tensorboard_process is not None:
            data = self.tensorboard_process.readAllStandardOutput().data().decode()
            self.output_display.append(data.strip())
        # self.output_display.append(data.strip())

    def handle_stderr(self):
        if self.process is not None:
            error_data = self.process.readAllStandardError().data().decode()
            self.output_display.append("\nErrors:\n" + error_data.strip())
        if self.tensorboard_process is not None:
            error_data = self.tensorboard_process.readAllStandardError().data().decode()
            self.output_display.append("\nErrors:\n" + error_data.strip())

    '''
       ___ _              ___         _      _   
      / __| |_ ___ _ __  / __| __ _ _(_)_ __| |_ 
      \__ \  _/ _ \ '_ \ \__ \/ _| '_| | '_ \  _|
      |___/\__\___/ .__/ |___/\__|_| |_| .__/\__|
                  |_|                  |_|       
    '''
    def stop_script(self):
        self.output_display.clear()  # Clear the QTextEdit widget
        if self.process:  # Make sure the process exists before trying to kill it
            self.process.terminate()
        if self.tensorboard_process:
            self.tensorboard_process.terminate()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())

