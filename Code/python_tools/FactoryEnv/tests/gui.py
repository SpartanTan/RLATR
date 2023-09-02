import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QFileDialog, QLabel, QLineEdit, QTextEdit, QHBoxLayout, QCheckBox, QTabWidget, QComboBox, QGridLayout

from PyQt5.QtCore import QProcess


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
        self.initUI()
        self.process = None

    def initUI(self):
        self.setWindowTitle(self.title)
        layout = QVBoxLayout()
        self.tab_widget = QTabWidget()
        
        
        # Create install tab
        tab1 = QWidget()
        tab1_layout = QVBoxLayout()
        ## Add info
        tab1_layout.addWidget(QLabel("This tab is for installing and testing the environment.\nCheck the --install box below if you want to install the training environment."))
        ## Add args options
        boolean_arg_checkbox = QCheckBox(" --install", tab1)
        tab1_layout.addWidget(boolean_arg_checkbox)
        ## Add button
        button1_is_env_installed = QPushButton('Is environment installed?', tab1)
        button1_is_env_installed.clicked.connect(self.run_script_is_env_installed)
        tab1_layout.addWidget(button1_is_env_installed)
        ## synthesis tab1
        tab1.setLayout(tab1_layout)
        self.tab_widget.addTab(tab1, "Install")
        
        # Create test tab
        tab2 = QWidget()
        tab2_layout = QVBoxLayout()

        ## Add info
        tab2_layout.addWidget(QLabel("This page is for testing the environment."
                                     "You can:\n"
                                     "- manually control the robot\n"
                                     "- use RL agent\n"
                                     "- use lattice planner\n"
                                     "to drive the robot in the simulation env."))
        ## Add args options
        which_test_label = QLabel("Which test to perform?")
        self.which_test_combo = QComboBox(tab2)
        self.which_test_combo.addItem("manual", 0)
        self.which_test_combo.currentIndexChanged.connect(self.toggle_control_buttons_display)
        self.which_test_combo.addItem("benchmark", 1)
        self.which_test_combo.addItem("test agent", 2)
        self.which_test_combo.setCurrentIndex(0)  # Set default value
        tab2_layout.addWidget(which_test_label)
        tab2_layout.addWidget(self.which_test_combo)

        which_option_label = QLabel("Which option to perform?")
        self.which_option_combo = QComboBox(tab2)
        self.which_option_combo.addItem("run", 0)
        self.which_option_combo.addItem("save", 1)
        self.which_option_combo.addItem("load", 2)
        self.which_option_combo.setCurrentIndex(0)  # Set default value
        tab2_layout.addWidget(which_option_label)
        tab2_layout.addWidget(self.which_option_combo)

        ## Add teleop buttons
        self.controlButtonLayout = QGridLayout()
        upButton = QPushButton("Up", tab2)
        leftButton = QPushButton("Left", tab2)
        rightButton = QPushButton("Right", tab2)
        stopButton = QPushButton("Stop", tab2)

        # Connect buttons to functions
        upButton.clicked.connect(lambda: self.send_key('w'))
        leftButton.clicked.connect(lambda: self.send_key('a'))
        rightButton.clicked.connect(lambda: self.send_key('d'))
        stopButton.clicked.connect(lambda: self.send_key('e'))

        self.controlButtonLayout.addWidget(upButton, 0, 0)
        self.controlButtonLayout.addWidget(stopButton, 0, 1)
        self.controlButtonLayout.addWidget(rightButton, 1, 0)
        self.controlButtonLayout.addWidget(leftButton, 1, 1)
        tab2_layout.addLayout(self.controlButtonLayout)
        
        # # Add input console
        # self.input_display = InputTextEdit(self)
        # tab2_layout.addWidget(self.input_display)

        ## Add button
        button2_test = QPushButton('Run test', tab2)
        button2_test.clicked.connect(self.run_test)
        tab2_layout.addWidget(button2_test)

        tab2.setLayout(tab2_layout)        
        self.tab_widget.addTab(tab2, "Test run")

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
        self.show()
    
    def toggle_control_buttons_display(self, index):
        is_manual = (self.which_test_combo.currentData() == 0)
        for i in range(self.controlButtonLayout.count()):
            widget = self.controlButtonLayout.itemAt(i).widget()
            if is_manual:
                widget.show()
            else:
                widget.hide()

    def stop_script(self):
        if self.process:  # Make sure the process exists before trying to kill it
            self.process.terminate()

    def send_key(self, key):
        self.process.write(key.encode('utf-8'))

    def run_script(self):
        self.output_display.clear()  # Clear the QTextEdit widget

        args = []
        if self.boolean_arg_checkbox.isChecked():
            args.extend(["--TorF", "true"])
        else:
            args.extend(["--TorF", "false"])
        
        # Use QProcess to run the script
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)

        self.process.start("python3", ["-u", "is_env_registered.py"] + args)

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

    def run_test(self):
        self.output_display.clear()  # Clear the QTextEdit widget
        args = []
        which_test = self.which_test_combo.currentData()
        which_option = self.which_option_combo.currentData()
        
        args.append("--which-test")
        args.append(str(which_test))
        
        args.append("--which-option")
        args.append(str(which_option))

        # print("python3", ["-u", "test.py"] + args)
        # Use QProcess to run the script
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        # self.input_display.setProcess(self.process)
        self.process.start("python3", ["-u", "test.py"] + args)

    def handle_stdout(self):
        data = self.process.readAllStandardOutput().data().decode()
        self.output_display.append(data.strip())

    def handle_stderr(self):
        error_data = self.process.readAllStandardError().data().decode()
        self.output_display.append("\nErrors:\n" + error_data.strip())



if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())

