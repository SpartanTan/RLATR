import keyboard
import threading
import numpy as np
import time


import os
import signal
import sys
import termios
import tty

class TelepControl:
    def __init__(self, ratio):
        self.ratio = ratio
        self.action = np.array([0, 0])
        self.std_speed = np.array([0.8, 0.8])
        self.running = False
        self.thread = threading.Thread(target=self.update, args=())
    
    def start(self):
        self.running = True
        self.thread.start()
    
    def update(self):
        while self.running:
            if keyboard.is_pressed('w'):
                self.action = self.std_speed * self.ratio
            elif keyboard.is_pressed('s'):
                self.action = -self.std_speed * self.ratio
            elif keyboard.is_pressed('a'):
                self.action = np.array([0.8, -0.8]) * self.ratio
            elif keyboard.is_pressed('d'):
                self.action = np.array([-0.8, 0.8]) * self.ratio

    def stop(self):
        self.running = False
        self.thread.join()

    def get_action(self):
        return self.action

        
def main():
    telep = TelepControl(1)
    telep.start()
    while True:
        action = telep.get_action()
        print(action)
        time.sleep(1)

class NewTeleopControl:
    def __init__(self, ratio):
        self.ratio = ratio
        self.key = 'e'
        self.std_speed = np.array([0.8, 0.8])
        self.running = True

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            if ch == 'q':  # Quit condition
                self.running = False
            if ch == '\x1b':
                ch += sys.stdin.read(1)
                ch += sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def get_action(self):
        if self.is_pressed('w'):
            action = self.std_speed * self.ratio
        elif self.is_pressed('s'):
            action = -self.std_speed * self.ratio
        elif self.is_pressed('a'):
            action = np.array([0.8, -0.8]) * self.ratio
        elif self.is_pressed('d'):
            action = np.array([-0.8, 0.8]) * self.ratio
        elif self.is_pressed('e'):
            action = np.array([0.0, 0.0]) * self.ratio
        return action
    
    def is_pressed(self, ch):
        if ch == self.key:
            return True

    def run_keyboard(self):
        while self.running:
            self.key = self.get_key()

class robot:
    def __init__(self, ratio):
        self.teleop = NewTeleopControl(ratio)
        self.ratio = ratio
        self.action = np.array([0.0, 0.0])
        self.std_speed = np.array([0.8, 0.8])
        self.thread = threading.Thread(target=self.teleop.run_keyboard)
        self.thread.start()

    def run(self):
        try:
            while self.teleop.running:
                self.action = self.teleop.get_action()
                print(f'\raction input: {self.action}')
                time.sleep(0.2)
        finally:
            self.thread.join()
            print("\rThread stopped.")

if __name__ == "__main__":
    rb = robot(1)
    rb.run()