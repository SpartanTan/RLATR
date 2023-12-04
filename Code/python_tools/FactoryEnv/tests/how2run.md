# How to test the agent 
### Install required dependencies
1. Create a new conda environment with python 3.10
2. Change directory to atr_rl/Documentation/How2s
    ```pip install -r requirements.txt```
### 2. Verify installation
1. Change directory to atr_rl/Code/python_tools/FactoryEnv/tests
2. ```python gui.py```
3. The GUI should pop up. Check the '--install' box then click 'Is environment installed?' button. If everything's correct, you should see the full plot. 
4. In case the factory environment cannot be instaled in this way, then first change directory to python_tools then manually install the environment by
    ```pip install -e FactoryEnv```
### 3. Test the agent
1. Change parmeters in 'Parameters' tab to set up the length of the map, the number of obstacles and whether to use emulated rangefinder.
2. Go to 'Test run' tab, select 'test agent' in 'Which test to perform'. Select 'sim' under 'Which environment to play' to run simulated map or 'real' to run NONA factory map.
3. Under 'Select Directory', select 'good_agents', then select a proper agent. For testing with multiple obstacles and walls, the '0walls2obs10sec_6128.pth' is recommended. 
4. Click 'Run test' button, you should see the annimation pop up.