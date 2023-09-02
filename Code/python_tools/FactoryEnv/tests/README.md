# Readme for tests
## Sensor model testing
In file 'why.py' you can test the simulated lidar sensor. Just simply navigate to the directory then `python why.py` should work. The current problem is, it takes almost 3 seconds to loop 300 steps under the resolution set to 3 degrees. As for reference, the current environment using my urgly method takes only 0.27 seconds to finish 300 steps.

The accuracy of this method is of course better than my urgly one. But I do not like the time cost. I have tried using `y=m*x` to represent the laser beam, and hoped that it can be vectorized in numpy so that accelerate the computation. But it didn't. It costs even more time. And it requires to transform all obstacles and walls into the car frame. For obstacles its' ok, but for walls, I think it will waste a lot of time transforming the walls into the correct frame. 

Anyway, if I cannot improve it any more, I might uset my urgly way for training and use this method for testing. Or, if it won't take 10 billion years when using the sensor model, I might use it for training as well.