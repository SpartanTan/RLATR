# Notes for agents and experiments

## 0 walls 0 obs
| Syntax      |Date         | Description |
| ----------- | ----------- | ----------- |
| 356         | 11-27       |Trainig data doesn't look good but the agent works perfectly|


## 0 walls 1 obs
| Syntax      |Date         | Description |
| ----------- | ----------- | ----------- |
|10sec_824    | 11-28       |Trained on 1 map,but work on random map with random obstacles <br> pf not perfect, sometimes cannot reach goal, but walking around goal|

## 0 walls 2 obs
| Syntax      |Date         | Description |
| ----------- | ----------- | ----------- |
| 615   |   11-28     |Can always pass the first obstacle, but always crash into the second one|
|6128   |   11-29     |Quite good. Still can get stuck when obs are too concentrated. <br> Model is trained baed on 824. Env map with sigma_d=0.0. The random seed for numpy is removed. <br> Have tested with walls and more obstacles, still work <br> gamma=0.85, total-timesteps=500000, num-steps=512<br> seems work better with emulated rangefinder, but the overall success rate is not high. It will crah into obstacles from time to time. Maybe more training is needed.|
|4600070|12-2|Test training, good avoidence, but poor in path following. Terrible on emulated laser|

## Idea
Ignore speed penalty when the robot is too close to the obstacle.
But I'm not sure if it will get stuck over there.
Make lambda as the state instead of log10(lambda)

The obstacle reward still needs tuning. The r_oa should be kept below 0.
How_many_points_forward is suggested to be 3 or more. Smaller number will make it less possible to success.
## Thesis plan
Remove recurrent network and LSTM part.
