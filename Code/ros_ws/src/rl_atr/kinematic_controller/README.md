<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS_HTML"></script>

# kinematic_controller

## Description

---

This package provides the kinematic control for the ATR. Together with the atr_driver forms the ATR's Low-level control system. The communication interface of the node provided by this package and the other nodes is depicted in the following figure:

![Simple model](docs/figures/atr_controller_driver_96.png)

The main task of this node is to transform Twist commands into wheel velocity commands. The Twist commands represent the ATR's 3DOF velocity relative to the world coordinate frame (wcf), i.e. the desired linear velocities (x and y) and angular velocity (z) of the ATR in the wcf. These wheel velocity commands will be connected to the atr_driver (<https://gitlab.com/volvo_gto/gpss_mvp/control/atr_driver/-/tree/vanilla>) to publish the atr_state and visualize the ATR in rviz.

### Input

The input of this node is the desired Twist velocity for the ATR as **geometry_msgs/TwistStamped** message.

### Output

This node produces two outputs:

1. The wheel velocity commands as an **ATRJointCommand** message (<https://gitlab.com/volvo_gto/gpss_mvp/shared/gpss_interfaces/atr_state_msgs/-/blob/vanilla/msg/ATRJointCommand.msg>)

2. The internal control data for tunning purposes. This data is published as **ATRControlData** message (<https://gitlab.com/volvo_gto/gpss_mvp/shared/gpss_interfaces/atr_state_msgs/-/blob/vanilla/msg/ATRControlData.msg>)

### Common methods

From the kinematics of the ATR robot, we can derive the transform between wheel angular velocity and the ATR's 3DOF velocity relative to the wcf.

- Track width of the robot (or robot length) $L$
- Wheel radius $R$
- Heading angle: $\theta$
- Wheel angular speed: $\omega_i$, $i=R,L$
- Robot rotational speed: $\omega$
- Two angle constants, $C = cos(\theta), S=sin(\theta)$ 

Linear velocity w.t. wcf

$v_x = v \cdot cos(\theta)$

$v_y = v \cdot sin(\theta)$

Linear veolcity w.t. atr frame

$v = \frac{\omega_L*R + \omega_R*R}{2}$

$\omega = (\omega_R*R - \omega_L*R)/L$

Transformation

$\dot{X}=\begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}=JW=J\begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}$  

$J=\begin{bmatrix}
    \frac{R}{2}C &  \frac{R}{2}C \\
    \frac{R}{2}S &  \frac{R}{2}S \\
     \frac{R}{L} &  -\frac{R}{L}
    \end{bmatrix}$  

However in the controller, we receive the desired Twist velocity, which is the $\dot{X}$ here, thus we need to have a inversed $J$, $J^{\#}$. Notice that the $J$ is not a square matrix, thus we need to caculate its pseudo-inverse.

The method to calculate a pseudo-inverse is 
$A^{\dag}=(A^TA)^{-1}A^T$. Using matlab symbolic toolbox, we can compose the J-inverse as

$J^{\#}=\begin{bmatrix}
            \frac{C}{R} & \frac{S}{R} & \frac{L}{2R} \\
            \frac{C}{R} & \frac{S}{R} & -\frac{L}{2R}
        \end{bmatrix}$

Matlab code
```matlab
syms R L RL T theta real
C = cos(theta);
S = sin(theta);
Jn = [R*C/2 R*C/2;
      R*S/2 R*S/2;
      R/L -R/L]
iJ = simplify(pinv(Jn))
```
The final transformation equation will be

$W=
    \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}
     = J^{\#} \dot{X}
     =\begin{bmatrix}
            \frac{C}{R} & \frac{S}{R} & \frac{L}{2R} \\
            \frac{C}{R} & \frac{S}{R} & -\frac{L}{2R}
        \end{bmatrix}
        \cdot
        \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}$
        
To this end, we implement the transform from a Twist command to a wheel speed command.

## Pipeline
The controller node subscribes to two topics: \teleop_1 and \atr_driver_state_1

\teleop_1: [vx, vy, wz] in the wcf
\atr_driver_state_1: [x, y, theta, vx, vy, wz] in the wcf

Thus the state and the teleop_ref_ get updated.

Then a 10ms timer triggers the controller_.update(tc_), gets and sends the control data and wheel speeds.

### update(tc_):

calculate the reference by calculate_desired_state(dt). dt is the time interval between two updates.  

**calculate_desired_state(dt)**

We need reference $X_d$. $X_d$ is the estimated position of the ATR given a desired $\dot{X_d}$. $\dot{X_d}$ is given by the teleop_ref_, which is by the trajectory node, [vx, vy, wz]. 

Thus in calculate_desired_state
First give the current pos. This formulates the translation part of the rotation matrix.
```cpp
Tm_w.translation() << desired_state_.at(0), desired_state_.at(1), 0;
```
Get reference position wrt ATR cf. Notice that the [vx, vy, wz] are in wcf. Thus the Tm_w is an identity matrix.
```cpp
Tm_w.setIdentity();
```
This is the point in ATR cf. Remember here we assume the orientation of ATR cf is the same as the wcf because the velocities are in wcf. 
```cpp
Eigen::Vector3d ref_pos_m(dt * teleop_ref.at(0), dt * teleop_ref.at(1), 0);
```
Last step to perform the transform
```cpp
ref_pos_w = Tm_w * ref_pos_m.homogeneous();
```
In equation:
$P_{world}^{r} = T_{world}^{robot} * P_{robot}^{r}$

That is, the new position in wcf is the transform matrix multiplied by the new position in robot cf.

To this point we get $X_d$, the reference position in wcf. 3D vector.

**Formulate error vecotrs**

$\dot{X} = K_p\Delta X + K_d \Delta \dot{X}$  

$\Delta X = X_d ​− X$  
We already have $X_d$, the reference position. X should be extracted from the state_ variable which in fact contains 6 elements. [x, y, \theta]  

$\Delta \dot{X} = \dot{X}_d​−\dot{X}$   
We need a velocity vector here. $\dot{X}_d$ is from the difference between two reference positions, the current and the last, divided by dt.
```cpp
  xpd_w = (xd_w - xd_old_w_) / dt;
```
Same for \dot{X} but use the difference between current state and the last state
```cpp
  xp_w = (x_w - x_old_w_) / dt;
```
Now we have all error vectors, so that the final control velocity [vx, vy, w] can be calculated by  
$\dot{X} = K_p\Delta X + K_d \Delta \dot{X}$  

However, the control input for ATR should be the wheel speed instead of the needed velocity in wcf. Thus we need to transform the $\dot{X}$ to wheel speed by  
$W = J^{\#} \dot{X}$  
As have stated above.

## Get and publish wheel speed
We already calculated the wheel speed from above, next step is to send it out.