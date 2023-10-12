# 3dof_arm_robot
The main goals of this project is to build a simulation of a Three Degrees of Freedom (3 DOF) robotic arm using OpenGL and control the robot based on Task Space Control Algorithm with line trajectory path. 
- The mathematical model of 3 DOF robotic arm is obtained from International Journal of Computer Applications with the title is **Design of a Three Degrees of Freedom Robotic Arm** by Farman, Madiha & Al-Shaibah, Muneera & Aoraiath, Zoha & Jarrar, Firas.
- The OpenGL template is obtained from Robotics and Mechatronics Lectures in University of Indonesia and was created by Dr. Abdul Muis, M.Eng (Autonomous Control Electronics (ACONICS) Research Group).

<h1>Mathematical Model</h1>

<h2>Denavit-Hartenberg Parameters</h2>

The Denavit-Hartenberg (D-H) parameters were determined as shown in the table below. These parameters can be used to determine forward kinematic expressions in $xyz$-axis by turning it to homogenous transformation matrix.
| $i$ | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$ |
| --- | --- | --- | --- | --- |
| 1 | $0^o$ | $0$ | $L_1$ | $\theta_1$ |
| 2 | $90^o$ | $0$ | $0$ | $\theta_2$ |
| 3 | $0^o$ | $L_2$ | $0$ | $\theta_3$ |
| 4 | $0^o$ | $L_3$ | $0$ | $0^o$ |

<h2>Forward Kinematics</h2>

- $x = L_2 \cos(\theta_1) \cos(\theta_2) + L_3 \cos(\theta_1) \cos(\theta_2 + \theta_3)$
- $y = L_2 \sin(\theta_1) \cos(\theta_2) + L_3 \sin(\theta_1) \cos(\theta_2 + \theta_3)$
- $z = L_1 + L_2 \sin(\theta_2) + L_3 \sin(\theta_2 + \theta_3)$

<h2>Jacobian Matrix</h2>

$$
J = \begin{pmatrix}
\frac{\partial x}{\partial \theta_1} & \frac{\partial x}{\partial \theta_2} & \frac{\partial x}{\partial \theta_3}\\
\frac{\partial y}{\partial \theta_1} & \frac{\partial y}{\partial \theta_2} & \frac{\partial y}{\partial \theta_3}\\
\frac{\partial z}{\partial \theta_1} & \frac{\partial z}{\partial \theta_2} & \frac{\partial z}{\partial \theta_3}\\
\end{pmatrix}
$$

where

$\frac{\partial x}{\partial \theta_1} = - L_2 \sin(\theta_1) \cos(\theta_2) - L_3 \sin(\theta_1) \cos(\theta_2 + \theta_3)$

$\frac{\partial x}{\partial \theta_2} = - L_2 \cos(\theta_1) \sin(\theta_2) - L_3 \cos(\theta_1) \sin(\theta_2 + \theta_3)$

$\frac{\partial x}{\partial \theta_3} = - L_3 \cos(\theta_1) \sin(\theta_2 + \theta_3)$

$\frac{\partial y}{\partial \theta_1} = L_2 \cos(\theta_1) \cos(\theta_2) + L_3 \cos(\theta_1) \cos(\theta_2 + \theta_3)$

$\frac{\partial y}{\partial \theta_2} = - L_2 \sin(\theta_1) \sin(\theta_2) - L_3 \sin(\theta_1) \sin(\theta_2 + \theta_3)$

$\frac{\partial y}{\partial \theta_3} = - L_3 \sin(\theta_1) \sin(\theta_2 + \theta_3)$

$\frac{\partial z}{\partial \theta_1} = 0$

$\frac{\partial z}{\partial \theta_2} = L_2 \cos(\theta_2) + L_3 \cos(\theta_2 + \theta_3)$

$\frac{\partial z}{\partial \theta_3} = L_3 \cos(\theta_2 + \theta_3)$

<h2>Inverse Jacobian Matrix</h2>

$$
J^{-1} = \begin{pmatrix}
J_{11} & J_{12} & J_{13}\\
J_{21} & J_{22} & J_{23}\\
J_{31} & J_{32} & J_{33}\\
\end{pmatrix}
$$

where

$J_{11} = -\frac{\sin(\theta_1)}{L_3\cos(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1) + L_3\cos(\theta_2 + \theta_3)\sin(\theta_1)\sin(\theta_1) + L_2\cos(\theta_1)\cos(\theta_1)\cos(\theta_2) + L_2\cos(\theta_2)\sin(\theta_1)\sin(\theta_1)}$

$J_{12} = \frac{\cos(\theta_1)}{L_3\cos(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1) + L_3\cos(\theta_2 + \theta_3)\sin(\theta_1)\sin(\theta_1) + L_2\cos(\theta_1)\cos(\theta_1)\cos(\theta_2) + L_2\cos(\theta_2)\sin(\theta_1)\sin(\theta_1)}$

$J_{13} = 0$

$J_{21} = -\frac{\cos(\theta_2 + \theta_3)\cos(\theta_1)}{L_2\cos(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1)\sin(\theta_2) - L_2\sin(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1)\cos(\theta_2) + L_2\cos(\theta_2 + \theta_3)\sin(\theta_1)\sin(\theta_1)\sin(\theta_2) - L_2\sin(\theta_2 + \theta_3)\cos(\theta_2)\sin(\theta_1)\sin(\theta_1)}$

$J_{22} = -\frac{\cos(\theta_2 + \theta_3)\sin(\theta_1)}{L_2\cos(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1)\sin(\theta_2) - L_2\sin(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1)\cos(\theta_2) + L_2\cos(\theta_2 + \theta_3)\sin(\theta_1)\sin(\theta_1)\sin(\theta_2) - L_2\sin(\theta_2 + \theta_3)\cos(\theta_2)\sin(\theta_1)\sin(\theta_1)}$

$J_{23} = -\frac{\sin(\theta_2 + \theta_3)}{L_2\cos(\theta_2 + \theta_3)\sin(\theta_2) - L_2\sin(\theta_2 + \theta_3)\cos(\theta_2)}$

$J_{31} = \frac{\cos(\theta_1)(L_3\cos(\theta_2 + \theta_3) + L_2\cos(\theta_2))}{L_2L_3\cos(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1)\sin(\theta_2) - L_2L_3\sin(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1)\cos(\theta_2) + L_2L_3\cos(\theta_2 + \theta_3)\sin(\theta_1)\sin(\theta_1)\sin(\theta_2) - L_2L_3\sin(\theta_2 + \theta_3)\cos(\theta_2)\sin(\theta_1)\sin(\theta_1)}$

$J_{32} = \frac{\sin(\theta_1)(L_3\cos(\theta_2 + \theta_3) + L_2\cos(\theta_2))}{L_2L_3\cos(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1)\sin(\theta_2) - L_2L_3\sin(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1)\cos(\theta_2) + L_2L_3\cos(\theta_2 + \theta_3)\sin(\theta_1)\sin(\theta_1)\sin(\theta_2) - L_2L_3\sin(\theta_2 + \theta_3)\cos(\theta_2)\sin(\theta_1)\sin(\theta_1)}$

$J_{33} = \frac{L_3\sin(\theta_2 + \theta_3) + L_2\sin(\theta_2)}{L_2L_3\cos(\theta_2 + \theta_3)\sin(\theta_2) - L_2L_3\sin(\theta_2 + \theta_3)\cos(\theta_2)}$

<h1>Control Flow</h1>

<p align="center">
  <img src="https://github.com/JordyMarcius/3dof_arm_robot/assets/65435469/9b5c04cd-e82b-430d-b883-5c217a2ae79c"/>
</p>

Flow of task space control: 
- Sensors read the angle position ($\theta_1, \theta_2, \theta_3$) of each servo motor.
- These angles ($\theta_1, \theta_2, \theta_3$) are used to calculate the coordinate ($x, y, z$) of end effector using forward kinematic expressions.
- Compute line trajectory path. If the control algorithm is implemented correctly, the end effector of the robot will follow this line repeatedly in a certain time.
- Calculate error of end effector coordinates.
- Input error to PID controller to calculate coordinate acceleration in $xyz$-axis ($\ddot{x}, \ddot{y}, \ddot{z}$).
- Input angle position ($\theta_1, \theta_2, \theta_3$) and coordinate acceleration ($\ddot{x}, \ddot{y}, \ddot{z}$) to inverse jacobian function. It will return the angle acceleration reference ($\ddot{\theta_1}, \ddot{\theta_2}, \ddot{\theta_3}$). 
- Calculate PWM signal from output signal of inverse jacobian function.
- Calculate angle speed in $xyz$-axis ($\dot{\theta_1}, \dot{\theta_2}, \dot{\theta_3}$) using PWM signal and motor model.
- Calculate the motor position ($\theta_1, \theta_2, \theta_3$) by adding the previous angle position with integral of angle speed.
- Wait for the next loop and start from the first point.

<h1>Simulation</h1>

*Please click the start button in the right top of the .gif picture if the .gif motion is disabled.*

![sumbu_x_Gif](https://github.com/JordyMarcius/4dof_arm_robot/assets/65435469/83a8258e-7abf-44e9-928b-3b480e0fe042)
Figure 1. Robotic arm movement in $x$-axis

![sumbu_y_Gif](https://github.com/JordyMarcius/4dof_arm_robot/assets/65435469/66ca2859-b413-4c6c-9c42-1d5fe1c8fc35)
Figure 2. Robotic arm movement in $y$-axis

![sumbu_z_Gif](https://github.com/JordyMarcius/4dof_arm_robot/assets/65435469/20317ebf-1973-4b40-8507-b56b4ea6200d)
Figure 3. Robotic arm movement in $z$-axis

![sumbu_full_gerak_Gif](https://github.com/JordyMarcius/4dof_arm_robot/assets/65435469/8d082b2a-e4f1-4db6-abf9-0e51abe6345b)
Figure 4. Robotic arm movement in $xyz$-axis

<h1>Results Comparison</h1>

Below are the results comparison of an actual and reference of end effector coordinate. The purple line shows the reference coordinate and the green line shows the actual coordinate of end effector in $xyz$-axis. As can be seen in those figures, the end effector of this robotic arm can track the reference coordinate and follow the line trajectory. It indicates that the control algorithm was implemented correctly.

![image](https://github.com/JordyMarcius/3dof_arm_robot/assets/65435469/f420d7cb-e81c-41ef-ae20-f4ba02b2c175)
Figure 5. Comparison of $x$ coordinate.

![image](https://github.com/JordyMarcius/3dof_arm_robot/assets/65435469/f72c69a1-5a7e-40b5-8709-f4bf846182cf)
Figure 6. Comparison of $y$ coordinate.

![image](https://github.com/JordyMarcius/3dof_arm_robot/assets/65435469/f6da398c-a32c-474a-af16-9c75bfc9d5b0)
Figure 7. Comparison of $z$ coordinate.

<h1>References</h1>

- Farman, Madiha & Al-Shaibah, Muneera & Aoraiath, Zoha & Jarrar, Firas. (2018). Design of a Three Degrees of Freedom Robotic Arm. International Journal of Computer Applications. 179. 12-17. 10.5120/ijca2018916848. 
