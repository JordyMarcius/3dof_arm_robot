# 3dof_arm_robot
The main goals of this project is to build a simulation of a Three Degrees of Freedom (3 DOF) robotic arm using OpenGL and control the robot based on Task Space Control Algorithm with line trajectory path. 
- The mathematical model of 3 DOF robotic arm is obtained from International Journal of Computer Applications with the title is **Design of a Three Degrees of Freedom Robotic Arm** by Farman, Madiha & Al-Shaibah, Muneera & Aoraiath, Zoha & Jarrar, Firas.
- The OpenGL template is obtained from Robotics and Mechatronics Lectures in University of Indonesia and was created by Dr. Abdul Muis, M.Eng (Autonomous Control Electronics (ACONICS) Research Group).

<h1>Mathematical Model</h1>

<h2>Denavit-Hartenberg Parameters</h2>

The Denavit-Hartenberg (D-H) parameters were determined as shown in the table below.
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

$ J_{11} = -\frac{\sin(\theta_1)}{L_3\cos(\theta_2 + \theta_3)\cos(\theta_1)\cos(\theta_1) + L_3\cos(\theta_2 + \theta_3)\sin(\theta_1)\sin(\theta_1) + L_2\cos(\theta_1)\cos(\theta_1)\cos(\theta_2) + L_2\cos(\theta_2)\sin(\theta_1)\sin(\theta_1)}$

![sumbu_x_Gif](https://github.com/JordyMarcius/4dof_arm_robot/assets/65435469/83a8258e-7abf-44e9-928b-3b480e0fe042)

![sumbu_y_Gif](https://github.com/JordyMarcius/4dof_arm_robot/assets/65435469/66ca2859-b413-4c6c-9c42-1d5fe1c8fc35)

![sumbu_z_Gif](https://github.com/JordyMarcius/4dof_arm_robot/assets/65435469/20317ebf-1973-4b40-8507-b56b4ea6200d)

![sumbu_full_gerak_Gif](https://github.com/JordyMarcius/4dof_arm_robot/assets/65435469/8d082b2a-e4f1-4db6-abf9-0e51abe6345b)

<h1>References</h1>

- Farman, Madiha & Al-Shaibah, Muneera & Aoraiath, Zoha & Jarrar, Firas. (2018). Design of a Three Degrees of Freedom Robotic Arm. International Journal of Computer Applications. 179. 12-17. 10.5120/ijca2018916848. 
