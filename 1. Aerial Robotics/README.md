# [Aerial Robotics](https://www.coursera.org/learn/robotics-flight)

## 1. Introduction
Unmanned Aerial Vehicles(UAV) = Remotely Piloted Vehicles(RPV) = Aerial Robots = Drones  

### 1.1 Quadrotors
#### Types of Micro Air Vehicles
- Fixed wing
- Flagging wing
  - Insect flight
  - Avian flight
- Rotor crafts
  - Helicopter
  - Ducted fan
  - Co-axial

#### Quadrotors
A quadrotor consists of 4 independently controlled rotors mounted on a rigid frame. If we take one of the rotors and spin that rotor **faster**, we will cause the robot to pitch in one direction. The quadrotor has 4 rotors, but it has **6** degree of freedom(**3** translational and **3** rotational movements).

#### Key Components of Autonomous Flight
- **State Estimation**: Estimate the **position** and **velocity** (including rotation and angular velocity of the robot), the aim of this process is to **obtain reliable estimates of position and velocity**. For a real quadrotor in a 3-D environment, we must estimate:
    - the displacement of the robot in 3-D
    - the change in orientation of the robot
    - the linear velocity of the robot
    - the angular velocity of the robot  
- **Control**: **Command** motors and **produce** desired actions in order to navigate to desired state
- **Mapping**: Have some basic capability to map its environment. If it does not know what the surrounding environment looks like, then it's incapable of reasoning about this environment and planning safe trajectories in this environment
- **Planning**: Given a set of obstacles and given a destination, the vehicle must be able to compute a trajectory, a safe path to go from one point to another.

#### Supplementary Material
##### [IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit)
Inertial Measurement Unit(IMU) is an electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of [accelerometers](https://en.wikipedia.org/wiki/Accelerometer)(a device that measures proper acceleration) and [gyroscopes](https://en.wikipedia.org/wiki/Gyroscope)(a device used for measuring or maintaining orientation and angular velocity), sometimes also magnetometers.   

An IMU works by detecting **linear acceleration** using one or more accelerometers and **rotational rate** using one or more gyroscopes. Some IMU also include a magnetometer which is used as a **heading reference**. Typical configurations contain **1 accelerometer, 1 gyro, and 1 magnetometer per axis** for each of the three vehicle axes: **pitch, roll and yaw**.

##### [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
Simultaneous Localization And Mapping(SLAM) is often used in robotic mapping and navigation, which is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location with it. While this initially appears to be a chicken-and-egg problem, there are several algorithms known for solving it. Popular approximate solution methods include the **particle filter, extended Kalman filter and GraphSLAM**.

## 1.2 Energetics and System Design
### Basic Mechanics
A quad rotor has 4 rotors that support the vehicle's weight. Each rotor spins and generates the thrust. The diagonal rotors run spin in the same direction.
> Q: Why don't all the rotors of a quadrator spin in the same direction?
> A: Spinning all rotors in the same direction will cause the robot to constantly rotate. 

#### Rotor Physics
![Rotor Physics](./img/w1/RotorPhysics)   

#### Acceleration
![Acceleration](./img/w1/Acceleration)   

### Dynamics and 1-D Linear Control
#### Dynamical Systems - Supplementary Material
> Q: What is **Dynamical System**?   
> A: A dynamical system is a system in which the effects of input actions **DO NOT immediately** affect the system.  

Every dynamcial system is defined by its state.
> Q: What is **state**?   
> A: State is a collection of variables that completely **characterizes** the motion of a system.   
- $$x(t)$$ gives the values of these states over time  

#### Control of height
**GOAL**: Drive the quadrotor to a desired vertical position either up or down!
**Solution**:
- use $$x$$ to measure the vertical displacement
![1000]()   

> Q: What **input** drives the robot to the desired position?   
> A: 

#### Second Order Linear System
![0235]()   

#### [Rates of Convergence](https://www.coursera.org/learn/robotics-flight/lecture/32DXG/supplementary-material-rates-of-convergence) - Supplementary Material
We wanted to determine the appropriate input that will cause the error between the desired state and the actual state of a dynamical system to eventually reach 0. But how **fast** do we want this error to go to 0?   
Throughout this course we will use a **PD or PID** controller as our method for controlling a quadrotor.
![supplementary material: rates of convergence, 0135]()   

### Design Considerations
In reality, the motor thrusts are **limited**, because the motors have a limited capacity. The trust that the motor can produce is limited by the **peak torque**.
![0200]()   

### Agility and Maneuverability

### Component Selection
We have to pay particular attention to the weights of the hardware, because eventually the robot will have to carry these at it flies.

#### Control Architecture
![0115]()   
- low level processor: drive the motors and the propellers;
- high level processor: e.g. Intel to communicates with a lower level processor and commands a low level processor to drive the vehicle;
- (optional) radio controller: in case you want to control of the vehicle;
- sensors: laser scanner, cameras;

### Effects of Size
The platform becomes larger and becomes heavier, therefore **the thrust to weight ratio** changes. The maximum amount of thrust the robot exert, the maximum moment it can generate.

![0300]()    
![0435]()    


# 2. Geometry and Mechanics
## 2.1 Quadrotor Kinematics
### Rigid Body Transformations
#### Reference Frames
We associate with any position and orientation a **reference frame**.
![0123]()    

We can write any vector as a **linear combination** of the basis vectors in either frame.   
$$v = v_1 a_1 + v_2 a_2 + v_3 a_3$$   

![0455]()   
![0808]()   

Rigid body transforms satisfy 2 important properties:
- the map preserves **lengths**;
- **cross products** are preserved by the induced map.

#### Note
Rigid body displacements and rigid body transformations are used **interchangeably*. 

1. **Transformations** generally used to describe relationship between **reference frames** attached to different rigid bodies.
2. **Displacements** describe relationships between **two positions** and **orientation** of a frame attached to a displaced rigid body.

### Rotation
#### Rotation Matrix
![1125]()   

Properties of a Rotation Matrix
- Orthogonal;
- Determinant is +1;
- Closed under multiplication;
- The inverse of a rotation matrix is also a rotation matrix.
![1208]()   
![1529]()   

**Special Orthogonal Matrices SO(n)**
![0130]()   

#### Euler Angles
Euler shows 3 coordinates to describe a general rotation. The key point of Euler angles is that we break the rotation up into **3 successive rotations**. 
![0245]()  

> Q: For every rotation matrix, is there a unique set of Euler angles? / Does this map between the three coordinates, the three Euler angles, and the rotation matrix, is this map one to one?    
> A: NO!   

![0821]()   
![1101]()   

#### Axis/Angle Representation for Rotations
![0341]()

> Q: How to find the rotation matrix for a general axis and angle of rotation? 
![0750]()  
![1025]()
![1403]()

#### Supplementary
##### Eigenvalues and Eigenvectors of Matrices
- Determinant: is a scalar property of square matrices, donated $$det(A)$$ or $$|A|$$.
  - think of rows of an n x n matrix as n vectors in $$R^n$$;
  - the determinant represents the "**space contained**" by these vectors.
- Eigenvector: is a vector associated by a square matrix that **do not** change **in direction** when multiplied by the matrix.
- Eigenvalue: is a scalar value representing how much each eigenvector changes **in length**.
$$Av = \lambda v$$

> Q: How to find eigenvalues?
- 1. Calculate:
$$det(A-\lambda I)$$
- 2. Find solutions to 
$$det(A-\lambda I) = 0$$
**There will be n eigenvalues for an n x n matrix, but not all of them have to be distinct or real values.**
- 3. For each eigenvalue, solve the equation:
$$Av = \lambda v$$
**There will be at least 1 eigenvector for each eigenvalue.** If some eigenvalues are repeated, there might be an infinite number of eigenvectors for that eigenvalue.

##### Skew-Symmetric Matrices and the Hat Operator
The transpose is defined by $$A_ij^T = A_ji$$, the rows and columns of A are "flipped".

Symmetric: 
$$A^T = A$$   
Skew-symmetric:
$$A^T = -A$$   

Therefore, a skew-symmetric matrice is: 
$$A = \begin{bmatrix} 0 & -A_21 & A_13 \\ A_21 & 0 & -A32 \\ -A_13 & A_32 & 0 \end{bmatrix}$$   
A 3x3 skew-symmetric matrix only has **3** independent parameters. We can concisely represent a skew-symmetric matrix as 3x1 vector.   

$$A = \begin{bmatrix} 0 & -A_21 & A_13 \\ A_21 & 0 & -A32 \\ -A_13 & A_32 & 0 \end{bmatrix} \Longrightarrow a = \begin{bmatrix} A_32 \\ A_13 \\ A_21 \end{bmatrix} = \begin{bmatrix} a_1 \\ a_2 \\ a_3 \end{bmatrix}$$   

We use **hat operator** to switch between these two representations.
$$\hat(a) = \begin{bmatrix} \hat(a_1) \\ a_2 \\ a_3 \end{bmatrix} = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}$$   

The hat operator is also used to denote the **cross product** between **two vectors**.   
$$u x v = \hat(u)v$$   

##### Quaternions
Quaternion:   
$$q = (q_0, q_1, q_2, q_3)$$   
which can be interpreted as a **constant + vector**.   

###### Operations with quaternion
![0048]()
- Addition/subtraction
$$p \pm q = (p_0 \pm q_0, \textbf{p} \pm \textbf{q})$$  
- Multiplication
$$pq = (p_0q_0 - \textbf{p}^T\textbf{q}, p_0\textbf{q} + q_0\textbf{p} + \textbf{p} x \textbf{q})$$
- Inverse
$$q^(-1) = (q_0, -\textbf{q})$$  


## 2.2 Quddrotor Dynamics 

