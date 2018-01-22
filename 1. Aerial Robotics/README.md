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

#### Supplement
##### [IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit)
Inertial Measurement Unit(IMU) is an electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of [accelerometers](https://en.wikipedia.org/wiki/Accelerometer)(a device that measures proper acceleration) and [gyroscopes](https://en.wikipedia.org/wiki/Gyroscope)(a device used for measuring or maintaining orientation and angular velocity), sometimes also magnetometers.   

An IMU works by detecting **linear acceleration** using one or more accelerometers and **rotational rate** using one or more gyroscopes. Some IMU also include a magnetometer which is used as a **heading reference**. Typical configurations contain **1 accelerometer, 1 gyro, and 1 magnetometer per axis** for each of the three vehicle axes: **pitch, roll and yaw**.

##### [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
Simultaneous Localization And Mapping(SLAM) is often used in robotic mapping and navigation, which is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location with it. While this initially appears to be a chicken-and-egg problem, there are several algorithms known for solving it. Popular approximate solution methods include the **particle filter, extended Kalman filter and GraphSLAM**.

## 1.2 Energetics and System Design
### Basic Mechanics
A quad rotor has 4 rotors that support the vehicle's weight. Each rotor spins and generates the thrust. The diagonal rotors run spin in the same direction.
> Why don't all the rotors of a quadrator spin in the same direction?
> Spinning all rotors in the same direction will cause the robot to constantly rotate. 

### Control

### Design considerations

### Agility

### Component selection

### Effects of size