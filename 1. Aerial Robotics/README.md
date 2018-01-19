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
- **State Estimation**: Estimate the **position** and **velocity** (including rotation and angular velocity of the robot)
- **Control**: **Command** motors and **produce** desired actions in order to navigate to desired state
- **Mapping**: Have some basic capability to map its environment. If it does not know what the surrounding environment looks like, then it's incapable of reasoning about this environment and planning safe trajectories in this environment
- **Planning**: Given a set of obstacles and given a destination, the vehicle must be able to compute a trajectory, a safe path to go from one point to another.