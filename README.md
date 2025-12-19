# LQR Control for Bipedal Robot Simulation

This repository contains the implementation of an LQR controller for a two-wheeled bipedal robot simulation using MuJoCo. This project demonstrates the linearization of the robot's dynamics and the design of a state-feedback controller to achieve balance and velocity tracking.

## 1. System Description

Based on the derivation from the course materials (Decoupled Balance Subsystem), the system state-space matrices used in this simulation are:

### A Matrix (System Matrix)
State vector $x = [x, \dot{x}, \theta, \dot{\theta}, \delta, \dot{\delta}]^T$
```text
[[ 1      0.002   0      0      0      0     ]
 [ 0      1     -0.0399  0      0      0     ] 
 [ 0      0      1      0.002   0      0     ]
 [ 0      0      0.1207  1      0      0     ]
 [ 0      0      0       0      1      0.002 ]
 [ 0      0      0       0      0      1     ]]

