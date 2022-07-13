# PedDynSim
A simulation of pedestrian dynamics in an intersecting corridor environment with different values of flux and pedestrian group size, using a Headed Social Force Model (HSFM).

## Objective
Simulate the pedestrian flow in  the following geometry representing two intersecting corridors with walls, with a constant flux in the AB and CD directions. Representation of individual pedestrians in simulation is necessary. Perform the simulation with at least four different pedestrian flux values in the range $\phi = 0.2 - 2$ pedestrians/second, taking into account the desired target path/exit and velocity, as well as the avoidance of other pedestrians.

![Simulation domain]([Dragster.jpg](https://user-images.githubusercontent.com/61327278/93505954-da4af480-f91b-11ea-87c6-8f362ee1c250.png))


## Assumptions
For the representation of the pedestrians, assume that:
* They have a circular cross-section with radius r=0.25m
* They have a desired velocity in the range of 1.1-1.3 m/sec
* Their maximum acceleration is 2 m/sec^2
* Integrate the motion of the pedestrians for 60 sec

## Model
A Social Force Model (SFM) is used to simulate the pedestrian flow. The main assumptions of a SFM is that the systematic temporal changes of a pedestrian's preferred velocity and path can be described as a sum of `social forces' that represents the effect of the environment (other pedestrians and obstacles) on their behaviour. As such, the pedestrian's trajectory can be modelled using a system of nonlinear ordinary differential equations. 
