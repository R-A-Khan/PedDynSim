# PedDynSim
A simulation of pedestrian dynamics in an intersecting corridor environment with different values of flux and pedestrian group size, using a Headed Social Force Model (HSFM).

## Objective
Simulate the pedestrian flow in  the following geometry representing two intersecting corridors with walls, with a constant flux in the AB and CD directions. Representation of individual pedestrians in simulation is necessary. Perform the simulation with at least four different pedestrian flux values in the range $\phi = 0.2 - 2$ pedestrians/second, taking into account the desired target path/exit and velocity, as well as the avoidance of other pedestrians.

![Simulation domain](domain.png)


## Assumptions
For the representation of the pedestrians, assume that:
* They have a circular cross-section with radius r=0.25m
* They have a desired velocity in the range of 1.1-1.3 m/sec
* Their maximum acceleration is 2 m/sec^2
* Integrate the motion of the pedestrians for 60 sec

## Model
A Social Force Model (SFM) is used to simulate the pedestrian flow. The main assumptions of SFMs are that the temporal changes of a pedestrian's preferred velocity and path can be described as a sum of 'social forces' that represents the effect of the environment (other pedestrians and obstacles) on their behaviour. As such, the pedestrian's trajectory can be modelled using a system of nonlinear ordinary differential equations (Helbing & Molnar, 1998). For this simulation, we implement the Headed Social Force Model (HSFM) developed by Farina et. al. (2017). In a traditional SFM, a pedestrian is able to move in any direction at all times. In the HSFM the pedestrians' velocity vectors are aligned with their heading, based on empirical evidence that humans have a tendency to move forward. The HSFM also introduces an additional force in order to reproduce the behavior of people intentionally walking together as a single group (e.g., friends or colleagues), which is designed to drive individuals back into the group region whenever they leave it. 

## Numerical Simulation
The numerical implementation is adapted for the current geometry from the methodology and algorithm developed by Farina et. al. (2017). To simulate a constant flux of pedestrians in the ABCD region, pedestrians are initialised at $t=0$ at equidistant points outside the regions A and C, such that the desired flux is maintained as they enter and leave the corridors. Values of $\phi \in [0.2, 0.5, 1, 2]$ are used. The simulations are conducted for two scenarios:
* Individual pedestrians entering the region with flux $\phi$.
* Groups entering the region with group flux $\phi$.

### Individuals

table with case numbers 


video gifs

* Random initial point in the x-direction in the corridor
* they start to form a single file
* the spacing as they enter the corridor is not fixed
* The waypoint sequence can have a negative effect
* The headed force can prevent them from moving back


### Groups

table with case numbers 


video gifs

* Similar issues with groups
* Let it run longer, see the group force preventing others from leaving the corridor


### Sensitivity to Group Force Parameters

### Performance indiciators

