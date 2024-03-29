# BayesDesOpt

## Description:

BayesDesOpt is a framework to automate robot design, based on Bayesian Optimization.
The purpose is to facilitate the process of finding optimal design parameters for robots.

This work assumes that a robot structure is defined by DH parameters | *d* | *θ* | *a* | *α* | and
the user can choose among many different parameters to optimize for such as:
- link lenghts *a*;
- link offsets *d*;
- joint angular offsets *θ*, *α*;
- joint types (prismatic or revolute).

Currently, the framework deals only with serial link manipulators. Yet, any robotic structure can be added by modifying
the forward kinematics method in `Robot_class.m` .

## How to run BayesDesOpt
The folder *examples/* contains `BayesdesOpt.m` which is the main function to run the design optimizaton problem.
It loads a configuration file in .xml format where the user can set the variables to optimize for.
Some exemplary configuration files can be found *examples/Data*.
`BayesdesOpt.m` calls `readSolverConfig.m`, which loads and parses the configuration file, and `setOptimizationvariables.m` to set the variables to optimize for, based on the user's choice.

Different option for Baysian Optimization can be chosen by the user directly in `BayesdesOpt.m`.

## Cost Function

The framework is meant to find the optimal parameters that maximize the volume of the dexterous workspace (WS).

At the moment, three different cases are provided:
- **Single Arm Optimization** : the parameters of one single robotic arm are optimized. The goal is maximize the volume of its dexterous WS.
- **Copy Dual Arms Optimization**: the parameters of a dual arm configuration are optimized. In this scenario, the two arms are identical. 
The optimization variables include the disnace of the bases of the two arms and the goal is to maximize the common intersecting dexterous WS.
- **Dual Arms Optimization**: similar to the previous one, but in this case the two arms can have completely different structures.

In the optimization of the dual arms we also take into account a **Safety Measure** that penalizes those designs for which the distance between the two robots is too small
and may lead to high risk of collisions between the two arms.

## Greetings

We thank you for your interest and for any comment or question please contact *cursifrancesco@gmail.com*.
