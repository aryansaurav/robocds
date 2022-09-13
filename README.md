# robocds: Learning to coordinate fingers motion with arm's motion while approaching an object for grasping

coupled dynamics of hand as master, with orientation and fingers as slaves

This repositority is written mostly in C++ to work on ROS platform (Robot Operating System) and contains a sample implementation of work presented in the paper:

Coupled dynamical system based arm–hand grasping model for learning fast adaptation strategies
https://www.sciencedirect.com/science/article/pii/S0921889011001576?casa_token=kjWyVCrdo1MAAAAA:OsVax6P5zMB5aYBk4E6x_3rtAENUv5XyhlqAcUlevjeX-EcSdafpu_IcPEmTzyfhssYlAF5g1sBSRA

# Summary 

There are two robots used here, KUKA arm with a humanoid hand (Allegro) mounted on it. This repository contains algorithms to control the KUKA arm's motion and Allegro hand's fingers motions simultaneously in conjunction with each other while approaching an object for grasping. The robotic hand's fingers moves in coordination with the movement of the KUKA arm. As the arm brings the hand closer to the object, the hand's fingers open up to encircle the object and eventually close around it. 

The coordination strategy between the arm's motion and the hand's fingers were learnt from the real human hand and finger motions recorded using Motion Capture system and cyberglove. Classical Machine Learning Approach such as GMM were employed here.
