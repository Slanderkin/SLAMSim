# SLAMSim

**Windows Executable Folder contains an Executable file along with the required .dll files for use without compilation.**

##How to Use
Use left click to spawn cylinders, WAD to move (no S), R to toggle lidar lines being drawn, F to toggle world drawing, and G to toggle gaussian noise. 

## About FastSLAM 1.0
### Particle Filter Explained

A Particle Filtered SLAM algorithm utilizies multiple individual entities called particles that all are guesses for where the robot is. Each particle has its own position and heading, along with corresponding uncertainties for both. Each time the robot move, its control inputs are fed to every particle. Each particle in turn adds minor Guassian noise to this input and then move accordingly. Now that the particle is in a new position, it estimates the liklihood that each landmark it sees corresponds to a landmark seen by the robot using a Kalman filter (for each landmark). The sum of these liklihoods leads to the particle's overall weight. After all particles are given a weight, they are randomly sampled, particles with higher weights are more likely to be picked to participate in the next movement. The particle closest to the mean of all of the particles is chosen to represent the filter's estimation for the robot's position.
