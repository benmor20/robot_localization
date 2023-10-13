# Robot Localization
### AJ Evans, Jack Levitsky, Ben Morris


## About
The goal of this project was to use a particle filter to localize a NEATO in a mapped environment.
To figure out where it was within the environment, the NEATO would estimate where it was currently. Then it would take a LiDAR scan of its surroundings and create a cloud of particles. Once the particles were verified to be inside the mapped area, a theoretical "scan" was taken at the pose of each particle. The difference between the theoretical and actual scan distances is calculated for every angle and summed up. The sums are then compared, the weight of each particle is inversely proportional to the sum of the differences.

How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
What if any challenges did you face along the way?
What would you do to improve your project if you had more time?
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
