# Robot Localization
### AJ Evans, Jack Levitsky, Ben Morris


## About
The goal of this project was to create a particle filter that localizes a NEATO in a mapped environment given a guess at the initial position.
To figure out where it was within the environment, the NEATO first creates a cloud of particles (x, y, and orientation) near the initial guess. Whenever the NEATO receives a LiDAR scan, it centers it on each particle. For each point in the scan for each particle, the NEATO calculates the distance between that point and the nearest obstacle. Once it has those distances, it sums them for each angle in the scan. It then assigns a “weight” to each particle, proportional to the inverse of this sum of distances (lower distances correspond to a better fit and therefore a higher weight). Finally, it creates a new particle cloud based on the old particle cloud and its weights. Each particle is allowed to clone itself a number of times proportional to its weight. Each clone is randomly sampled from a normal distribution centered on the original particle, with a standard deviation proportional to the inverse of the weight. These clones are coalesced into a new particle cloud, and the process repeats. The guess of the robot's pose at any given time is given by the particle with the highest weight.


## Design
We attempted to create our own version of the `ParticleFilter` node to explore how to make more complicated ROS nodes on our own. We tried to implement it in a more straightforward manner, with only two subscriptions and a publisher (no threading or timers). However, over time we realized that this approach was unfeasible with the time granted to us. The starter code influenced us more and more as we progressed until we decided to adopt it completely.


## Challenges
We faced more than a few challenges throughout this project. In attempting to create a new original particle filter node, we ran into our own unique set of issues that had to be solved by Paul himself. After we were given a good starting point, we began to make slow but consistent progress. Smaller issues persisted but were solved either by talking with a CA or giving ourselves a bit of time to recover before attempting the problem again.


## Future Work
We managed to get the particle filter working in time but feel as though there are definitely areas we could have explored further. Some parameters, such as the standard deviation of the particles, could be tuned more finely. In addition to that, it would be worth testing if our particle filter works in environments other than the gauntlet challenge. Then, of course, with a lot more time and resources we would attempt to finish our own particle filter. Ultimately, our stretch goal would be to tackle the NEATO kidnapping problem and be able to localize in any environment our NEATO was dropped into.


## Lessons for the Future
In future projects, we will spend more time with any supporting code that we know works. While splitting off from the beaten path allowed for a deeper dive into the creation of ROS nodes like this, it alienated us from a lot of our potential sources of aid. Paul Ruvolo was the only one who had done this before, meaning that the CA team was less able to help with some of the more complex issues we faced.
