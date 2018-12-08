# CMSC389F: Final Project

## The Problem

The problem involved a setup of RigidBodies - objects with realistic (albeit idealized) physical properties. You have a planet, a set of agents that fly around the planet, and projectiles that the agents launch at each other.

We never got that far - we ended up settling on teaching a RigidBody to hover over a planet in 3 dimensions, that is, subject to linear and angular momentum.

## The Environment
The environment, once again, consists of RigidBodies - 

1. Planet (sphere)
2. Agent (cube)

These have the properties of dimension, starting position, rotation (instantiated using Quaternions as opposed to Euclidean rotation), linear and angular momentum.

The agents has a set a thrusters along each face that it can use to adjust direction.

## The Solution

We initially tried to get this working using Policy Gradient iteration. There were a few complications - we initially tried to modify a hill climbing problem to suit our needs, but this proved to require more effort than it was worth. It was technically challenging to get the setup running.

We finally settled on a Deep Q Network that ended up working quite well in 2 dimensions but not in 3. Challenges included training time, training complexity, the arbitrariness of our network structure (including the graph and the activation functions).

## Conclusion

While we weren't able to achieve our original lofty goals, we still learnt quite a bit. If we were to do things again, we would have picked a much smaller problem in a less complicated environment.
