
# Seminar Autonomous Driving (**P**raktikum **A**utonomes **F**ahren)

## About
This project is a competition-like challenge in the course Seminar Autonomous Driving
at the University of Augsburg.

Two teams of students (6-7 team members each) are creating their own self-driving car
software interacting with the CARLA simulation environment. At the end of the project,
there will be a live demo including a city and a racing mode where the cars of both
teams compete against each other.

The live demo will include the interaction with other cars / pedestrians, traffic lights.
Moreover there will be driving tests on a variety of maps to ensure that the self-driving
is not bound to a single map that was learnt by heart.

## Team Members
- Berghoff, Joshua
- Böll, Axel
- Mospan, Pavlo
- Stoljar, Johannes
- Sturm, Daniel
- Tröster, Marco

## Quick Start
0. Before you start, make sure that your PC runs a NVIDIA GPU that supports nvidia-docker.
1. First, set up your PC according to this
   [tutorial](https://github.com/ll7/paf21-1/wiki/Development-Machine-Setup-(NVIDIA-Docker)).
   It assists you at installing docker with GPU support.
2. Clone this GitHub repository, e.g. use '*git clone https://github.com/ll7/paf21-1*'
3. Next, build all components required to run self-driving car scenarios, see [docs](./components/README.md).
4. Finally, go to one of the scenarios (e.g. the [default scenario](./scenarios/default_scenario)
   and launch it with docker-compose.

## Repository Structure
TODO: add some information on where the different parts of the project are located at
and what they are supposed to be doing

## Components
TODO: add some information on the data flow between system components and the architecture

## AI Training
TODO: add some information on how to train the AI components

## License
TODO: think of an appropriate license, e.g. MIT license
