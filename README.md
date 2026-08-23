# 🚀 Welcome to the Intro-ROS2 Learning [Wiki](https://github.com/VSSUT-Robotics-Society/Intro-ROS2/wiki)!

This is a short, visual, hands-on guide designed for undergraduates with basic Arduino and Python exposure who are new to Linux and ROS2.

## Overview

This [wiki](https://github.com/VSSUT-Robotics-Society/Intro-ROS2/wiki) uses a single sample project (a motorized inverted pendulum) to teach the typical robot development flow with ROS2, why teams move from microcontroller-only designs to ROS, and what the extra pieces (simulation, launch systems, message types, tooling) actually solve.

Quick snapshot
---
- **Audience:** Undergraduates familiar with basic Python and embedded C/Cpp.
- **Duration:** 3 days (bare-minimum, hands-on track)
- **Language:** Python-first examples (C++ notes where needed)
- **Distros:** examples work on Ubuntu 24.04 / ROS2 (Jazzy) and are compatible with Ubuntu 22.04 / ROS2 Humble where noted.
- **Simulation:** Gazebo (Harmonic) included for visual learning.

Steps to Follow
---
- [Overview](Overview.md): Why ROS2, architecture, and the big picture
- [Concepts](Concepts.md): Nodes, topics, services, actions, packages
- [Setup](Setup.md): Links to installers, quick Linux/WSL command notes
- [PendulumProject](PendulumProject.md): the sample project walkthrough
- [Exercises](Exercises.md): Short practical tasks and verification steps
- [Troubleshooting](Troubleshooting.md): Common issues and fixes
- [Resources](Resources.md): Official docs, tutorials and GIFs

## How We'll Use 3 Days

- **Day 1** - Install ROS2, learn basics, run simple talker/listener, learn CLI
- **Day 2** - Run the pendulum simulation in Gazebo, inspect topics/services
- **Day 3** - Implement a simple controller (PID), visualize and record a GIF

## Visual Learners

Each page suggests short screencasts or GIFs to illustrate the commands and simulator interactions.

## Next Step

Open the **[Overview](Overview.md)** to get the big picture!