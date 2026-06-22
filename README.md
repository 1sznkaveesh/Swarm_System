# Swarm_System

This project simulates a swarm of autonomous robots that cooperatively transport boxes to predefined goal locations. Each robot is assigned a task based on proximity, computes an optimal box-pushing path using a space-time A* search algorithm, and dynamically avoids collisions with other agents using a global reservation matrix. 

To maximize system throughput, bots prioritize active delivery runs without halting mid-route. Once a block is dropped, the bot evaluates the global timeline to check if its parking spot blocks an upcoming path. If a conflict is detected, it reactively moves the minimum distance necessary to clear the lane; otherwise, it stays parked to conserve energy.

## How to Run

```bash
py main.py
```
### For Demonstration
https://youtu.be/0U6KzOHRwBk
