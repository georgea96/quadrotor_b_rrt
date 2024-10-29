
# Group 31: Quadrotor Path Planning using B-Informed RRT

## Overview

This repository is part of a group project done by Georgios Apostolides, Markos Gkontzaris, Kevin Voogd, and Guru Deep Singh as part of the Planning and Decision Making Course (RO47005) at TU Delft. The project focuses on developing a path-planning algorithm for a quadrotor using the **Bi-directional Rapidly-exploring Random Tree (B-RRT) approach**. B-RRT optimizes the list of vertices sampled to provide a locally optimized trajectory, which is faster and computationally efficient compared to traditional RRT. Our algorithm was implemented as an extension of RRT and can potentially be expanded into more advanced path-planning algorithms. The planned trajectory is smoothened using minimum snap, and the collision-free paths were simulated in MATLAB while accounting for the dynamics of a Crazyflie 2.0 quadrotor. This project builds on an existing project by Yiren Lu, Myles Cao, Wudao Luo, and Xuanyu Zhao, available [[here]](https://github.com/yrlu/quadrotor). All credit for the foundational code goes to the original authors of that repository.

**Example Results from Different Maps:**

| B-Informed RRT                                 | RRT-Algorithm                               |
|------------------------------------------------|---------------------------------------------|
| <img src="https://github.com/georgea96/quadrotor_b_rrt/blob/main/readme_media/Proposed_B_RRT.gif" alt="Proposed RRT" width="300" /> | <img src="https://github.com/georgea96/quadrotor_b_rrt/blob/main/readme_media/Vanila_RRT_Algorithm.gif" alt="Vanilla RRT" width="300" /> |
| <img src="https://github.com/georgea96/quadrotor_b_rrt/blob/main/readme_media/map3_rrt_opt.png" alt="Proposed RRT" width="300" /> | <img src="https://github.com/georgea96/quadrotor_b_rrt/blob/main/readme_media/map3_rrt.png" alt="Vanilla RRT" width="300" /> |

## Functionality

The project involves modeling a Crazyflie 2.0 quadrotor, equipped with position and attitude controllers that enable it to follow a trajectory calculated with the B-RRT algorithm. The objective is to plan a collision-free path from a starting position outside a "room on fire" to a goal location near a human in need of assistance, where the quadrotor could theoretically deliver necessary tools or communication devices. The B-RRT algorithm guides the quadrotor's path through a 3D space filled with static obstacles, utilizing bidirectional trees to efficiently compute a feasible path.

## Features

- **Bi-directional RRT (B-RRT) Path Planning**: Uses two growing RRT trees to connect the start and goal points, reducing computation time and providing locally optimized paths.
- **Collision-Free Path Planning**: Ensures a feasible path is computed within the constraint of iterations, with obstacle avoidance.
- **Path Simplification and Optimization**: Utilizes a simplification algorithm to reduce the overall path distance, providing smoother and shorter trajectories.
- **MATLAB Simulation**: The quadrotor's trajectory is visualized in a 3D environment with dynamic controls and optimized paths simulated in real time.

## Project Structure

- **/code**: Contains MATLAB scripts for B-RRT implementation, path optimization, and simulation of the Crazyflie 2.0 quadrotor.
- **/results**: Holds result images and plots, including comparative performance metrics between B-RRT and traditional RRT.
- **/docs**: Documentation on algorithm pseudo-code, quadrotor dynamics, and project insights.
- **README.md**: Main file detailing setup, installation, and usage instructions.

## How to Use

### Installation Instructions

1. Clone the repository.
2. Ensure MATLAB is installed and has necessary toolboxes, such as the Robotics System Toolbox.
3. Install dependencies listed in `requirements.txt` if applicable.

### Running Instructions

1. Choose algorithm in file `traj_planning/test_sequence.m` and run.
2. On file `test_sequence.m` uncomment accordingly one of the five functions below.
3. Note that the first function is the RRT of the report, while the fifth function is the proposed RRT of the report.

```bash
    path{qn}=rrt_function(map,start,stop,num_iter);    
    path{qn}=rrt_goal_on_view(map,start,stop,num_iter);
    path{qn}=bi_directional_rrt_function(map,start,stop,num_iter);    
    path{qn}=bi_directional_rrt_function_on_view(map,start,stop,num_iter); 	path{qn}=bi_directional_rrt_function_optimized(map,start,stop,num_iter);
```
From these functions, one can navigate accordingly to the created functions such as `back_tracking`, `path_collision_checker` `plot_path_rrt`.

Furthermore, the maps created are saved in folder traj_planning/maps.

The main room map is saved as `map_ours_11`, while the second map mentioned on the report is `map3`.

In folder `traj_planning/maps` there exist other maps with different types of obstacles for the sake of more tries if necessary.
  

The files that were written by us are:
- `path_collision_checker.m `
- `bi_directional_rrt_function.m`
- `bi_directional_rrt_function_on_view.m`
- `bi_directional_rrt_function_optimized.m `
- `rrt_function.m`
- `rrt_goal_on_view.m back_tracking.m`

## Informed B-RRT Results

The B-RRT algorithm was tested in various simulated environments, including a room with fire as a static obstacle and additional obstacles with convex polyhedral shapes. Our results compared B-RRT and traditional RRT in terms of runtime and iterations, revealing that B-RRT converged faster and produced shorter, smoother paths. A thousand trials on an Intel 8550U processor confirmed significant reductions in computation time with B-RRT.

### Key Observations:

- **Convergence Speed**: B-RRT consistently converged faster, even with reduced maximum iterations (e.g., 300), where RRT often failed to converge.
- **Path Quality**: The generated paths were both collision-free and simplified for minimal distance, demonstrating B-RRT’s efficiency in environments with varied obstacle density.
- **Future Potential**: B-RRT can be further improved to B-RRT*, aiming for global optimality similar to RRT* but with the enhanced efficiency of bidirectional search.

These results underscore the robustness of B-RRT for path-planning tasks in environments with static obstacles, providing an effective foundation for future applications in dynamic or more complex environments.






  



  





