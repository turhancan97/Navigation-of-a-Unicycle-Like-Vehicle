# Navigation of a Unicycle-Like Vehicle using Dynamic Window Approach

* **Author:** Turhan Can Kargın and Adil Mortepe
* **Topic:** Navigation and Motion Planning in Robotics Course Project

*In this repository, you can find the source codes of the project of navigation and motion planning in robotics, which is a postgraduate course in Automatic Control and Robotics at Poznan University of Technology.*

***Project Video:***

[![Youtube_Cover](https://user-images.githubusercontent.com/22428774/147855837-cef86c97-20e1-46c1-b66c-acfb738e9398.png)](https://www.youtube.com/watch?v=Xq5NMQ2SGNM)

# Table of Contents
   * [Summary](#summary)
   * [How to Run This Project](#how-to-run-this-project)
	   * [Requirements](#requirements)
	   * [Instructions](#instructions)
   * [Introduction](#introduction)
	   * [Navigation and Motion Planning](#navigation-and-motion-planning)
	   * [Classification of motion planning algorithms](#classification-of-motion-planning-algorithms)
		   * [Sampling Based Planning Algorithms](#sampling-based-planning-algorithms)
		   * [Search Based Planning Algorithms](#search-based-planning-algorithms)
		   * [Combinatorial Motion Planning](#combinatorial-motion-planning)
	   * [Two Main Part of the Motion Planning](#two-main-part-of-the-motion-planning)
		   * [Global Motion Planning](#global-motion-planning)
		   * [Local Motion Planning](#local-motion-planning)
			   * [Dynamic Window Approach](#dynamic-window-approach)
			   * [DWA in Robotic Operation System Navigation Stack](#dwa-in-robotic-operation-system-navigation-stack)
			   * [In Short, The Algorithm Behind Dwa](#in-short-the-algorithm-behind-dwa)
* [DWA in Practice](#dwa-in-practice)
	* [Examples](#examples)
* [Methodology](#methodology)
	* [Dynamic Window Approach Simulation](#dynamic-window-approach-simulation)
		* [Simulation Environment](#simulation-environment)
		* [Robot Kinematics](#robot-kinematics)
		* [Updating the Robot Pose](#updating-the-robot-pose)
		* [Generating the Dynamic Window](#generating-the-dynamic-window)
		* [Calculating the Robot Heading](#calculating-the-robot-heading)
		* [Calculating the Distance to Obstacle](#calculating-the-distance-to-obstacle)
		* [Generating Cost Function](#generating-cost-function)
* [Project Presentation](#project-presentation)
* [Future Work](#future-work)
* [References](#references)


## Summary

The Dynamic Window Approach algorithm is a classical local motion planning algorithm used in mobile robot navigation to generate obstacle free trajectories that are feasible based on the motion dynamics of the robot. Trajectory selection is guided by a navigation function, a sum of sub functions that evaluate the Heading direction, Distance to obstacle and Forward velocity of the robot. These sub-functions are each weighted using parameter constants within the interval {0, 1}. Parameter weights play a role in determining the overall success of navigation in terms of reaching the goal point.
A navigation simulation was built in MATLAB to model the motion dynamics of a rigid point in a 2-D Cartesian space coupled with obstacle avoidance based on the Dynamic Window Approach algorithm.

As a result, the main purpose of this project is to simulate the dynamic window approach, which is one of the local motion planning techniques, using MATLAB on a unicycle-like robot. Our second aim is to provide an instructive resource to the users who read and use this repository by briefly explaining the navigation and motion planning methods used in robotics. As a result, we will first talk about the how to run the project, then make short explanations about the motion subjects and finally show how the project looks.

![motion_plan](https://user-images.githubusercontent.com/22428774/147509580-27183e07-a50e-434d-805c-032e1093ab30.PNG)
> Example motion planning figure

## How to Run This Project
### Requirements

1. MATLAB R2021b
2. MATLAB App Designer

### Instructions

1. Go to desired folder
2. Open your command prompt (cmd)
3. copy and paste `git clone https://github.com/turhancan97/Navigation-of-a-Unicycle-Like-Vehicle.git`
4. Open your MATLAB and go to the folder you copied the project files
5. Write `run` to the MATLAB command window
6. You should see a window like figure below

![app](https://user-images.githubusercontent.com/22428774/147792864-f3e1e848-c5bb-4725-8392-769a76d6e09e.PNG)
> Project Interface

Now, you can enter initial and goal location for the robot and start to navigation by pressing start button. 

It should be noted that since the 2D navigation space is 12x12 in size, the entered value must be in the range of 0-12, otherwise the interface will ask us to enter the value again. In addition, if the initial and goal locations are the same, interface again will want us to enter new values. Also, if you want to change the positions of the obstacles in the 2D navigation space, you can open the `obstacle.txt` file and change the position of the obstacles as you want from there.

## Introduction

A robot is a versatile mechanical device - for example, a manipulator arm, a multi-joint multi-fingered hand, a wheeled or legged vehicle, a free-flying platform, or a combination of these - equipped with actuators and sensors under the control of a computing system. It operates in a workspace within the real world. This workspace is populated by physical objects and is subject to the laws of nature. The robot performs tasks by executing motions in the workspace [1].

**Motion planning** is the process of selecting a motion and the corresponding inputs such that to assure that all constraints (obstacle avoidance, risk avoidance, etc.) are satisfied [2]. 

For any mobile device, the ability to **navigate** in its environment is important. Avoiding dangerous situations such as collisions and unsafe conditions (temperature, radiation, exposure to weather, etc.) comes first, but if the robot has a purpose that relates to specific places in the robot environment, it must find those places [3].

In the next section, the concepts of navigation and motion planning in robotics will be explained in more detail.

### Navigation and Motion Planning

Robot localization denotes the robot's ability to establish its own position and orientation within the frame of reference. **Motion planning** is effectively an extension of localisation, in that it requires the determination of the robot's current position and a position of a goal location, both within the same frame of reference or coordinates. Map building can be in the shape of a metric map or any notation describing locations in the robot frame of reference.

**Robot navigation** means the robot's ability to determine its own position in its frame of reference and then to plan a path towards some goal location. In order to navigate in its environment, the robot or any other mobility device requires representation, i.e. a map of the environment and the ability to interpret that representation.

Navigation can be defined as the combination of the three fundamental competences:
* Self-localisation 
* Motion planning 
* Map-building and map interpretation 

Some robot navigation systems use simultaneous localization and mapping to generate 3D reconstructions of their surroundings.

**Motion planning** can be considered as a set of computations which provide sub-goals or set points for the control of the robot. These computations and the resulting plans are based on a suitable model of the robot and the environment in which it is moved. The process by which the robot executes (follows) the planned motion is the control process.

![Control](https://user-images.githubusercontent.com/22428774/147490207-a985bfbb-2159-451a-ba31-2710ae143a15.PNG)
> Motion planning in the control scheme of an autonomous
robot

For example, consider navigating a mobile robot inside a building to a distant waypoint. It should execute this task while avoiding walls and not falling down stairs. A motion planning algorithm would take a description of these tasks as input, and produce the speed and turning commands sent to the robot's wheels. Motion planning algorithms might address robots with a larger number of joints (e.g., industrial manipulators), more complex tasks (e.g. manipulation of objects), different constraints (e.g., a car that can only drive forward), and uncertainty (e.g. imperfect models of the environment or robot) [4].

#### Motion Planning Ingredients

There are several basic ingredients that arise throughout virtually all of the topics covered as part of planning. They are as follow [5]:
* State
* Time
* Action
* Initial and goal states
* Criterion
	* Feasibility
	* Optimality
* A plan

#### Motion Planning Concepts

A basic motion planning problem is to compute a continuous path that connects a start configuration S and a goal configuration G, while avoiding collision with known obstacles. The robot and obstacle geometry is described in a 2D or 3D workspace, while the motion is represented as a path in (possibly higher-dimensional) configuration space.
1. **Configuration space** 
The configuration space, or C-space, of the robot system is the space of all possible configurations of the system. Thus a configuration is simply a point in this abstract configuration space.
![cspace](https://user-images.githubusercontent.com/22428774/147505126-f9af6f50-1e05-4a77-940a-2efd676ac81a.PNG)
> The circular mobile robot configuration space [6].

2. Free space 
The free space or free configuration space is the set of configurations at which the robot does not intersect any obstacle
3. Target space 
Target space is a subspace of free space which denotes where we want the robot to move to. In global motion planning, target space is observable by the robot's sensors. However, in local motion planning, the robot cannot observe the target space in some states. To solve this problem, the robot goes through several virtual target spaces, each of which is located within the observable area (around the robot). A virtual target space is called a sub-goal.
4. Obstacle space
Obstacle space is a space that the robot can not move to. Obstacle space **is not** opposite of free space.

### Classification of motion planning algorithms
#### Sampling Based Planning Algorithms

A sampling-based planning algorithm finds paths by sampling random points in the environment. Sampling Based Algorithms are as follow:

* Rapidly Exploring Random Trees (RRT)
* Probabilistic Roadmaps (PRMs)
* Randomized Potential Fields

#### Search Based Planning Algorithms

The methods presented in this section are just graph search algorithms, but with the understanding that the state transition graph is revealed incrementally through the application of actions, instead of being fully specified in advance. The presentation in this section can therefore be considered as visiting graph search algorithms from a planning perspective. An important requirement for these or any search algorithms is to be systematic. If the graph is finite, this means that the algorithm will visit every reachable state, which enables it to correctly declare in finite time whether or not a solution exists. To be systematic, the algorithm should keep track of states already visited; otherwise, the search may run forever by cycling through the same states. Ensuring that no redundant exploration occurs is sufficient to make the search systematic. If the graph is infinite, then we are willing to tolerate a weaker definition for being systematic. If a solution exists, then the search algorithm still must report it in finite time; however, if a solution does not exist, it is acceptable for the algorithm to search forever. This systematic requirement is achieved by ensuring that, in the limit, as the number of search iterations tends to infinity, every reachable vertex in the graph is explored. Since the number of vertices is assumed to be countable, this must always be possible [5].

**Search-based planning** uses graph search methods to compute paths or trajectories over a discrete representation of the problem. Because a graph is inherently discrete, all graph-search algorithms require this discrete representation. Search-based planning can then be seen as two problems: how to turn the problem into a graph, and how to search the graph to find the best solution.

This is in contrast to sample-based planning methods, which are able to solve problems in continuous space without a graph representation (though certain sampling-based methods do discretize the space). While the pros and cons between the two methods can be debated for many hours, the most apparent difference between the two methods are speed and optimality: graph-search methods can guarantee how "efficient" a solution is (defined more later), but, in general, does so at the expense of computation time. Sampling based planners run fast, but can result in unusual looking and possibly inefficient paths [7]. Search Based Algorithms are as follow:

* Breadth First Search
* Depth First Search 
* Dijkstra’s algorithm
* A-star

#### Combinatorial Motion Planning

Combinatorial approaches to motion planning find paths through the continuous configuration space without resorting to approximations. Due to this property, they are alternatively referred to as exact algorithms. This is in contrast to the sampling-based motion planning algorithms. Combinatorial Motion Planning Algorithms are as follow:

* Vertical cell decomposition
* Shortest-path roadmaps
* Maximum-clearance roadmaps
* Cylindrical algebraic decomposition

### Two Main Part of the Motion Planning

Motion planning techniques, which have a very rich literature depending on kinematic and dynamic constraints, can be divided into two main parts at the most basic level: global and local motion planning methods.
#### Global Motion Planning

The path is created iteratively using a map of the whole environment. This allows you to find the optimal path (assuming a given criterion of optimality). The main limitation is the difficulty to cope with dynamic changes in the environment (e.g. moving obstacles).

The global techniques, such as rrt, road-map, cell decomposition and potential field methods, generally assume that a complete model of the robot's environment is available. The advantage of global approaches lies in the fact that a complete trajectory from the starting point to the target point can be computed off-line. However, global approaches are not appropriate for fast obstacle avoidance. Their strength is global path planning. Additionally, these methods have proven problematic when the global world model is inaccurate, or simply not available, as is typically the case in most populated indoor environments.

A second disadvantage of global methods is their slowness due to the inherent complexity of robot motion planning. This is particularly problematic if the underlying world model changes on-the- fly, because of the resulting need for repeated adjustments of the global plan. In such cases, planning in a global model is usually too expensive to be done repeatedly [8].

#### Local Motion Planning 

The path is created assuming limited knowledge of the environment (e.g. using rangefinder with a limited measurement range). The main advantage of these methods is numerical efficiency, which gives possibility to consider dynamic changes in the environment. The main disadvantage is the difficulty in determining the optimal path (a global solution is not guaranteed).

Local or reactive approaches, on the other hand, use only a small fraction of the world model, to generate robot control. This comes at the obvious disadvantage that they cannot produce optimal solutions. Local approaches are easily trapped in local minima (such as U-shaped obstacle configurations). However, the key advantage of local techniques over global ones lies in their low computational complexity, which is particularly important when the world model is updated frequently based on sensor information [8].

Local motion planning methods are divided into two types in their class: directional and velocity-space methods. Directional approaches calculate the appropriate robot heading angle for barrier-free navigation. Orientation-based approaches calculate the instantaneous orientation angle within the kinematic constraints of the system they are applied to. To achieve this angular position, extra speed control is applied and the navigation problem is handled as a closed-loop control system. Vector field histogram (VFH) and aperture tracking method (FGM) are examples of orientation-based motion planning methods. Velocity space methods take into account the dynamic properties of systems to perform safe navigation. Based on the speed and acceleration limits that can be reached per unit time, they generate linear and angular velocity commands that create partial trajectories suitable for the model. Timed-elastic-band (TEB), lattice scheduler and **Dynamic Window Approach (DWA)** are the most commonly used methods in this class. These methods analyze an optimization function and present the lowest cost speed data as the optimal solution [9].

In the section so far, general to specific explanations have been made on navigation and motion planning, which are general robotics problems. In the next section, the dynamic window approach will be emphasized and the project will be presented.

##### Dynamic Window Approach

A popular local planner is the Dynamic Window Approach (DWA) algorithm which is basically a local motion planning that works well in dynamic environments. In most physical applications of robotic navigation, global and local planning are combined to generate optimal and feasible waypoints for robotic navigation. For example, the popular Robotic Operating System (ROS) used by researchers and hobbyist alike to prototype robotic designs, uses the Dijkstra algorithm as a global planner and the DWA as a local planner in its motion planning architecture.

The DWA is a reactive collision avoidance algorithm that incorporates the geometry and motion dynamics of the robot. As with most other local planners, the goal of this planning algorithm is to generate feasible motion and steering commands in short time intervals that direct the robot to the goal. Based on the kinematic limitations of the robot, a 2-D dimensional velocity search space is created by considering all possible velocity pairs (v, w) that are reachable based on its kinematics. This search space (Vs) is then limited to only select velocities pairs that enable the robot to come to a stop in the vicinity of an obstacle considering the max deceleration of the robot.

These velocities are known as admissible velocities (Va). A velocity pair is considered admissible based on the formula below:

![e1](https://user-images.githubusercontent.com/22428774/147787213-7425a05b-347b-463f-b4c1-b990d71ecd85.png)

Where (v, w) are the linear (v) and angular (w) velocity pairs. Given a velocity pair, the subfunction dist(v, w) generates the distance to the closest obstacle from the robot. Also, given that vb and wb represent the linear and angular accelerations for breakage respectively, based on the expression above, a velocity pair is admissible if the robot can come to a stop before colliding into an obstacle. Once admissible velocity pairs have been established, a dynamic window is created. The dynamic window is a sub velocity search space constituting velocity pairs from Vd that are reachable within the next time interval given acceleration constraints. Velocity pairs are consideredr eachable based on formula below:

![e2](https://user-images.githubusercontent.com/22428774/147787215-4957a713-11f1-4731-a4b7-c617edb64df3.png)

Velocity pairs in the dynamic window Vd depend on selected admissible pairs and time. Based on formula of Vd, sets in Vd are time based, therefore velocities that are reachable given the current time step and current velocity, which constitute the dynamic window. Typically, a set of possible trajectories is generated at every time step using all velocity pairs in the dynamic window. The global velocity search space constitutes the possible velocities according to the specifications of the robot (Vs), admissible velocities (Va) and finally the dynamic window (Vd)

![e3](https://user-images.githubusercontent.com/22428774/147787217-182aca54-6c25-4665-90e6-c1ca5b9de435.png)

![dynamic](https://user-images.githubusercontent.com/22428774/147764707-574ec76c-e733-4581-b5d9-c6494126db0f.PNG)
> Dynamic window

In the space of these possible velocity pairs within the dynamic window that guide the robot through a path, there is an optimal velocity pair that yields the best local trajectory in the current time frame. This optimization problem is guided by a heuristic navigation function below:

![e4](https://user-images.githubusercontent.com/22428774/147787210-5beb47c2-ffad-4be0-a2c0-c894b3a343e0.png)

Recollect that the v, w pairs are selected from the dynamic window (Vd), optimal velocity pairs are those that maximize G(v, w). The sub-function heading(v, w) evaluates the positioning of the robot calculated by determining the angle of the goal point with reference to the current heading direction of the robot. The dist(v, w) evaluates the distance to the closest obstacle on a path and the vel(v, w) expression represents the translational velocity of the robot. In some cases, the velocity(v, w) is used to measure the forward progress of the robot. The weighting constants **alpha, beta** and **gamma** are values between {0, 1} that affect optimization of the navigation function which as a result affects the trajectory of the robot from start position to the target. All 3 sub-functions in the navigation function and their weights contribute to the optimal velocity pair selection which define the trajectory per time step.

![traj](https://user-images.githubusercontent.com/22428774/147765311-b05cd6c5-66ec-4d27-82d5-74f3800630f1.PNG)
> Trajectory roll-out based on v, w pair selection

Assigning higher weights to the distance to obstacle and forward velocity functions results in trajectories that do not move towards the goal. Also, bias towards the angle and velocity means the robot is not incentivized to maintain a safe distance away from obstacles within its path. Moreover, the navigation function can be adjusted to augment the selection of trajectories based on the use case.

##### DWA in Robotic Operation System Navigation Stack

The dwa_local_planner package on ROS provides a controller that drives a mobile base in the plane. This controller serves to connect the path planner to the robot. Using a map, the planner creates a kinematic trajectory for the robot to get from a start to a goal location. Along the way, the planner creates, at least locally around the robot, a value function, represented as a grid map. This value function encodes the costs of traversing through the grid cells. The controller's job is to use this value function to determine dx,dy,dtheta velocities to send to the robot [11]. 

![ros](https://user-images.githubusercontent.com/22428774/147766140-0fb87ae8-b6fc-4ed0-80cb-087719b88990.PNG)
> http://wiki.ros.org/dwa_local_planner

The basic idea of the Dynamic Window Approach (DWA) algorithm is as follows:

1.  Discretely sample in the robot's control space (dx,dy,dtheta)
2.  For each sampled velocity, perform forward simulation from the robot's current state to predict what would happen if the sampled velocity were applied for some (short) period of time.
3.  Evaluate (score) each trajectory resulting from the forward simulation, using a metric that incorporates characteristics such as: proximity to obstacles, proximity to the goal, proximity to the global path, and speed. Discard illegal trajectories (those that collide with obstacles).
4.  Pick the highest-scoring trajectory and send the associated velocity to the mobile base.
5.  Rinse and repeat.

##### In Short, The Algorithm Behind Dwa

![algo](https://user-images.githubusercontent.com/22428774/147766828-593483ce-b871-4598-a16c-33adb31a7058.PNG)
> Different parts of the dynamic window approach [8]

## DWA in Practice

When we look at the usage areas of the dynamic window approach in industry and academia, we come across many articles and papers in the literature. In addition, this method has been developed over the years by engineers in both academia and industry and has been used in different ways in robotics. In the following section, the usage areas and development of this method will be discussed with various examples.

### Examples

1. **An image based dynamic window approach [12]:** The innovative aspect of this method is as follows. Other works using DWA are conceived when the robot and goal positions are known in the world frame, recalling the localization problems previously mentioned. To avoid this problem, the proposed work presents a local navigation strategy for autonomous vehicles in urban environments with an Image Based Dynamic Window Approach (IDWA).

![IDWA](https://user-images.githubusercontent.com/22428774/147849288-1dd5ad11-e2d6-4ff3-bb92-beba4580c0ea.PNG)
> IDWA

2. **High-speed navigation using the global dynamic window approach [10]:** A prime example is an application of the DWA for global obstacle avoidance in which the navigation function selects velocity pairs based on maximization object of velocity, a reward to stay within an optimized global path and a reward for reaching the goal. This framework combines planning and real-time obstacle avoidance algorithms to generate motions for mobile robots that achieve the robot’s task, while securely navigating in an unknown and dynamic environment. The framework is applied to high-speed navigation in unknown environments using a holonomic mobile base.
3. **Dynamic window based approach to mobile robot motion control in the presence of moving obstacles [13]:** Method is based on the integration of focused D* search algorithm and dynamic window local obstacle avoidance algorithm with some adaptations that provide efficient avoidance of moving obstacles.
4. **ND-DWA: A Reactive Method for Collision Avoidance in Troublesome Scenarios [14]:** This paper addresses the reactive collision avoidance for mobile robots that move in troublesome (very dense, cluttered and complex) scenarios. They propose a novel method combining the Nearness Diagram (ND) method and Dynamic Window Approach (DWA).

![ND_DWA](https://user-images.githubusercontent.com/22428774/147850663-816443c0-a779-4c31-b7e4-2e96f4fa79c0.PNG)
> The framework has been implemented and tested on the ATRV-Jr

5. **Predictive Collision Avoidance for the Dynamic Window Approach [15]:** They propose a change in the Dynamic Window Approach—a dynamic collision model—that is capable of predicting future collisions with the environment, even when objects are in motion. Their new version of the Dynamic Window Approach computes future collision points with moving objects. 

![predictive](https://user-images.githubusercontent.com/22428774/147854708-f2a93986-8eec-4225-838f-8617395572ea.PNG)
> The red robot with the number 10 already senses the yellow one approaching and foresightedly picks an evasive trajectory. The DWA is guided by an intermediate target (the smaller red cross) gained from a shortest path shown with blue lines. Our hybrid world model consists of a blurred occupancy map and a polygonal representation of stationary and moving objects.

6. **A Novel Dynamic Window Approach Supported by Posture Control [9]:** In this study, a final posture control method is proposed to the Dynamic Window Approach (DWA) which is frequently used in the motion planning problem for mobile robot systems.
7. **Energy Efficient Dynamic Window Approach for Local Path Planning in Mobile Service Robotics [16]:** An energy efficient local path planner for navigation of omnidirectional battery-powered mobile robots in dynamic environments is presented in this paper. The proposed algorithm extends the Dynamic Window Approach (DWA) to incorporate a cost function based on energy consumption. The estimated energy consumption during planning is predicted using a linear regression model that is learned on the fly. Empirical results are presented on a mobile robot platform that show a 9.79% decrease in energy consumption in comparison to the DWA approach.
8. **A Hybrid Dynamic Window Approach for Collision Avoidance of VTOL UAVs [17]:** This paper describes an obstacle collision avoidance technique that considers the dynamics of a VTOL UAV. The dynamic characteristics and controller structure of VTOL UAVs such as helicopters and tiltrotors vary greatly depending on the flight condition. This paper proposes a hybrid dynamic window approach (DWA) which combines a holonomic DWA, nonholonomic DWA and the flight dynamics of a helicopter to avoid collisions with obstacles. Based on the results of obstacle avoidance simulations, the proposed method not only quickly reaches the target point along the optimal path in various obstacle environments but also is superior to the existing DWA without falling into the local minima by expanding the search space from two dimensions to three dimensions.

![uav](https://user-images.githubusercontent.com/22428774/147855002-2db079c3-7f11-420c-96c8-318d4715d4e5.PNG)
> Figure from the referred project
 
9. **Reinforcement based mobile robot path planning with improved dynamic window approach in unknown environment [18]:** In this paper, an improved DWA based on Q-learning is proposed. First, the original evaluation functions are modified and extended by adding two new evaluation functions to enhance the performance of global navigation. Then, considering the balance of effectiveness and speed, we define the state space, action space and reward function of the adopted Q-learning algorithm for the robot motion planning. After that, the parameters of the proposed DWA are adaptively learned by Q-learning and a trained agent is obtained to adapt to the unknown environment. At last, by a series of comparative simulations, the proposed method shows higher navigation efficiency and successful rate in the complex unknown environment. The proposed method is also validated in experiments based on XQ-4 Pro robot to verify its navigation capability in both static and dynamic environment.

![reinforcement](https://user-images.githubusercontent.com/22428774/147855065-41f18896-f8a2-4656-806d-4f38752144f6.PNG)
> The XQ-4 Pro robot

## Methodology
### Dynamic Window Approach Simulation
#### Simulation Environment

A navigation simulation was developed in MATLAB using static obstacles at different (x, y) coordinate locations as shown below. The objective of the simulation was to evaluate and visualize overall trajectories from robot start point to the goal location. Also, simple app was designed for the simulation by using MATLAB App Designer as shown below.

![app](https://user-images.githubusercontent.com/22428774/147774747-70f92ae3-8723-4e8c-8c82-bd764b812231.PNG)
![sim](https://user-images.githubusercontent.com/22428774/147774749-d906b228-a318-41cc-93e7-8096f7984ce7.PNG)
> App and Simulation Environment

#### Robot Kinematics

The robot used in this simulation is modeled as a rigid point moving in a 2-D co-ordinate frame, as a result, physical variables such as wheel rotation, friction, differential speeds at both wheels were ignored. Since physical constraints were excepted, motion control strategies to maintain stable locomotion in the event of environmental disturbances were also ignored. The robot’s position and orientation was defined by its point location in a 2-D reference frame (x, y) and an angle **phi** . Angle **phi** represents the relative angle of the robot with respect to the global reference frame, as shown in figure below, where Yg, Xg are the global frames and Yl, Xl are the local frames.

![local_global](https://user-images.githubusercontent.com/22428774/147775957-a84c5dde-be25-44ca-837b-812195c1ca3b.PNG)
> Robot Point in 2-D Global Reference Frame.

In the simulation, starting points for the x, y co-ordinate pair was (0, 0) and **phi** = pi/2. In order to yield a near realistic motion model for the robot point, motion variables were defined to magnitudes that constrain motion as shown in table below:

| Kinematic Variables                                              | Magnitude|
| ------------------------------------------------------- | ----- |
| Max Linear Velocity (m/s)             | 1     |
| Max Angular Velocity(rad/s)    | 0.3491     |
| Max Acceleration (m/s^2)         | 0.2     |
| Maximum Angular Acceleration (rad/s^2) | 0.8727     |
| Linear velocity resolution (m/s)              | 0.01     |
| Angular velocity resolution (rad/s)      | 0.0175     |

The Linear and angular velocity resolution are essentially the smallest velocity increments with respect to time. The point movement was simply guided by the linear and angular velocity pairs that maximized the navigation function per time step.

#### Updating the Robot Pose

At any time step increment, the position and orientation (pose) of the robot, identified by (x, y) and **phi** needs to be defined and updated. This update is basically guided by kinematic equations define the linear and angular velocities. The position co-ordinates (x, y) and **phi** are updated using the expression below respectively.

![e5](https://user-images.githubusercontent.com/22428774/147787212-41616c33-2ecc-44b0-9e6d-59be06d5e7cf.png)

Here, (Xi, Yi) and **phi_i** represent the robot’s current position and orientation in the 2-D reference frame, dt represents the time increment and w is the angular velocity .

 A state vector is defined to describe the state of the robot on the basis of the robot current co-ordinate, yaw angle and velocity pairs (v, w). Essentially, this vector is updated per time step as an appropriate velocity pair that maximizes the navigation function is chosen.
 
#### Generating the Dynamic Window

A velocity search space was generated for the robot using motion settings from Table of Kinematic which is above to select safe velocity pairs as described above as the formula of Vd. A vector Vs was defined to store values of the max and minimum linear and angular velocities allowable. Minimum linear and angular velocities in this case are 0 and the negative of the max angular velocity. From this allowable velocity vector, a vector Va is generated that includes the current velocity, time, and resolution limits to generate a search space with only velocity pairs that are reachable within the next time step dt. The dynamic window is generated from this velocity search space vector Va. Furthermore, vectors Va and Vd are regularly updated at every time step as the robot updates its pose.

#### Calculating the Robot Heading

The Heading function keeps the robot facing towards the goal. This sub-function is evaluated by 180-**theta_corr**. The variable **theta_corr** is the angle between the robot local horizontal reference frame and a reference vertical line at 90 degree from the global horizontal reference frame Xg. **theta** is calculated by taking the `arctan` of the difference between the corresponding (x, y) co-ordinate locations of the robot and the goal. A perfect alignment between and the goal point would yield approximately or close to 0 for **theta_corr**. Therefore, keeping this value at a minimum would keep the robot at an orientation facing the goal at every time step. Since the goal of the DWA algorithm is to maximize the overall navigation per time step, if **theta** is large then the heading is penalized, so velocity control actions that minimize the value of **theta_corr** are always desired.

![angle](https://user-images.githubusercontent.com/22428774/147783821-9058a5c6-0ac2-44ff-bf96-ad059c664123.PNG)
> Robot Heading on a 2-D Coordinate.

#### Calculating the Distance to Obstacle

The distance to obstacle function measures the distance to the closest obstacle from the robot within the navigation space. This term is evaluated by calculating the Euclidean norm distance between the current robot coordinate to the coordinate of each obstacle. Since the function runs within a loop, it always calculates distances for every obstacle point, as the loop runs, without taking into consideration if that point is close enough to the robot. To resolve this, a condition is set to only consider euclidean distances that are less than 2 as distances to the closest obstacle, then this variable is used as an output for the navigation function.

#### Generating Cost Function

Generally, cost functions are modeled as error functions between predictions and actual values. In this case, the cost function is modeled using metrics that define the navigation simulation.
## Project Presentation

Simulation of the project (Navigation of a Unicycle-Like Vehicle using Dynamic Window Approach) was performed on MATLAB. As seen below, a simple and plain App has been designed using MATLAB APP Designer for the convenience of those who will perform this simulation.

![app](https://user-images.githubusercontent.com/22428774/147792864-f3e1e848-c5bb-4725-8392-769a76d6e09e.PNG)
> Interface of the project

The interface first asks us to enter **initial and goal location** for the robot. These location will be on x and y axis. Since the size of the 2D Navigation space is 12x12, we need to enter values in the range of 0-12. If a value other than 0-12 is entered, the interface asks us to change the value.  Also, if the initial and goal locations are the same, interface again will want us to enter new values.

After you enter tha initial and goal locations for the robot, then you can press the **start button**. When you press the **start button**, simulation will start as you can see below.

![working_app](https://user-images.githubusercontent.com/22428774/147793297-b38e2a8e-f6e0-4d12-8f2c-8fa3c050e6fe.PNG)

You can pause and continue the simulation whenever you want by pressing **pause** and **continue** **button**. In addition, you can stop the simulation by pressing the **stop button**, even if the robot didn't reach the goal location. 

When the robot reached the goal location, interface of the app will show the plot of trajectory (x, y), linear velocity (v), angular velocity (w), and orientation angle (phi). Besides, you will see the time it takes for the robot to reach the goal on the **Elapsed Time** section. See figure below:

![app_son](https://user-images.githubusercontent.com/22428774/147793750-2550774c-7fe3-4ee0-9380-6bc4b31916ad.PNG)
> Interface after robot reached the goal location

You can see the simulation figure for the robot which has reached the goal location below.  

![complated](https://user-images.githubusercontent.com/22428774/147793945-9acb35f7-907a-4a69-afc9-a3cb790f0c51.PNG)
> When the robot has reach the goal location

![project_gif](https://user-images.githubusercontent.com/22428774/147794331-fec7ca52-92e3-48a9-b3a0-276d72a2820c.gif)
> Simulation

## Future Work

As can be seen, within the scope of this project, the dynamic window approach was simulated on a mobile robot using MATLAB and APP Designer. In the MATLAB based simulation used to demonstrate obstacle avoidance, the robot kinematics was a simple kinematic model of a point moving in a 2-D Cartesian co-ordinate. Therefore, more realistic parameters such as wheel rotation and differential speeds were not factored into the forward and inverse kinematics calculation. Also, static obstacles were used to demonstrate obstacle avoidance. Some future ideas for the development of this project will be discussed in this section. 

Realistically, navigation environments are not static, even in indoor environments. Autonomous mobile robots in real world scenarios encounter moving objects, clustered locations and in some cases unexpected jerking from external forces. For example, simulating the 2D navigation space by adding moving obstacles with MATLAB is a planned idea in the future.

Also, the parameter of the objective functions (**alpha, beta, gamma**) can be selected by usıng bayesian optimizatıon. Running training modules for Bayesian optimization with a robot simulation that incorporates a more realistic dynamics and environment will more than likely yield optimal parameter sets that work in a host of environments. In addition to utilizing more realistic environments, external robot simulation environments such as Gazebo, Unity and Unreal Engine can be used. These simulators have a wide availability of robots with their respective kinematic models for navigation testing. So there is no need to generate kinematic equations or physical models of the environment. This can be second idea of the future work. 

Finally, the last idea that can be done in the future to improve this project is to add the image based dynamic window approach to the existing project.

![IDWA_2](https://user-images.githubusercontent.com/22428774/147855525-914948fe-1252-421d-98b6-a3d1bdefb039.PNG)
> Block Diagram of image based dynamic window


## References

1. Jean-Claude Latombe, Robot Motion Planning, Springer 1991
2. Dariusz Pazderski, Navigation and motion planning Lecture Notes
3. https://en.wikipedia.org/wiki/Robot_navigation 
4. https://en.wikipedia.org/wiki/Motion_planning
5. S. Lavalle, Planning algorithms, Cambridge University Press, 2006
6. R. C. Arkin (editor), Principles of Robot Motion Theory, Algorithms and Implementation, Massachussets Institute of Technology (MIT), 2005.
7. Search-Based Planning Lab, http://sbpl.net/
8. D. Fox, W. Burgard and S. Thrun, "The dynamic window approach to collision avoidance," in _IEEE Robotics & Automation Magazine_, vol. 4, no. 1, pp. 23-33, March 1997, doi: 10.1109/100.580977.
9. Karakaya, S. & Ocak, H. (2020). Açısal Duruş Kontrolü Destekli Özgün bir Dinamik Pencere Yaklaşımı . Bilecik Şeyh Edebali Üniversitesi Fen Bilimleri Dergisi , 7 (1) , 184-200 . DOI: 10.35193/bseufbd.705765
10. Oliver Brock and Oussama Khatib. High-speed navigation using the global dynamic window approach. In Proceedings 1999 IEEE International Conference on Robotics and Automation (Cat. No. 99CH36288C), volume 1, pages 341–346. IEEE, 1999.
11. http://wiki.ros.org/dwa_local_planner
12. Danilo Alves de Lima, Alessandro Corrêa Victorino. An image based dynamic window approach for local navigation of an autonomous vehicle in urban environments. IEEE ICRA Workshop on Modelling, Estimation, Perception and Control of All Terrain Mobile Robots (WMEPC 2014), May 2014, Hong Kong, Hong Kong SAR China.
13. M. Seder and I. Petrovic, "Dynamic window based approach to mobile robot motion control in the presence of moving obstacles," Proceedings 2007 IEEE International Conference on Robotics and Automation, 2007, pp. 1986-1991, doi: 10.1109/ROBOT.2007.363613.
14. Guoyang Li, Genxia Wu and Wei Wei, "ND-DWA: A Reactive Method for Collision Avoidance in Troublesome Scenarios," 2006 6th World Congress on Intelligent Control and Automation, 2006, pp. 9307-9311, doi: 10.1109/WCICA.2006.1713802.
15. Marcell Missura and Maren Bennewitz "Predictive Collision Avoidance for the Dynamic Window Approach," 2019 International Conference on Robotics and Automation (ICRA), DOI: 10.1109/ICRA.2019.8794386
16. Henkel C, Bubeck A, Xu W. Energy "Efficient Dynamic Window Approach for Local Path Planning in Mobile Service Robotics," 2016, doi:10.1016/j.ifacol.2016.07.610 ‌
