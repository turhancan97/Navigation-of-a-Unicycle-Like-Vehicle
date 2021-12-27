# Navigation of a Unicycle-Like Vehicle using Dynamic Window Approach
* **Author:** Turhan Can Kargın
* **Topic:** Navigation and Motion Planning in Robotics Course Project

*In this repository, you can find the source codes of the project of navigation and motion planning in robotics, which is a postgraduate course in Automatic Control and Robotics at Poznan University of Technology.*
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
			   * [Algorithm Behind DWA](#algorithm-behind-dwa)
* [Project Presentation](#project-presentation)
* [Future Work](#future-work)
* [References](#references)


## Summary
The main purpose of this project is to simulate the dynamic window approach, which is one of the local motion planning techniques, using MATLAB on a unicycle-like robot. Our second aim is to provide an instructive resource to the users who read and use this repository by briefly explaining the navigation and motion planning methods used in robotics. As a result, we will first talk about the how to run the project, then make short explanations about the motion subjects and finally show how the project looks.
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
![app](https://user-images.githubusercontent.com/22428774/147417177-7a193ce6-8522-48ed-a5ac-538fd84af2d4.PNG)
> Project Interface

Now, you can enter initial and goal location for the robot and start to navigation by pressing start button. 
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

Local motion planning methods are divided into two types in their class: directional and velocity-space methods. Directional approaches calculate the appropriate robot heading angle for barrier-free navigation. Orientation-based approaches calculate the instantaneous orientation angle within the kinematic constraints of the system they are applied to. To achieve this angular position, extra speed control is applied and the navigation problem is handled as a closed-loop control system. Vector field histogram (VFH) and aperture tracking method (FGM) are examples of orientation-based motion planning methods. Velocity space methods take into account the dynamic properties of systems to perform safe navigation. Based on the speed and acceleration limits that can be reached per unit time, they generate linear and angular velocity commands that create partial trajectories suitable for the model. Timed-elastic-band (TEB), lattice scheduler [10] and **Dynamic Window Approach (DWA)** are the most commonly used methods in this class. These methods analyze an optimization function and present the lowest cost speed data as the optimal solution [9].

In the section so far, general to specific explanations have been made on navigation and motion planning, which are general robotics problems. In the next section, the dynamic window approach will be emphasized and the project will be presented.

##### Dynamic Window Approach
##### Algorithm Behind DWA
##### DWA in Practice
## Project Presentation
## Future Work
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
