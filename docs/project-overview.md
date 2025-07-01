# Explorer Bot Project 


## Project Objectives
Based on the project overview, we have two main tasks:
- **Single robot task**: Explore and map an unknown environment in the fastest time
- **Multi-robot adversarial task**: Be the first to register the tag on the back of an opponent robot

## Phase 1: Simulation Environment

### Setup Requirements
1. **Gazebo Harmonic** simulation environment
   - Create simulated world with obstacles and boundaries
   - Simulate TurtleBot3 Waffle Pi model with sensors
   - Configure camera simulation for tag detection

2. **ROS 2 Packages**
   - Navigation stack integration (Nav2)
   - SLAM for mapping
   - Camera integration
   - Tag detection algorithms

### Components to Develop

#### 1. Core Navigation System
- **SLAM Node**:
  - Interface with turtlebot3_navigation for autonomous mapping
  - Store and update map data
  - Report exploration coverage

- **Waypoint Navigation**:
  - Interface with Nav2
  - Path planning and obstacle avoidance
  - Dynamic waypoint generation for exploration

#### 2. Exploration Strategy
- **Frontier Detection**:
  - Identify unexplored areas
  - Prioritize frontiers based on information gain

- **Decision Making**:
  - Behavior Tree implementation for autonomous decisions
  - Balance between exploration and target pursuit (for adversarial task)

#### 3. Adversarial Components
- **Tag Detection System**:
  - Image processing pipeline
  - Tag recognition algorithms
  - Position estimation

- **Pursuit Strategy**:
  - Target tracking
  - Intercept path planning

## Development Stages

### Stage 1: Simulation Setup and Teleoperation
- [ ] Setup Gazebo Harmonic with TurtleBot3 Waffle Pi model
- [ ] Use a standard world from the Turtlebot package
- [ ] Implement teleoperation using rqt_robot_steering
- [ ] Create a TwistStamper node to convert Twist to TwistStamped messages

**Test Suite:**
- [ ] Test that Gazebo world has successfully spawned (check topics/services)
- [ ] Test that TurtleBot3 model has spawned correctly (verify model state)
- [ ] Test that robot is publishing telemetry data (sensor topics active)
- [ ] Test robot movement via cmd_vel:
  - Capture initial position from Gazebo topic
  - Send movement commands via cmd_vel
  - Verify position has changed after commands

### Stage 2: Autonomous Mapping and Navigation
- [ ] Implement SLAM for environment mapping
- [ ] Configure Nav2 stack for autonomous navigation
- [ ] Develop waypoint generation for exploration

**Test Suite:**
- [ ] Test that map topic is being published
- [ ] Test that Nav2 stack is operational (check services/action servers)
- [ ] Test waypoint navigation (verify robot receives and follows waypoints)
- [ ] Test map growth:
  - Capture initial map size/coverage
  - Allow robot to explore autonomously
  - Verify map size has increased substantially after exploration

### Stage 3: Aruco Tag Detection
- [ ] Add Aruco tag to the simulation environment
- [ ] Implement tag detection using camera feed
- [ ] Create visualization of detected tags in RViz

**Test Suite:**
- [ ] Test camera feed is available
- [ ] Test tag detection pipeline (with test images)
- [ ] Test detected tag is published as marker in map frame
- [ ] Test complete workflow:
  - Robot explores environment
  - Detects Aruco tag
  - Publishes tag location in map frame

### Stage 4: Multi-Robot Adversarial Task
- [ ] Add second TurtleBot3 with Aruco tag attached
- [ ] Implement random navigation for second robot
- [ ] Develop tag pursuit strategy for primary robot

**Test Suite:**
- [ ] Test second robot spawns correctly
- [ ] Test second robot moves randomly
- [ ] Test tag detection works on moving robot
- [ ] Test adversarial objective:
  - Primary robot detects and captures image of tag on second robot
  - System recognizes and reports successful capture

## Technical Guidelines
- Use **ROS 2** with **rclpy** for Python-based nodes
- Prefer **ament-cmake** over ament-python for project structure
- Use **CMake** instead of setup.py
- Utilize standard ROS message types where possible
- Design nodes to have a single purpose
- Orchestrate the system using Behaviour Trees
- Follow ROS 2 best practices for node communication
- Use ROS 2 Jazzy
