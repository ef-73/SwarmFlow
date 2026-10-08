# SwarmFlow
## Intelligent Warehouse Fleet Coordination & Digital Twin
**Technical Design Document — Version 1.1**

> Source document as written by the user, kept for reference (§§9.1, 13.3, 15, 17–20 lightly
> condensed; content unchanged). Superseded by `docs/design.md` (v2.0). Do not edit.

**Status:** Planning  
**Primary Stack:** ROS 2 Jazzy, Gazebo Harmonic, Nav2, Docker, Python  
**Project Focus:** Robotics, distributed systems, fleet optimization, simulation, and visualization

---

# 1. Project Vision

SwarmFlow is a containerized, multi-robot warehouse simulation and digital twin designed to investigate intelligent fleet coordination.

Autonomous mobile robots navigate a shared 3D warehouse, collect packages at loading stations, and transport them to delivery destinations.

A central fleet orchestrator continuously manages delivery assignments, traffic reservations, congestion predictions, route decisions, and order deadlines.

The environment includes narrow aisles, varying package geometries, simulated human workers, temporary blockages, and changing workloads.

SwarmFlow combines physically simulated robotics with fleet-wide optimization and an interactive warehouse operations dashboard.

### Central Question

**Can predictive, deadline-aware traffic coordination improve warehouse throughput, reduce congestion, and respond more effectively to dynamic disruptions than conventional independent robot navigation?**

The project is intended to demonstrate ROS 2 and Docker engineering while providing an experimental platform for evaluating warehouse fleet optimization strategies.

# 2. Primary Goals

1. **ROS 2 engineering:** Build modular robot controllers, navigation stacks, custom interfaces, TF2 integration, and fleet communication.
2. **Docker infrastructure:** Containerize robot services, orchestration, simulation infrastructure, monitoring, and development environments.
3. **Intelligent orchestration:** Implement predictive congestion management, rolling-horizon replanning, deadline-aware scheduling, and dynamic corridor reservations.
4. **3D warehouse visualization:** Show robot movement, package loading and unloading, human workers, obstacles, and warehouse traffic in Gazebo.
5. **Interactive digital twin:** Build an operations dashboard displaying live fleet state, traffic conditions, orders, and scheduling decisions.
6. **Reproducible benchmarking:** Compare different orchestration strategies across warehouse densities, fleet sizes, and workloads.
7. **Scalability:** Support both detailed 3D simulation and faster, simplified large-fleet experiments.

# 3. Core Warehouse Problems

## 3.1 Fleet Congestion

Multiple robots may independently choose the shortest route, creating bottlenecks that reduce total delivery throughput.

SwarmFlow addresses this through:

- Predictive corridor utilization
- Proactive route selection
- Conflict-zone reservations
- Coordinated waiting
- Direction-aware traffic scheduling
- Traffic-aware task assignment

## 3.2 Dynamic Warehouse Operations

Unexpected changes can invalidate existing plans.

Examples:

- Workers crossing robot routes
- Temporary aisle blockages
- A robot taking longer than expected
- An occupied loading station
- A sudden increase in incoming orders

The system must detect changes and adapt without repeatedly disrupting otherwise valid plans.

## 3.3 Delivery Deadlines

Warehouse operations prioritize fulfilling orders, not simply minimizing individual robot travel distances.

Orders have release times, pickup locations, destinations, priorities, and deadlines.

The orchestrator considers estimated completion time, predicted congestion, existing commitments, and delivery urgency.

## 3.4 Dense Layout Constraints

Tightly packed storage shelves create limited maneuvering space and shared traffic bottlenecks.

SwarmFlow tests whether predictive coordination becomes more valuable as warehouse density and fleet utilization increase.

## 3.5 Payload-Dependent Navigation

Robots transport packages of different dimensions.

A robot's effective collision footprint changes when loaded, affecting aisle clearance, turning feasibility, and permissible routes.

# 4. 3D Simulation and Visual Experience

## 4.1 Warehouse World

Gazebo Harmonic provides the main visual simulation environment.

The warehouse contains:

- Shelving and storage racks
- Loading stations
- Delivery stations
- Narrow corridors and intersections
- Wide alternative routes
- Human worker models
- Mobile robot platforms
- 3D packages
- Temporary obstructions

Three configurable layouts are planned: open, standard, and dense.

## 4.2 Robot Design

Initially, the fleet consists of three differential-drive robots, expandable to larger fleets.

Each robot has:

- A simulated LiDAR
- Odometry and localization
- A flat cargo platform
- Independent ROS 2 navigation
- A visible robot identifier
- A configurable navigation footprint
- A local safety and navigation controller

Robots move continuously, not through discrete grid-cell transitions. Internal planners may still use occupancy grids and costmaps.

## 4.3 Visible Package Loading

Physical package manipulation is intentionally simplified.

**Loading workflow:**

1. Robot navigates to a loading station.
2. Robot aligns with the designated loading pose.
3. A 3D package appears above the platform.
4. The package descends onto the platform.
5. The package becomes rigidly attached to the robot.
6. Navigation footprint and payload state are updated.
7. Robot travels toward the delivery destination.

Packages may be rendered with different dimensions, colors, and order identifiers.

The initial animation may be scripted. Full contact simulation is not required.

**Unloading workflow:**

1. Robot reaches its destination.
2. Robot stops and initiates unloading.
3. The package is detached and placed at the unloading location.
4. The unloaded footprint is restored.
5. The order is completed.

## 4.4 Initial Payload Categories

| Type | Example dimensions | Main effect |
|---|---|---|
| Small | 0.40 × 0.40 m | Minimal restrictions |
| Medium | 0.65 × 0.65 m | Moderate clearance requirements |
| Wide | 1.00 × 0.65 m | Restricted aisle access |
| Long | 0.60 × 1.20 m | More difficult turns |

All dimensions are provisional and must be scaled to the selected robot model.

The navigation system uses the occupied footprint of both the chassis and payload, including safety margins.

# 5. System Architecture

SwarmFlow uses a hierarchical architecture.

## 5.1 Fleet Orchestrator

Responsible for high-level fleet decisions:

- Task assignment
- Order priority
- Estimated delivery completion times
- Congestion forecasting
- Corridor reservations
- Rolling-horizon planning
- Route selection
- Deadline management
- Fleet performance optimization

It does not directly command robot wheel velocities.

## 5.2 Independent Robot Controllers

Each robot runs a dedicated, namespaced ROS 2 navigation stack.

Responsibilities include:

- Executing navigation goals
- Reporting robot state
- Requesting corridor access
- Updating payload state
- Avoiding local obstacles
- Responding to emergency stops
- Reporting delays and failures

## 5.3 Traffic Manager

The traffic manager coordinates access to constrained shared spaces.

Its core functions include:

- Conflict-zone definition
- Reservation granting
- Reservation expiration
- Time-window scheduling
- Predicted zone occupancy
- Priority arbitration
- Deadlock prevention
- Fairness monitoring

The initial implementation uses centralized reservation authority to simplify correctness.

## 5.4 Scenario and Order Generator

Creates reproducible warehouse workloads.

It controls:

- Order arrival patterns
- Package types
- Delivery deadlines
- Human movement
- Corridor obstructions
- Robot delays
- Simulation events

## 5.5 Monitoring and Digital Twin

Collects telemetry and provides a real-time view of fleet operation.

The monitoring system should not become a source of truth for safety-critical navigation decisions.

# 6. Core Coordination Algorithms

## 6.1 Rolling-Horizon Replanning

The orchestrator periodically evaluates fleet state and upcoming tasks.

The planning loop:

1. Collect robot and order states.
2. Update predicted travel times.
3. Identify congested corridors.
4. Evaluate active task assignments.
5. Consider alternative routes.
6. Update reservations when beneficial.
7. Dispatch revised instructions.

An initial planning interval of approximately five seconds is proposed.

Significant events may trigger immediate replanning.

To avoid oscillation, existing routes should only be replaced when the expected improvement exceeds a configurable threshold.

## 6.2 Dynamic Corridor Reservations

Narrow corridors and intersections are modeled as conflict zones.

Reservations specify:

- Robot ID
- Zone ID
- Expected entry time
- Expected exit time
- Direction of movement
- Priority
- Expiration conditions

The system must guarantee that conflicting reservations are not granted simultaneously.

A robot may enter a reserved zone only when its reservation is valid and local navigation confirms the path is safe.

Initial policy: first-come-first-served exclusive reservations.

Advanced policy: deadline-sensitive and congestion-aware scheduling with direction batching.

## 6.3 Congestion Forecasting

The orchestrator predicts corridor utilization using:

- Robot trajectories
- Estimated arrival times
- Active reservations
- Expected passage duration
- Current obstructions
- Robot and package geometry

The objective is to avoid traffic bottlenecks before they form.

Initially, forecasting uses analytical estimates and scheduling data rather than machine learning.

## 6.4 Deadline-Aware Task Assignment

The orchestrator estimates when each available robot could complete an order.

Assignments consider:

- Robot distance from pickup
- Route feasibility
- Predicted pickup and delivery travel times
- Traffic delays
- Package compatibility
- Existing robot commitments
- Order deadline

A rolling-horizon scheduling algorithm adjusts assignments as conditions change.

Safety and feasible navigation remain hard constraints.

# 7. Interactive Warehouse Operations Dashboard

The dashboard is a core presentation and debugging feature.

A lightweight web application communicates with the monitoring and orchestration services through a ROS 2 bridge or dedicated API.

**Suggested stack:** React, TypeScript, FastAPI, WebSockets, and a lightweight charting library.

## 7.1 Fleet Overview

Displays:

- Active robots
- Robot location and state
- Current package
- Assigned destination
- Estimated arrival time
- Queue of pending orders
- Active and completed deliveries
- Current simulation mode

## 7.2 Live Warehouse Map

Displays a top-down representation of the simulation.

Potential overlays:

- Robot positions
- Intended paths
- Predicted trajectories
- Reserved traffic zones
- Congested corridors
- Human obstacle locations
- Restricted routes
- Loading and unloading stations

Gazebo remains responsible for the main 3D visualization.

The dashboard provides an operational view rather than duplicating the entire 3D renderer.

## 7.3 Performance Metrics

Live measurements:

- Deliveries per minute
- On-time delivery percentage
- Mean waiting time
- Mean delivery latency
- Corridor utilization
- Active congestion events
- Collision and clearance violations
- Number of route replans

## 7.4 Robot Inspection

Selecting a robot reveals:

- Its current assignment
- Remaining route
- Active reservations
- Payload geometry
- Estimated completion time
- Current navigation status
- Recent orchestration decisions

## 7.5 Optimization Mode Selection

The user can select between:

**Throughput Mode:** Prioritize overall completed deliveries.

**Deadline Mode:** Prioritize reducing late deliveries.

**Balanced Mode:** Balance travel efficiency, waiting time, and tardiness.

Changing modes updates the orchestrator's scheduling weights without relaxing safety constraints.

# 8. Explainable Fleet Orchestration

The orchestrator should expose why it makes significant decisions.

Every major scheduling or routing decision produces a structured event.

### Example Events

**Route change**

"Robot 2 rerouted because Corridor B has an estimated 12-second delay. Alternative Route C is predicted to save 7 seconds."

**Priority update**

"Robot 3 granted priority because Order 104 has a predicted deadline risk."

**Congestion response**

"Corridor A utilization exceeded the configured threshold. New route assignments will avoid the corridor when feasible."

**Human disruption**

"Unexpected obstruction detected in Corridor D. Existing reservations are being reevaluated."

## 8.1 Event Data

Events should record:

- Timestamp
- Robot or order identifier
- Decision type
- Previous decision
- New decision
- Relevant cost estimates
- Triggering event
- Predicted improvement

## 8.2 Importance

This feature supports:

- Debugging
- Algorithm evaluation
- Reproducibility
- Dashboard explanations
- Technical presentations
- Identifying poor scheduling decisions

Decision explanations must be generated from actual scheduler inputs and outputs, not invented after execution.

# 9. Interactive Stress-Test Mode

SwarmFlow should support live injection of warehouse disruptions.

This turns the simulation into an interactive test environment.

## 9.1 Supported Events

**Urgent order burst** — Generate multiple orders with tight deadlines.

**Corridor closure** — Temporarily block a selected aisle.

**Human traffic increase** — Increase worker movement around busy routes.

**Robot slowdown** — Reduce a selected robot's maximum speed.

**Loading station delay** — Create a temporary pickup queue.

**Fleet expansion** — Spawn additional robots where supported by the active simulation configuration.

**Traffic policy change** — Switch between reactive and predictive coordination.

## 9.2 Event Handling

The event generator publishes a structured simulation event.

The appropriate subsystem modifies the scenario.

Examples:

- A corridor closure updates the traffic manager.
- A new order enters the scheduling queue.
- A simulated worker begins crossing an aisle.
- A robot slowdown updates ETA predictions.

The orchestrator then reacts through its normal planning mechanism.

## 9.3 Reproducible Scenarios

Stress-test configurations should support:

- Random seed
- Scenario duration
- Fleet size
- Warehouse layout
- Order arrival rate
- Worker activity
- Deadline distribution
- Optimization policy

Interactive experiments can be recorded as event timelines and replayed.

This allows the same disruption sequence to be evaluated against multiple orchestration algorithms.

# 10. Dual Simulation Architecture

SwarmFlow should distinguish between visual fidelity and evaluation speed.

## 10.1 Detailed 3D Backend

**Technology:** Gazebo Harmonic + ROS 2 + Nav2

Purpose:

- Showcase realistic robot motion
- Visualize box loading and unloading
- Simulate sensors and obstacles
- Demonstrate ROS 2 integration
- Validate navigation behavior
- Produce demonstration videos

Initial target: three robots.

The exact number may increase if performance allows.

## 10.2 Lightweight Fleet Backend

**Technology:** Python-based 2D simulation, potentially with a custom kinematic model.

Purpose:

- Test large fleets efficiently
- Evaluate congestion algorithms
- Run repeated experiments
- Compare scheduling strategies
- Vary workload and density
- Benchmark scalability

The lightweight backend does not need Gazebo physics or detailed 3D models.

It should use the same warehouse topology, order formats, robot capabilities, traffic constraints, and fleet-orchestrator interfaces wherever practical.

Continuous planar robot positions and simple movement dynamics should still be represented.

Its behavior will be an approximation, not a full substitute for Nav2 and Gazebo.

## 10.3 Shared Logic

Both simulation backends should reuse the core:

- Order models
- Task assignment algorithms
- Traffic reservations
- Congestion forecasting
- Deadline optimization
- Orchestrator state machine
- Metrics collection

Simulation-specific adapters provide robot states and accept abstract navigation assignments.

This prevents the lightweight simulation from becoming an entirely separate project.

## 10.4 Development Priority

The 3D backend is the initial deliverable.

The lightweight backend should be implemented after the core coordination interfaces are stable.

Do not postpone the first working ROS 2 demonstration to build a large custom simulator.

# 11. Dynamic Obstacles and Human Workers

Workers are simulated as moving 3D agents.

They may:

- Cross aisles
- Walk between workstations
- Pause near shelves
- Block a corridor temporarily
- Occupy loading station approaches

The local robot controller handles immediate obstacle avoidance.

The fleet orchestrator handles sustained congestion and longer-term route changes.

Human motion is not assumed to be coordinated with robot traffic reservations.

The initial implementation uses scripted trajectories with randomized timing.

A future extension may estimate worker movement trends, with uncertainty explicitly considered.

# 12. Technical Stack

| Subsystem | Technology |
|---|---|
| Robotics middleware | ROS 2 Jazzy |
| 3D simulation | Gazebo Harmonic |
| Navigation | Nav2 |
| Robot definition | URDF, Xacro, SDF |
| Simulation interface | ros_gz |
| Transform management | TF2 |
| Robotics logic | Python / rclpy |
| Optional optimized nodes | C++ / rclcpp |
| Containerization | Docker |
| Service management | Docker Compose |
| Operations dashboard | React + TypeScript |
| Backend API | FastAPI |
| Live telemetry | WebSockets |
| Optimization | Python heuristics, optional OR-Tools |
| Lightweight simulation | Python |
| Data analysis | pandas, NumPy, matplotlib |
| Experiment recording | rosbag2, CSV/JSON logs |
| Testing | pytest, ROS 2 integration tests |
| Version control | Git / GitHub |

# 13. ROS 2 and Docker Architecture

## 13.1 Proposed ROS 2 Services

- Fleet orchestrator node
- Traffic reservation manager
- Congestion prediction node
- Order generator
- Scenario controller
- Robot fleet agents
- Package state manager
- Monitoring node

Use namespaces for per-robot interfaces.

## 13.2 Proposed Topics and Actions

| Interface | Purpose |
|---|---|
| `/fleet/robot_states` | Robot telemetry |
| `/fleet/orders` | Incoming order events |
| `/fleet/reservations` | Traffic reservations |
| `/fleet/congestion` | Predicted corridor utilization |
| `/fleet/decisions` | Structured explanation events |
| `/fleet/scenario_events` | Stress-test events |
| `/fleet/request_reservation` | Corridor reservation service |
| `/fleet/dispatch_task` | Delivery task action |
| `/robot_N/navigate_to_pose` | Nav2 navigation action |
| `/robot_N/payload_state` | Package and footprint state |

Exact message schemas will be finalized during implementation.

## 13.3 Docker Services

Proposed services: `gazebo`, `robot_1`, `robot_2`, `robot_3`, `fleet_orchestrator`, `scenario_engine`, `telemetry_api`, `dashboard`.

A reusable robot image should support multiple robot instances through configuration.

Use Docker Compose for orchestration, service configuration, and reproducible startup.

Robot navigation, fleet coordination, and monitoring should remain logically separate even if container boundaries are adjusted for practical reasons.

# 14. Optimization and Benchmarking

## 14.1 Core Performance Metrics

**Throughput:** Completed deliveries per unit time.

**Deadline success rate:** Percentage of deliveries completed before their deadlines.

**Average delivery latency:** Order creation to completed delivery.

**Average robot waiting time:** Time spent delayed by congestion and reservations.

**Collision and clearance violations:** Safety-related failures during simulation.

## 14.2 Secondary Metrics

- Travel distance per order
- Robot idle time
- Corridor utilization
- Number of replans
- Orchestrator computation time
- Prediction accuracy
- Scheduling fairness
- Unfinished orders at the end of an experiment

## 14.3 Benchmark Policies

**Baseline A: Independent Nav2** — Robots navigate independently with local obstacle avoidance.

**Baseline B: Reactive Reservations** — Robots coordinate through basic first-come-first-served corridor reservations.

**Proposed Policy: Predictive Orchestration** — Uses congestion forecasting, adaptive reservations, rolling-horizon replanning, and deadline-aware priorities.

## 14.4 Benchmark Environments

- Open warehouse
- Standard warehouse
- Dense warehouse
- Low, medium, and high order demand
- Different package-size distributions
- Increasing robot counts
- Different worker activity levels

Each policy should be evaluated across matched scenarios and random seeds.

A key research result will be identifying the congestion tipping point at which increasing fleet size ceases to improve throughput.

# 15. Updated Implementation Roadmap

1. **Working 3D Warehouse** — Gazebo world, three robots, delivery stations, visible loading/unloading, Dockerized ROS 2. Milestone: three robots complete deliveries.
2. **Basic Fleet Orchestration** — order generator, robot state, task assignment, orchestrator, corridor reservations. Milestone: robots coordinate access to shared passages.
3. **Predictive Optimization** — forecasting, rolling horizon, adaptive corridor scheduling, deadlines, decision logging. Milestone: orchestrator explains changes and responds to predicted congestion.
4. **Dynamic Warehouse Operations** — scripted workers, blockages, delay tracking, event-triggered replanning. Milestone: disruptions produce observable fleet responses.
5. **Payload-Aware Routing** — package types, footprint updates, infeasible routes, turning clearance. Milestone: decisions change with payload geometry.
6. **Live Dashboard** — fleet overview, positions, orders, reservations, inspection, explanations, metrics. Milestone: user can inspect fleet decisions while observing Gazebo.
7. **Interactive Stress Testing** — scenario injection, bursts, closures, worker controls, policy changes, record/replay. Milestone: user triggers disruptions and observes responses.
8. **Lightweight Simulation and Benchmarking** — 2D backend, shared interfaces, workload generation, repeatable experiments, plots. Milestone: performance evaluated across layouts, workloads, fleet sizes.
9. **Final Presentation** — polish, demo scenario, walkthroughs, benchmark results, video, docs.

# 16. Development and Testing Strategy

Start with a minimal working system before integrating advanced features.

Critical early validation tasks:

1. Confirm ROS 2 Jazzy, Gazebo Harmonic, and Nav2 work together.
2. Confirm multi-robot namespacing and TF2 behavior.
3. Confirm ROS 2 communication between Docker containers.
4. Confirm reliable Gazebo package attachment and detachment.
5. Confirm dynamic navigation footprint updates.
6. Confirm traffic reservations do not grant overlapping conflicting permissions.

Automated tests should cover scheduler decisions, reservation conflicts, deadline calculations, task assignment, and event replay.

Integration tests should verify that robot services, traffic coordination, and simulation adapters communicate correctly.

# 17. Additional Engineering Considerations

**17.1 Simulation Time** — All components use consistent simulated time; distinguish simulation from wall-clock timestamps.

**17.2 Deterministic Experiments** — Reproducible seeds and recorded scenario events; store config and software versions with results.

**17.3 Deadlock and Starvation** — Timeouts, reservation release rules, fairness checks, recovery procedures.

**17.4 Safety Constraints** — Safety not traded for speed; local collision avoidance retained if orchestrator unavailable; simulated human avoidance is not a certified safety system.

**17.5 Explainable Decisions** — Logs expose real scheduler reasoning; explanations only as accurate as predictions.

**17.6 Resource Management** — Monitor Gazebo, ROS 2 process count, Docker resources; use the lightweight backend for scale.

# 18. Future Extensions (out of initial scope)

Adaptive traffic lanes · Package handoffs · Rotating cargo platforms · Heterogeneous fleets · Battery-aware scheduling · ML-based congestion prediction · Reinforcement learning · Decentralized coordination · Cooperative transport · Physical robot deployment.

# 19. Final Demonstration Concept

Normal demand → visible package loading/delivery → rising demand congests shared corridors under a baseline → predictive orchestration enabled, fleet adjusts reservations and routes → urgent order reprioritized → human blocks a corridor, robots stop/reroute locally while the orchestrator replans → dashboard shows explanations, orders, congestion, metrics → recorded experiments compare strategies.

# 20. Long-Term Project Identity

**An interactive warehouse robotics digital twin and experimental fleet-optimization platform built with ROS 2, Docker, and Gazebo.**

The project does not attempt to recreate Amazon's internal warehouse software or claim to have solved fleet congestion universally. It provides an extensible and measurable implementation of intelligent warehouse coordination techniques.

**Final Goal:** Deliver a visually impressive robotics project that demonstrates both practical ROS 2/Docker engineering and meaningful algorithmic work in warehouse fleet optimization.
