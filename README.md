# Autonomous Navigation Stack (ROS 2)

A complete navigation system that drives a simulated differential-drive robot from its current position to a goal you click, avoiding obstacles it discovers along the way. It uses only a 2D lidar, and the robot starts with no map of the room.

Built with ROS 2 Humble, Gazebo and Foxglove, running in Docker.

## How it works

Four nodes form a sense → plan → act pipeline. Each one only reads and writes ROS topics, so any stage can be inspected or replaced on its own.

```
Gazebo ──/lidar──▶ Costmap ──/costmap──▶ Map Memory ──/map──▶ Planner ──/path──▶ Control ──/cmd_vel──▶ Robot
                                              ▲                  ▲  ▲                ▲
                                /odom/filtered┘   /goal_point ───┘  └─ /odom/filtered ┘
```

| Node | Role | What it does |
|---|---|---|
| **Costmap** | Perception | Converts each lidar scan into a 30 × 30 m grid around the robot (0.1 m cells). Marks obstacles and inflates them with a linearly decaying cost over 1 m, to keep a safety margin. |
| **Map Memory** | Mapping | Transforms each costmap into world coordinates using the robot's pose and fuses it into a persistent global map. Updates every 1.5 m of travel, and immediately on startup. |
| **Planner** | Planning | Runs A\* (8-connected, Euclidean heuristic) on the global map, with step costs weighted by proximity to obstacles. Replans whenever the map updates; handles goals inside obstacles, timeouts, and starting inside an inflation zone. |
| **Control** | Acting | Follows the path with Pure Pursuit (1.5 m lookahead, κ = 2y/L²). Rotates in place when the target is far off-heading, and stops cleanly at the goal without overriding manual teleop. |

The planner also publishes a goal marker (`/goal_marker`): a sphere labelled *Goal N: driving / reached / no safe route*, coloured orange, green or red.

## Results

| Test | Distance | Time | Final error |
|---|---|---|---|
| Around the large cylinder, starting against it | ≈ 7.6 m | 13 s | 0.19 m |
| Far side of the large cylinder | ≈ 16 m | 27 s | 0.18 m |
| Past a box, goal clicked in Foxglove | ≈ 15 m | 27 s | 0.16 m |
| Goal clicked against a wall | n/a | n/a | correctly refused |

## Running it

Requires Docker (Linux, or WSL on Windows).

```bash
./watod build robot      # compile the navigation nodes
./watod up --detach      # start the simulator, robot and Foxglove bridge
```

Then open Foxglove in a browser:

```
https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws://127.0.0.1:20000
```

Import the layout from `config/`, enable the `/map`, `/path` and `/goal_marker` layers in the 3D panel, select the **publish point** tool (⊙) and click a goal on the floor.

Stop everything with `./watod down`.

## Tuning

All parameters live in each node's `config/params.yaml`.

| Parameter | Node | Default | Effect |
|---|---|---|---|
| `inflation_radius` | costmap | 1.0 m | Clearance kept from obstacles |
| `distance_threshold` | map_memory | 1.5 m | Travel between map updates |
| `obstacle_threshold` | planner | 50 | Cost above which a cell is treated as a wall |
| `cost_weight` | planner | 2.0 | Preference for routes away from obstacles |
| `lookahead_distance` | control | 1.5 m | Smoothness vs. path-tracking accuracy |
| `linear_speed` | control | 0.8 m/s | Forward speed |

## Repository layout

```
src/robot/costmap/       Costmap node
src/robot/map_memory/    Map Memory node
src/robot/planner/       Planner node (A*, state machine, goal marker)
src/robot/control/       Control node (Pure Pursuit)
src/robot/bringup_robot/ Launch file for all robot nodes
config/                  Foxglove layout
docker/, modules/        Container definitions
```

## Acknowledgements

The simulation environment, Docker tooling and node skeletons come from the WATonomous ASD training template, used under the Apache 2.0 license (see `LICENSE`). The Costmap, Map Memory, Planner and Control implementations are my own.
