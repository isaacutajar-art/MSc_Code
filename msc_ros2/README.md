# Collaborative end-effector ROS 2 pipeline

ROS 2 Jazzy, Ubuntu 24.04, Python 3.12. Six separately launched Python nodes and a custom ROS interface package. Requires the Universal Robots ROS 2 driver.

## Nodes to Run

Install Jazzy using the official ROS instructions, plus the Universal Robots ROS 2 driver. Then, from this repository root:


## Nodes and interfaces

All array data is flattened in sensor order S1–S6, each sensor's zones 0–63. Distance is millimetres; velocity is metres/second, positive closing. Header timestamps use the PC ROS clock at UDP reception, not synchronized acquisition timestamps. Run this configuration on one PC with real time (`use_sim_time: false`).

| Node | Subscriptions | Publications / output |
|---|---|---|
| STVL53L7CX driver | Teensy UDP, port 8888 | `/measurement` (Measurement), `/status` (SensorStatus) |
| Human detection | `/measurement`, `/status`; UR RTDE pose | `/detection_1` (Detection), `/velocity_1` (Velocity) |
| Sensor health | `/status` | `/health_status` (std_msgs/Bool), 100 Hz |
| Speed scaling | `/detection_1` | `/speed_scaling_factor` (SpeedFactor), per detection |
| Decision making | `/speed_scaling_factor`, `/health_status` | `/robot_speed_command` (std_msgs/Float64), 100 Hz; UR speed service |
| LED lighting | `/health_status`, `/detection_1` | `/led_packet` (std_msgs/String) and optional serial, 10 Hz |
