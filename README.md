download: [laundry_dt_readme.md](https://github.com/user-attachments/files/26049565/laundry_dt_readme.md)
# laundry_dt — Autonomous Laundry Robot & Digital Twin

> A ROS 2-based autonomous segmented robot that handles the full laundry lifecycle — stair navigation, machine loading, wash/dry cycle monitoring, and delivery back to the user. Built with a real-time digital twin pipeline mirroring physical hardware state in simulation.

---

##  Concept

Laundry is one of the last fully manual household chores. Existing "smart" appliances only automate the washing itself — someone still has to load, transfer, and deliver. `laundry_dt` rethinks the entire pipeline.

The robot is a **3-segment autonomous platform** with a detachable drum that mounts directly into standard washing and drying machines. It navigates autonomously — including up and down stairs — picks up laundry, loads the washer, monitors the cycle, transfers to the dryer, and returns clean laundry to the user.

---

##  System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    laundry_dt ROS 2 Stack                    │
│                                                             │
│  [Ultrasonic Publisher] ──/ultrasonic/{pos}/distance──►     │
│  [Camera Publisher]     ──/camera/image_raw──►              │
│                                   │                         │
│                          [Robot Controller]                 │
│                          (state machine)                    │
│                    ┌──────────┴──────────┐                  │
│                    ▼                     ▼                  │
│       [DC Motor Controller]    [Stepper Controller]         │
│         /{pos}/cmd_vel           /stepper/{pos}/cmd         │
│                    │                     │                  │
│                    ▼                     ▼                  │
│       [DC Motor Subscriber]   [Stepper Subscriber]          │
└─────────────────────────────────────────────────────────────┘
```

### Nodes & Topics

| Node | Publishes | Subscribes To | Role |
|---|---|---|---|
| `ultrasonic_publisher` | `/ultrasonic/front/distance` `/ultrasonic/mid/distance` `/ultrasonic/back/distance` | — | Per-segment obstacle & stair detection |
| `robot_controller` | `/{pos}/cmd_vel` `/stepper/{pos}/cmd` | `/ultrasonic/{pos}/distance` `/stepper/{pos}/steps` | Central state machine brain |
| `dc_motor_controller` | `/{pos}/motors/dc/status` | `/{pos}/cmd_vel` | Per-segment drive control |
| `stepper_controller` | `/stepper/{pos}/steps` | `/stepper/{pos}/cmd`  && `mid/winch/cmd` | Per-segment precision lift control |
| `dc_motor_subscriber` | — | `/{pos}/cmd_vel` | Motor state monitoring |
|||||

---

##  Hardware

### Physical Components

| Component | Quantity | Role |
|---|---|---|
| HC-SR04 Ultrasonic | 6 | One per segment — obstacle avoidance + stair measurement |
| DC Motors | 7 | Locomotion per segment + winch pull on mid segment (1) |
| Stepper Motors | 2 | Precision lift mechanism per segment for stair climbing |


### Sensor Layout

| Sensor | Position | Direction | Purpose |
|---|---|---|---|
| `front` | Front segment | Forward | Stair/obstacle detection |
| `mid` | Mid segment | Forward | Stair/obstacle detection |
| `back` | Rear segment | Forward | Stair/obstacle detection |

Each ultrasonic faces forward. Stair detection uses distance **increase** (floor dropping away) combined with stepper dead reckoning for vertical positioning.

---

##  State Machine

The `robot_controller` node manages a finite state machine (FSM) that governs all robot behavior.

```python
class RobotState(Enum):
    IDLE             = 0
    ADVANCING        = 1
    LIFTING          = 2
    SEGMENT_FORWARD  = 3
    BACK_WINCHING    = 4
    COMPLETE         = 5
```

---

##  Stair Climbing Algorithm

The robot uses a **segmented inchworm approach** — front, mid, and back segments leapfrog each other up the staircase independently.

### Per-Stair Cycle (repeats for every stair, every segment)

```
1. STAIR_DETECTED
   └─ Ultrasonic reads distance increase (floor drops away)
   
2. FRONT_LIFTING
   └─ Front stepper lifts, counting steps
   └─ Ultrasonic clears threshold → record step count as stair height
   └─ Add fixed offset for front segment physical size
   
3. FRONT_ADVANCING
   └─ Front DC motors drive forward
   └─ Front ultrasonic detects next stair edge → stop
   
4. MID_LIFTING
   └─ Mid stepper lifts using same measurement process
   └─ Mid advances to where front segment was
   
5. FRONT_LIFTING (again)
   └─ Front lifts to next stair, advances to stair 3 position
   
6. BACK_LIFTING
   └─ Back stepper lifts, advances to stair 1 position
   
7. REPEAT
   └─ Cycle continues until top of staircase
```

### Motor Behavior Per State

| State | Front DC | Mid DC | Back DC | Front Stepper | Back Stepper |
|---|---|---|---|---|---|
| `IDLE` | STOP | STOP | STOP | HOLD | HOLD |
| `MOVING_FORWARD` | FORWARD | FORWARD | FORWARD | HOLD | HOLD |
| `STAIR_DETECTED` | STOP | STOP | STOP | HOLD | HOLD |
| `FRONT_LIFTING` | STOP | STOP | STOP | UP | HOLD |
| `FRONT_ADVANCING` | FORWARD | HOLD | HOLD | HOLD | HOLD |
| `MID_LIFTING` | STOP | STOP | STOP | DOWN | UP |
| `MID_ADVANCING` | HOLD | FORWARD | HOLD | HOLD | HOLD |
| `BACK_LIFTING` | STOP | STOP | STOP | HOLD | DOWN |
| `BACK_ADVANCING` | HOLD | HOLD | FORWARD | HOLD | HOLD |

### Key Design Decisions

**Non-uniform stair handling** — each segment independently measures its own stair height via stepper dead reckoning + ultrasonic feedback on every single stair. No assumption of uniform stair height. This enables reliable navigation of real-world staircases.

**Stepper as position encoder** — steppers move in precise increments so step count = exact vertical displacement. No additional height sensor needed. The ultrasonic detects *when* to stop lifting; the stepper tracks *how much* it lifted.

**Per-segment independence** — front, mid, and back can be climbing different-height stairs simultaneously, which is the realistic scenario on any real staircase.

### Data Flow for Stair Climbing

| Topic | Data | Used by |
|---|---|---|
| `/ultrasonic/{pos}/distance` | Raw distance (cm) | `robot_controller` |
| `/stepper/{pos}/steps` | Current step count | `robot_controller` |
| `/stepper/{pos}/cmd` | Target steps to move | `stepper_controller` |
| `/{pos}/cmd_vel` | Linear/angular velocity | `dc_motor_controller` |

---

##  Laundry Lifecycle (Future States)

```
[Pickup]
  → [Navigate to Washer]
    → [Align & Detach Drum]
      → [Monitor Wash Cycle]
        → [Reattach Drum]
          → [Navigate to Dryer]
            → [Align & Detach Drum]
              → [Monitor Dry Cycle]
                → [Reattach Drum]
                  → [Return to User]
```

---

##  Getting Started

### Prerequisites
- Docker
- Mac / Linux / Windows (WSL2)

### Run with Docker

```bash
docker pull osrf/ros:humble-desktop

git clone https://github.com/RHy910/laundry_bot.git

docker run -it --rm \
  --name laundry_dt \
  -v $(pwd)/laundry_bot:/root/ros2_ws \
  osrf/ros:humble-desktop bash
```

### Build & Launch

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select laundry_dt
source install/setup.bash
ros2 launch laundry_dt launch.py
```

### Visualize Node Graph

```bash
ros2 run rqt_graph rqt_graph
```

### Monitor Live Sensor Data

```bash
ros2 topic echo /ultrasonic/front/distance
ros2 topic echo /front/cmd_vel
```

---

##  Project Structure

```
laundry_dt/
├── laundry_dt/
│   ├── __init__.py
│   ├── ultrasonic_publisher.py     # 3-sensor pub with dict comprehension
│   ├── camera_publisher.py         # Visual data publisher
│   ├── dc_motor_controller.py      # Reactive per-segment motor controller
│   ├── dc_motor_subscriber.py      # Per-segment motor state monitor
│   ├── stepper_controller.py       # Precision lift controller (WIP)
│   └── robot_controller.py         # Central FSM brain (WIP)
├── launch/
│   └── launch.py                   # Single-command full system launch
├── package.xml
└── setup.py
```

---

## Roadmap

- [x] ROS 2 pub/sub pipeline — ultrasonic publisher, motor controller, motor subscriber
- [x] Per-segment topic architecture using dictionary comprehensions
- [x] Lambda callbacks for position-aware subscriptions
- [x] Single launch file for full system startup
- [x] State machine design (FSM defined, implementation in progress)
- [ ] `robot_controller` node — central FSM implementation
- [ ] Stepper dead reckoning — step count → vertical displacement
- [ ] Stair climbing algorithm implementation
- [ ] Gazebo simulation — virtual laundry_dt mirroring physical state
- [ ] rosbridge integration — live ESP32 hardware data into ROS 2 topics
- [ ] AWS S3 sensor data logging pipeline
- [ ] Computer vision node — stair geometry detection via ESP32-CAM
- [ ] Going down stairs

---

## Known Limitations & Future Work

- Camera-based stair geometry detection not yet implemented — currently relies purely on ultrasonic distance thresholds
- Downstairs navigation is a separate algorithm not yet designed
- Drum detach/attach mechanism is mechanical — software integration pending hardware build

---

## 🔗 Context

Built as part of a broader exploration into autonomous systems and digital twin infrastructure. The architecture here — hardware abstraction nodes, reactive per-segment controllers, FSM-based behavior, simulation-first validation — mirrors the development workflows used in production autonomous vehicle stacks.

---

## 👤 Author

**Ralph Hyacinthe** · [LinkedIn](https://linkedin.com/in/ralphhyacinthe/) · [GitHub](https://github.com/RHy910)
