# laundry_dt — Autonomous Laundry Robot & Digital Twin

> A ROS 2-based autonomous segmented robot that handles the full laundry lifecycle — stair navigation, machine loading, wash/dry cycle monitoring, and delivery back to the user. Built with a real-time digital twin pipeline mirroring physical hardware state in simulation.

---

## Concept

Laundry is one of the last fully manual household chores. Existing "smart" appliances only automate the washing itself — someone still has to load, transfer, and deliver. `laundry_dt` rethinks the entire pipeline.

The robot is a **3-segment autonomous platform** with a detachable drum that mounts directly into standard washing and drying machines. It navigates autonomously — including up and down stairs — picks up laundry, loads the washer, monitors the cycle, transfers to the dryer, and returns clean laundry to the user.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    laundry_dt ROS 2 Stack                    │
│                                                             │
│  [Ultrasonic Publisher] ──/ultrasonic/{pos}/distance──►     │
│  [Staircase Publisher]  ──/ultrasonic/{pos}/distance──►     │
│                         ──/stepper/front/steps──►           │
│                                   │                         │
│                          [Robot Controller]                 │
│                          (state machine)                    │
│          ┌───────────────┬─────────┴──────────┐            │
│          ▼               ▼                    ▼            │
│  [DC Motor Controller] [Stepper Controller] [Winch]        │
│    /{pos}/cmd_vel      /stepper/{pos}/cmd  /mid/winch/cmd  │
│          │               │                                  │
│          ▼               ▼                                  │
│  [DC Motor Subscriber] [Stepper Subscriber]                 │
└─────────────────────────────────────────────────────────────┘
```

### Nodes & Topics

| Node | Publishes | Subscribes To | Role |
|---|---|---|---|
| `ultrasonic_publisher` | `/ultrasonic/{pos}/distance` | — | Simulated per-segment distance readings (random, for testing) |
| `staircase_publisher` | `/ultrasonic/{pos}/distance` `/stepper/front/steps` | `/segment_lifted` | Simulates a realistic staircase — decrements readings as segments approach, publishes stair heights, unlocks segments on lift confirmation |
| `robot_controller` | `/{pos}/cmd_vel` `/stepper/{pos}/cmd` `/mid/winch/cmd` `/segment_lifted` | `/ultrasonic/{pos}/distance` `/stepper/{pos}/steps` | Central state machine brain — drives FSM transitions, coordinates all segment movement |
| `dc_motor_controller` | `/{pos}/cmd_vel` | `/ultrasonic/{pos}/distance` | Reactive per-segment drive controller — independently stops a segment when its ultrasonic reads below threshold |
| `stepper_controller` | `/stepper/{pos}/steps` `/mid/winch/steps` | `/stepper/{pos}/cmd` `/mid/winch/cmd` | Relays stepper and winch commands, echoes back step counts |
| `dc_motor_subscriber` | — | `/{pos}/cmd_vel` | Motor state monitoring / logging |
| `stepper_subscriber` | — | `/stepper/{pos}/cmd` | Stepper command monitoring / logging |

---

## Hardware

### Physical Components

| Component | Quantity | Role |
|---|---|---|
| HC-SR04 Ultrasonic | 6 | Two per segment — one forward-facing (obstacle/stair detection), one downward-facing (step-down detection for descending) |
| DC Motors | 7 | Locomotion per segment + winch pull on mid segment (1) |
| Stepper Motors | 2 | Precision lift mechanism — front and back segments |

### Sensor Layout

| Sensor | Position | Direction | Purpose |
|---|---|---|---|
| `front_forward` | Front segment | Forward | Stair/obstacle detection, stair height scouting |
| `mid_forward` | Mid segment | Forward | Stair/obstacle detection |
| `back_forward` | Rear segment | Forward | Stair/obstacle detection |
| `front_down` | Front segment | Downward | Step-down detection for stair descent |
| `mid_down` | Mid segment | Downward | Step-down detection for stair descent |
| `back_down` | Rear segment | Downward | Step-down detection for stair descent |

Forward-facing sensors detect stairs and obstacles — stair detection triggers when distance drops below `stair_threshold` (default: 10 cm). Downward-facing sensors detect when a segment is over a stair edge during descent, triggering the lowering sequence for that segment.

---

## State Machine

The `robot_controller` node manages a finite state machine (FSM) that governs all robot behavior.

```python
class RobotState(Enum):
    IDLE             = 0   # waiting to start
    ADVANCING        = 1   # all segments driving forward
    LIFTING          = 2   # active segment lifting, waiting to clear threshold
    SEGMENT_FORWARD  = 3   # all segments moving to next position
    BACK_WINCHING    = 4   # winch pulling back segment before it advances
    COMPLETE         = 5   # all segments at top, climb finished
```

---

## Stair Climbing Algorithm

The robot uses a **segmented inchworm approach** — front, mid, and back segments leapfrog each other up the staircase, coordinated by `get_active_seg()` and a per-segment level tracker.

### Active Segment Logic (`get_active_seg`)

Each segment tracks which stair level it's on. The active segment — the one that should act next — is determined by the level relationship between segments:

```
levels {f:1, m:1, b:1} → active: front   (front == mid, front leads)
levels {f:2, m:1, b:1} → active: mid     (front > mid, mid == back)
levels {f:2, m:2, b:1} → active: back    (front == mid > back)
levels {f:3, m:2, b:1} → active: back    (front > mid > back)
levels {f:3, m:2, b:2} → active: mid     (front > mid, mid == back)
levels {f:3, m:3, b:2} → active: front   (front == mid > back)
```

Level order is always enforced as `front >= mid >= back`. Any violation is logged as an error.

### Per-Stair Cycle

```
1. IDLE → ADVANCING
   └─ First ultrasonic reading received — all segments start driving forward

2. ADVANCING
   └─ Active segment ultrasonic drops below stair_threshold
   └─ All segments halt → state → LIFTING
   └─ front: step_up(999) to scout stair height; scouting_level stored
   └─ mid/back: step using stored stair_heights[target_level]

3. LIFTING
   └─ Active segment ultrasonic clears threshold → segment has climbed the stair
   └─ All segments drive forward → state → SEGMENT_FORWARD

4. SEGMENT_FORWARD
   └─ Each segment halts when it detects the next stair OR travels a full segment_length (dead reckoning cutoff)
   └─ front_at_top flag set if ultrasonic reads > 300 (landing signal)
   └─ When all segments stopped → level[active] += 1
   └─ Next active segment determined:
        if next_active == 'back' → state → BACK_WINCHING
        else → do_lift(next_active) → state → LIFTING

5. BACK_WINCHING
   └─ Winch fires to pull back segment
   └─ After winch_duration seconds → back drives forward → state → ADVANCING

6. COMPLETE
   └─ front_at_top == True and all levels equal → climb finished
```

### Stepper Behavior Per Segment

| Segment Being Lifted | Front Stepper | Back Stepper |
|---|---|---|
| `front` | UP (scout — step_up 999) | HOLD |
| `mid` | DOWN (lowers front to release mid) | UP (raises back, lifts mid) |
| `back` | HOLD | DOWN (lowers back segment) |

### Motor Behavior Per State

| State | Front DC | Mid DC | Back DC | Front Stepper | Back Stepper |
|---|---|---|---|---|---|
| `IDLE` | STOP | STOP | STOP | HOLD | HOLD |
| `ADVANCING` | FORWARD | FORWARD | FORWARD | HOLD | HOLD |
| `LIFTING` | STOP | STOP | STOP | per table above | per table above |
| `SEGMENT_FORWARD` | FORWARD | FORWARD | FORWARD | HOLD | HOLD |
| `BACK_WINCHING` | STOP | STOP (winch active) | STOP | HOLD | HOLD |
| `COMPLETE` | STOP | STOP | STOP | HOLD | HOLD |

### Key Design Decisions

**Scouting on every stair** — front always lifts to scout before mid and back use the stored height. This means no assumption of uniform stair height and reliable navigation of real-world staircases.

**Stepper as position encoder** — step count equals exact vertical displacement. The ultrasonic detects *when* to stop lifting; the stepper tracks *how much* it lifted. Heights are stored in `stair_heights[level]`.

**Dead reckoning cutoff** — if no stair is detected after traveling `segment_length` cm (default: 30 cm), the segment halts anyway. This handles the landing at the top of the staircase.

**Landing detection** — when the front ultrasonic reads > 300, `front_at_top` is set. Once all segments reach the same level with this flag set, the climb is marked `COMPLETE`.

**Back winch** — when back is the next active segment, a winch motor pulls it before it drives forward, allowing it to advance without requiring a full lift cycle.

### Data Flow for Stair Climbing

| Topic | Data | Direction |
|---|---|---|
| `/ultrasonic/{pos}/distance` | Raw distance (cm) | → `robot_controller` |
| `/stepper/{pos}/steps` | Live step count feedback | → `robot_controller` |
| `/stepper/{pos}/cmd` | Target steps (positive = up, negative = down) | `robot_controller` → `stepper_controller` |
| `/{pos}/cmd_vel` | Linear velocity (Twist) | `robot_controller` → `dc_motor_controller` |
| `/mid/winch/cmd` | Winch step count | `robot_controller` → `stepper_controller` |
| `/segment_lifted` | Segment name string | `robot_controller` → `staircase_publisher` |

---

## Laundry Lifecycle (Future States)

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

## Getting Started

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
ros2 topic echo /stepper/front/steps
```

---

## Project Structure

```
laundry_dt/
├── laundry_dt/
│   ├── __init__.py
│   ├── ultrasonic_publisher.py     # Simulated 3-sensor publisher (random values, for unit testing)
│   ├── staircase_publisher.py      # Realistic staircase simulator for FSM testing
│   ├── dc_motor_controller.py      # Reactive per-segment motor controller (ultrasonic-driven)
│   ├── dc_motor_subscriber.py      # Per-segment motor state monitor / logger
│   ├── stepper_controller.py       # Stepper + winch relay with step count feedback
│   ├── stepper_subscriber.py       # Stepper command monitor / logger
│   ├── robot_state.py              # RobotState enum (shared)
│   └── robot_controller.py         # Central FSM brain — stair climbing state machine
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
- [x] State machine design and implementation (FSM with 6 states)
- [x] `robot_controller` node — active segment logic, level tracking, lift coordination
- [x] Stepper dead reckoning — step count → vertical displacement
- [x] Stair height scouting — front stepper measures each stair on first lift
- [x] Staircase simulator (`staircase_publisher`) for closed-loop FSM testing
- [x] Dead reckoning cutoff — segment_length-based halt when no stair detected
- [x] Landing detection — `front_at_top` flag via ultrasonic > 300 signal
- [ ] Gazebo simulation — virtual laundry_dt mirroring physical state
- [ ] rosbridge integration — live ESP32 hardware data into ROS 2 topics
- [ ] AWS S3 sensor data logging pipeline
- [ ] Computer vision node — stair geometry detection via ESP32-CAM
- [ ] Going down stairs

---

## Known Limitations & Future Work

- Camera-based stair geometry detection not yet implemented — currently relies purely on ultrasonic distance thresholds and stepper dead reckoning
- Stair height scouting currently uses only the front stepper; mid and back rely on the front's stored measurements rather than independently measuring
- Downstairs navigation hardware is in place (downward-facing ultrasonics per segment) — descent algorithm not yet designed
- Drum detach/attach mechanism is mechanical — software integration pending hardware build
- `robot_controller2.py` is an in-progress experimental refactor and not part of the active system

---

## 🔗 Context

Built as part of a broader exploration into autonomous systems and digital twin infrastructure. The architecture here — hardware abstraction nodes, reactive per-segment controllers, FSM-based behavior, simulation-first validation — mirrors the development workflows used in production autonomous vehicle stacks.

---

## 👤 Author

**Ralph Hyacinthe** · [LinkedIn](https://linkedin.com/in/ralphhyacinthe/) · [GitHub](https://github.com/RHy910)