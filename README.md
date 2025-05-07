# ✈️ Draw "S" Flight Mode for PX4 ROS 2

This custom PX4 flight mode traces a blocky **"S" shape** in the **NED (North-East-Down)** coordinate frame using discrete waypoint navigation.

## 📍 NED Waypoint Path

| Segment     | Relative Position (x, y, z) | Description           |
|-------------|-----------------------------|-----------------------|
| **Start**   | `(0, 0, 0)`                 | Starting point        |
| **West1**   | `(0, -25, 0)`              | Move West 25m         |
| **South1**  | `(-25, -25, 0)`            | Move South 25m        |
| **East**    | `(-25, 0, 0)`              | Move East 25m         |
| **South2**  | `(-50, 0, 0)`              | Move South 25m        |
| **West2**   | `(-50, -25, 0)`            | Move West 25m (End)   |

## 🧭 Coordinate Frame (NED)

- **North (x)**: Positive forward, **South is negative**
- **East (y)**: Positive right, **West is negative**
- **Down (z)**: Positive down (z is kept at 0 for 2D movement)

## 🔁 Path Diagram

(0, 0) ----> (0, -25) ← West1
| |
v |
(-25, -25) <---- (-25, 0) ← South1, East
| |
v |
(-50, 0) ----> (-50, -25) ← South2, West2 (End)


## ⚙️ Flight Mode States

The mode transitions through these states:
1. **SettlingAtStart** – Ensures starting point is stable.
2. **West1** – Move 25m West.
3. **South1** – Move 25m South.
4. **East** – Move 25m East.
5. **South2** – Move 25m South.
6. **West2** – Move 25m West.
7. **Done** – Finish the sequence.

## 🔧 How to Build

```bash
cd ~/custom_flightmode
colcon build --packages-select draw_s_flight_mode --allow-overriding px4_ros2_cpp

▶️ How to Run

source ~/custom_flightmode/install/setup.bash
ros2 run draw_s_flight_mode draw_s

📋 Logging Output

[INFO] Starting West1 from (0.0, 0.0)
[INFO] Reached West1 at (0.0, -25.0), moving to South1
[INFO] Reached South1 at (-25.0, -25.0), moving to East
...
[INFO] Reached West2 at (-50.0, -25.0), finished

📡 Visualize

Use a simulator like Gazebo or QGroundControl to monitor the flight path and verify the S-shaped trajectory.
📝 Notes

    Speed: Travels at 3.0 m/s horizontally.

    Yaw: Adjusted per segment to face the direction of movement.

    Trajectory Type: Uses discrete Goto setpoints (not smooth splines).

    Custom Start Point: Always relative to the current NED pose on activation.

🛠 Future Improvements

    Use Bezier or spline interpolation for smooth S-curves.

    Add altitude variation for a 3D path.

    Tune transition thresholds for precision in real-world flights.
