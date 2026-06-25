# P2 ArUco Docking Automated Testing Report

This document records the specifications and results of the automated tests for verifying the P2 ArUco docking module.

## Test Environment
- **OS**: WSL Ubuntu 22.04
- **ROS 2**: Humble Hawksbill
- **Tested Module**: `src/storagy_hide/storagy_hide/aruco_dock.py`
- **Test File**: `tools/test_p2_aruco.py`

## Test Cases

### 1. ArUco ID 0 Detection
- **Description**: Verifies that the ArUco marker with ID 0 can be successfully recognized from an image stream.
- **Method**: Generates a 640x480 canvas featuring a drawn 4x4 ArUco marker with ID 0, feeds it into `_on_image()`, and asserts that `_tag_visible` transitions to `True`.

### 2. Pose and P-control Command Verification
- **Description**: Verifies the P-control velocity output (Twist) at $x = +0.10\text{ m}$, $z = 1.00\text{ m}$ given a `stop_distance` of $0.40\text{ m}$.
- **Method**: Mocks `cv2.solvePnP` to yield exactly the defined pose. Asserts that the published Twist cmd_vel matches:
  - $linear.x = 0.10\text{ m/s}$ (calculated $0.21\text{ m/s}$, clipped to maximum velocity $0.10\text{ m/s}$)
  - $angular.z = -0.18\text{ rad/s}$ (calculated $-1.8 \times 0.10 = -0.18\text{ rad/s}$)

### 3. Dock Done Once and State Gate
- **Description**: Verifies that `/hide/dock_done=True` is only published exactly once upon successful docking, and that no cmd_vel command is sent under the `FREEZE` state.
- **Method**:
  - Mocks `solvePnP` with a terminal pose ($z = 0.41\text{ m}$, $x = 0.02\text{ m}$), sends two consecutive frames, and asserts that `/hide/dock_done` publication count is exactly `1`.
  - Sets the robot state to `FREEZE` and asserts that no further `cmd_vel` messages are published.

## Verification Execution
Run command:
```bash
wsl bash -c "source /opt/ros/humble/setup.bash && python3 tools/test_p2_aruco.py"
```

### Execution Output:
```
[INFO] [1782366179.527265275] [hide_aruco_dock]: R2 ArucoDock 시작 (target_id=0, stop=0.4m, cv2=True)
[INFO] [1782366179.786906816] [hide_aruco_dock]: ArUco tag detected: id=0
[INFO] [1782366179.820643399] [hide_aruco_dock]: ArUco 도킹 완료: z=0.38m, x=-0.000m
[INFO] [1782366179.879700927] [hide_aruco_dock]: R2 ArucoDock 시작 (target_id=0, stop=0.4m, cv2=True)
[INFO] [1782366179.915177793] [hide_aruco_dock]: ArUco tag detected: id=0
[INFO] [1782366179.997241314] [hide_aruco_dock]: R2 ArucoDock 시작 (target_id=0, stop=0.4m, cv2=True)
[INFO] [1782366180.022950339] [hide_aruco_dock]: ArUco tag detected: id=0
[INFO] [1782366180.027602798] [hide_aruco_dock]: ArUco 도킹 완료: z=0.41m, x=+0.020m
----------------------------------------------------------------------
Ran 3 tests in 2.057s

OK

[P2 TEST 1/3] PASS - ArUco ID 0 detection

[P2 TEST 2/3] PASS - Pose and P-control command

[P2 TEST 3/3] PASS - Dock done once and state gate
[P2 SUMMARY] 3/3 PASS
```
