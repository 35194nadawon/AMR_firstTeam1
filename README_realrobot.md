# 실제 로봇(storagy) 접속 메모

storagy 실하드웨어로 SLAM / Navigation 실습할 때의 접속·실행 순서.

1. 윈도우 파워쉘(다른 터미널)에서 wsl 실행: `wsl -d Ubuntu-22.04`
2. wsl에서 로봇 연결: `ssh storagy@192.168.0.84 -XC`  (패스워드: `123412`)
3. 런치:
   ```bash
   cd Desktop/storagy_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

### 터미널1 — 맵 띄우는 런치
```bash
ros2 launch storagy bringup.launch.py
```

### 터미널2 — 라이다/카토그래퍼
```bash
ros2 param set /amcl tf_broadcast false
ros2 launch storagy cartographer.launch.py
```

### 터미널3 — 텔레옵
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
