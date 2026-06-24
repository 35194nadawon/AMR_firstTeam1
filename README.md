# AMR_firstTeam1
자율주행전문가교육(xyz아카데미) 팀 프로젝트

**storagy로 SLAM, Navigation 실습**
1. 윈도우 파워쉘(다른 터미널)에서 wsl입력 실행: wsl -d Ubuntu-22.04
2. wsl에서 로봇 연결: ssh storagy@192.168.0.84 -XC / 패스워드: 123412
3. 런치
   > cd Desktop/storagy_ws
   >
   >  source /opt/ros/humble/setup.bash
   >
   > source install/setup.bash

###### 터미널1: 
맵 띄우는 런치 파일
> ros2 launch storagy bringup.launch.py
###### 터미널2:
라이다 실행/  cartographer 실행
> ros2 param set /amcl tf_broadcast false
>
> ros2 launch storagy cartographer.launch.py
###### 터미널3:
- 텔레옵 실행
> ros2 run teleop_twist_keyboard teleop_twist_keyboard 

