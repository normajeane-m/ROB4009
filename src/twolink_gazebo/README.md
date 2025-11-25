-필수 패키지 설치

sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-ros2controlcli

-실행

터미널1: ros2 launch twolink_gazebo gazebo.launch.py

터미널2: ros2 run twolink_gazebo joint_publisher.py

-테스트

터미널3: ros2 run twolink_gazebo dummy_data_publisher.py 
