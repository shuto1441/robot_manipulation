# robot_manipulation
Created in the robot manipulation class practice of the Department of Mechanical Engineering, Graduate School of Engineering, The University of Tokyo.

# DEMO
## video
https://user-images.githubusercontent.com/51389556/150335664-d0772b72-b247-4630-9401-6826c35cf384.mp4

## ROS topic graph
<img width="917" alt="キャプチャ" src="https://user-images.githubusercontent.com/51389556/149461283-5f5c8c91-7611-4b17-ac87-ed33f03b04c5.PNG">


# Requirement
* Ubuntu 20.04
* ROS noetic

# Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/shuto1441/dobot_ros.git
git clone https://github.com/shuto1441/robot_manipulation.git
cd ../
catkin build
```

# Usage
```bash
roscore
roslaunch robot_manipulation nodes.launch
rosrun robot_manipulation image2position.py
rosrun robot_manipulation position_predictor.py
rosrun robot_manipulation visualizer.py
rosrun robot_manipulation dobot_orbit_decider.py
```
# Note
## Directory Layout
```
robot_manipulation：パッケージ
　├ launch：launchファイルの格納場所
　├ nodes：ノード用のpythonファイルの格納場所
　├ scripts：編集中のpythonファイルの格納場所
　├ msg：topicの型の格納場所
　├ src
　　└ robot_manipulation：pythonの自作ライブラリの格納場所
```

# Author
* [Ishii Shuto](https://github.com/shuto1441):全体システムのROSパッケージ化
* [Noda Masaki](https://github.com/masakinoda111):パックの画像認識(image2position.py)
* [Aono Kota](https://github.com/KotaAono):画像認識結果からパックの軌道予測(position_predictor.py)
* [Tanaka Akihisa](https://github.com/akihisa1128):軌道予測からDobotにパックを打たせる(dobot_orbit_decider.py)

# Reference
https://afrel.co.jp/dobot/

https://github.com/shuto1441/dobot_ros
