# robot_manipulation
Created in the robot manipulation class practice of the Department of Mechanical Engineering, Graduate School of Engineering, The University of Tokyo.

# DEMO
## video
https://user-images.githubusercontent.com/51389556/150464360-6f2788f1-8791-470a-8df9-c5a96158f811.mp4

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
 ├  launch：launch ファイルの格納場所 
 ├  nodes：ノード用の python ファイルの格納場所 
 ｜├  image2position.py：カメラ画像からパック位置の推定（担当：野田） 
 ｜├  position_predictor.py：過去のパック位置を基にした未来の軌道の予測（担当：青野） 
 ｜└  dobot_orbit_decider.py：パックを打ち出す動作（担当：田中） 
 ├  scripts：編集中の python ファイルの格納場所 
 ├  msg：topic の型の格納場所 
 └  src 
  └  robot_manipulation：python の自作ライブラリの格納場所 
   └  pydobot_ros.py：Dobot 動作用のライブラリ（担当：石井）
```

# Author
* [Ishii Shuto](https://github.com/shuto1441):全体システムのROSパッケージ化
* [Noda Masaki](https://github.com/masakinoda111):パックの画像認識(image2position.py)
* [Aono Kota](https://github.com/KotaAono):画像認識結果からパックの軌道予測(position_predictor.py)
* [Tanaka Akihisa](https://github.com/akihisa1128):軌道予測からDobotにパックを打たせる(dobot_orbit_decider.py)

# Reference
https://afrel.co.jp/dobot/

https://github.com/shuto1441/dobot_ros
