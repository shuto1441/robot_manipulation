# robot_manipulation

## Directory Layout
```
robot_manipulation：パッケージ
　├ launch：launchファイル(複数のスクリプトを同時に実行できる)の格納場所
　├ nodes：ノード用のpythonファイル（ROS形式で記述済み）の格納場所
　├ scripts：編集中のpythonファイル（ROS未対応）の格納場所
　├ msg：topicの型の格納場所
　├ src
　　└ robot_manipulation：pythonの自作ライブラリの格納場所
```
## Install
```
cd ~/catkin_ws/src
git clone https://github.com/shuto1441/dobot.git
git clone https://github.com/shuto1441/robot_manipulation.git
cd ../
catkin build
```

## Ubuntu20.04 
Creating symbolic links to OS standard python commands

```
sudo apt install python-is-python3
```

## Reference
https://github.com/shuto1441/dobot