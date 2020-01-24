slack連携a

# 2020スペースブローブコンテストに向けて

## 概要

 - 基本的にここに全部上げていく、CADもコードも回路図も

## リスト

 - rosベースのシミュレータ

 - cadベースの設計図

 - semanticかobject detectionの学習のためのデータ

 - 確率ロボティクスと機械学習の融合

 - lidarの購入

 - jetobotの購入

 - 環境構築（docker？aws?）

 - 役割分担


## やること

 - 12月1月で確率ロボティクスをマスター(yamano)

 - 12月1月で環境構築を済ませる(endo)

 - 2月から3月末までjetbotで遊ぶ(以下yamano&endo)

 - 4月から6月まで設計

 - 7月組み立て完了

 - 8月テスト

 - 9月本番


## ROSコマンド

  ``` sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' ```

  ``` sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 ```

  ``` sudo apt update ```

  ``` sudo apt install ros-melodic-desktop-full ```

  ``` sudo rosdep init ```

  ``` rosdep update ```

  ``` echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc ```

  ``` source ~/.bashrc ```

  ``` mkdir -p ~/catkin_ws/src ```

  ``` cd ~/catkin_ws/src ```

  ``` catkin_init_workspace ```

  ``` cd ~/catkin_ws/ ```

  ``` catkin_make ```

  ``` echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc ```
