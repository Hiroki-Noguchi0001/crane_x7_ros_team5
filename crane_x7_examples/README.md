[English](README.en.md) | [日本語](README.md)

# crane_x7_examples

このパッケージは、オリジナル [https://github.com/rt-net/crane_x7_ros/tree/master/crane_x7_examples] を千葉工業大学 2020年度 設計製作論3 知能コース 5班が改変したものです。

---
## CRANE-X7のROSパッケージをインストール

```
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/RobotDesign3-Team5/crane_x7_ros_team5.git
```

---
## intelRealSenseの環境構築  
- RealSense-rosのインストール  
  - 詳しくは[こちら](https://github.com/IntelRealSense/realsense-ros)から.  
  - [こちら](https://demura.net/robot/16525.html)も参考になります.  
---
## 実機を使う場合
- ### モデルの配置
  実機で動作確認を行う場合，以下の図のようにモデルを配置してください．
  - 配置図
    <img src="![haitizu](https://user-images.githubusercontent.com/53966271/103184920-68604f00-48fd-11eb-89c3-a981ffacea60.jpg)
" width="640px">
  - 実際の配置
    <img src="https://user-images.githubusercontent.com/53966390/102636762-66c5b700-4198-11eb-89b8-87cf2d557c3d.png" width="640px">
  
- ### 実機の動かし方
  - CRANE-X7をUSB接続し，下記のコマンドでアクセス権を変更してください．
    ``` 
    $ sudo chmod 666 /dev/ttyUSB0
    ```
  - 動作確認をする場合は下記のコマンドを実行してから，プログラムを実行してください．
    ```
    $ roslaunch crane_x7_bringup demo.launch fake_execution:=false
    ```

---
## gazeboを使用する場合
  - ### gazeboモデルのインストール
    下記の方法でbashを実行してください．
    ```
    $ cd ~/catkin_ws/src/crane_x7_ros_team5
    $ ./gazebo.bash
    ```
  - ### gazebo起動方法
    下記のコマンドでgazeboを起動してください．
    ```
    $ roslaunch crane_x7_gazebo crane_x7_with_table.launch
    ```
---
## RealSenseを使用する場合 
  - ### 実行方法
    下記コマンドでintelRealSenseを起動し画像処理を開始する
    ```
    $ roslaunch realsense2_camera rs_camera.launch
    $ rosrun crane_x7_examples image
    ```
---
## プログラム
  - ### 実行方法
    下記のコマンドでプログラムを実行してください．(RealSenseを使用する場合は上記コマンドを先に実行してください.)
    ```
    $ roslaunch crane_x7_examples team5.launch
    ```

  - ### 一連の流れ

    1.  #### お辞儀
        - [greet.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/greet.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/greet.gif" width="320px">
      
    2.  #### 悪さ
        - [artifice.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/artifice.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/artifice.gif" width="320px">
      
    3.  #### 内容を確認
        - [check.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/check.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/check.gif" width="320px">
      
    4.  #### ハンコを跳ね除ける
        - [grab_release.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/grab_release.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/exclusion.gif" width="320px">
      
    5.  #### ハンコを掴む
        - [detect_seal.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/detect_seal.py)
          - [color_detection.cpp](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/color_detection.cpp)
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/kai_detect.gif" width="320px">
      
    6.  #### ハンコに朱肉をつける
        - [push_check.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/push_check.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/pushcheck.gif" width="320px">
      
    7.  #### 捺印する
        - [seal.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/seal.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/seal.gif" width="320px">
      
    8.  #### はんこを拭く
        - [wipe.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/wipe.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/wipe.gif" width="320px">
      
    9.  #### ハンコを戻す
        - [grab_release.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/grab_release.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/release.gif" width="320px">
      
    10. #### ガッツポーズ
        - [guts_pose.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/guts_pose.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/gutspose.gif" width="320px">
