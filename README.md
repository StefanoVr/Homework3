# HOMEWORK3 
Università degli Studi di Verona - Informatica 2017/2018
# Istruzioni
1) Installazione di ORB_SLAM2 <a href="https://github.com/raulmur/ORB_SLAM2">Link</a> 
2) Scaricare: <a href="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag"> V1_01_easy<a> (bag)
3) Lanciare i seguenti comandi da Terminale:
```
chmod +x build_ros.sh 

./build_ros.sh
```
```
roscore

rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true

rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw 
```

# Autori
Francesco Fontana - VR081502   <br>
Stefano Veraldi - VR378035 <br> <br>

# Requisiti & Links
- <a href="https://www.ubuntu-it.org/download">Ubuntu 16.04</a>
- <a href="https://github.com/stevenlovegrove/Pangolin">Pangolin</a>
- <a href="https://opencv.org/releases.html">OpenCV 3.2</a>
- ROS (Robot Operative System)
- <a href="http://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen3 3.3.4</a>
- <a href="https://github.com/dorian3d/DBoW2">DBoW2</a> 
- <a href="https://github.com/RainerKuemmerle/g2o">g2o</a> 
<br>





