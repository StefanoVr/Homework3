# HOMEWORK3 
Università degli Studi di Verona

Corso di Robotica - Laboratorio Ciberfisico

AA 2017/2018

# Istruzioni
1) Installazione di ORB_SLAM2 <a href="https://github.com/raulmur/ORB_SLAM2">Link</a> 
2) Scaricare: <a href="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag"> V1_01_easy<a> (bag)
3) Lanciare i seguenti comandi da Terminale:
```
chmod +x build_ros.sh 

./build_ros.sh
```

# Parte 1 & 2
Esecuzione di ORB_SLAM2 e della Bag V1_01_easy.bag
```
$ roscore
```
![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/1%20-%20roscore.png)

```
$ cd ORB_SLAM2/
$ rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
```
![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/2%20-%20rosrun.png)

```
$ rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
```
![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/3%20-%20rosplay.png)
<hr>
<strong>Volo e Acquisizione dei dati</strong>


![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/4%20-%20fly1.png)
![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/fly2.png)
![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/fly3.png)
![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/fly4.png)
<hr>
<strong>Fine del Volo e dell'acquisizione dei dati</strong>


![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/5%20-%20end-fly.png)



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





