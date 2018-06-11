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

# Parte 3
Per la realizzazione del punto 3 è stato implementato all'interno del file ORB_SLAM2/src/System.cc la seguente porzione di codice
```
void System::SaveMapPoints(const string &filename) {
    cout << endl << "Saving map points to " << filename << " ..." << endl;

    vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpMPs.size(); i++) {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        cv::Mat MPPositions = pMP->GetWorldPos();

        f << setprecision(7) << " " << MPPositions.at<float>(0) << " " << MPPositions.at<float>(1) << " " << MPPositions.at<float>(2) << endl;
    }

    f.close();
    cout << endl << "Map Points saved!" << endl;

}
```
[Link del Codice](https://github.com/raulmur/ORB_SLAM/issues/5)

Tale funzione permette di salvare i punti dell'algoritmo di Slam in un file .txt 
> MapPointsSave.txt
<br>
Il prototipo di funzione viene aggiunto:
<br>
ORB_SLAM2/include/System.h
```
void SaveMapPoints(const string &filename);
```

ORB_SLAM2/Examples/ROS/ORB_SLAM2/src/ros_stereo.cc
```
SLAM.SaveMapPoints("MapPointsSave.txt");
```
<br>
A questo punto il file .txt deve essere convertito in un file .pcd, per questo è stato implementato il codice <br>
> pcd_write.cc
Tale file si occuperà di prendere in input un file .txt e convertire i punti estratti, in seguito all'esecuzione della bag,
e convertirli in .pcd

```
// reading a text file
#include <iostream>
#include <fstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace boost;
using namespace std;
int main () {
  string line;
  char_separator<char> sep(" ");
  size_t i = 0;
  int c = 0;
  
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 8718;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  
  
  
  
  ifstream myfile ("MapPointsSave.txt");
  if (myfile.is_open())
  {
    while ( myfile.good() )
    {
      getline (myfile,line);
      tokenizer< char_separator<char> > tokens(line, sep);
      BOOST_FOREACH (const string& t, tokens) {
        cout << t << endl;
	    if (c == 0) {
		   cloud.points[i].x = std::stof(t);
		}
		else if (c == 1) {
		   cloud.points[i].y = std::stof(t);
		}
		else if (c == 2) {
		   cloud.points[i].z = std::stof(t);
		}
	  c++;
	  }	
	  c = 0;
	  i++;
      //cout << line << endl;
    }
    pcl::io::savePCDFileASCII ("pointcloud.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to pointcloud.pcd." << std::endl;
    /*for (size_t i = 0; i < cloud.points.size (); ++i) {
       std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
    }*/
    myfile.close();
  }
  else cout << "Unable to open file"; 
  return 0;
}
```

Per la generazione del flie .pcd, eseguire il seguente comando <br>
> $ ./pcd_write 

Per la visualizzazione della point_cloud generata, eseguire il seguente comando <br>
> $ pcl_viewer pointcloud.pcd

![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/poincloud.png)




















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





