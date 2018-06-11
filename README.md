# HOMEWORK3 
Università degli Studi di Verona

Corso di Robotica - Laboratorio Ciberfisico

AA 2017/2018

# ISTRUZIONI
1) Installazione di ORB_SLAM2 <a href="https://github.com/raulmur/ORB_SLAM2">Link</a> 

# Parte 1 & 2

2) Scaricare: <a href="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag"> V1_01_easy<a> (bag)
3) Lanciare i seguenti comandi da Terminale:
```
chmod +x build_ros.sh 
./build_ros.sh
```
	
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

Viene modificato il CMakeList.txt implementando al suo interno le seguenti righe per la generazione dei file eseguibili pcd_write e cluster_extraction

```
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cc)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_extraction)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cluster_extraction cluster_extraction.cc)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
```

NB: Ricompilare la directory ORB_SLAM2

```
cd build/
cmake ..
cmake --build .
```

A questo punto il file .txt deve essere convertito in un file .pcd, per questo è stato implementato il codice

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

- Rieseguire la 'rosbag' per la generazione del file .txt

- Per la generazione del flie .pcd, eseguire il seguente comando <br>
> $ ./pcd_write 


# Parte 4

Per la visualizzazione della point_cloud generata, eseguire il seguente comando <br>
> $ pcl_viewer pointcloud.pcd

![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/poincloud.png)

## Parte 4.2
Per clusterizzare il file .pcd ottenuto è stato seguito il codice al [link](http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)

E' stato creato il file <br>
> cluster_extraction.cc <br>

per eseguire l'estrazione Cluster Euclidea e generare i file cloud_cluster.pcd <br>

```
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


int main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    std::cout<<"Couldn't read the file "<<argv[1]<<std::endl;
    return (-1);
  }
  std::cout << "Loaded pcd file " << argv[1] << " with " << cloud->points.size() << std::endl;

  
  std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.001); 

  int nr_points = (int) cloud_filtered->points.size();
  while (cloud_filtered->points.size() > 0.9 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);
  
  double tolerance = 0.15; // 15cm
  std::vector<pcl::PointIndices> cluster_indices;
  unsigned int min_pts_per_cluster = 2; 
  unsigned int max_pts_per_cluster = (cloud_filtered->points.size()); 
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tolerance); 
  ec.setMinClusterSize(min_pts_per_cluster);
  ec.setMaxClusterSize(max_pts_per_cluster);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
 
  int j = 0;
  
  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {	  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
    
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    std::stringstream ss;
    ss << "./cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); //*
    j++;
  }
  return (0);
}
```


In seguito a vari tentativi sono stati modificati i valori:
> seg.setMaxIteration(100);  <br>
> seg.setDistanceThreshold(0.001); <br>
> cloud_filtered->points.size() > 0.9 * nr_points <br>
> tollerance = 0.15; // 15 cm <br>
> min_pts_per_cluster = 2; <br>
> max_pts_per_cluster = (cloud_filtered->points.size()) <br>

Eseguire il seguente comando per generare e visualizzare la point_cloud clasterizzata
```
$ ./script.sh
```

Generando così la point_cloud riportata di seguito

![alt text](https://github.com/StefanoVr/Homework3/blob/master/images/cluster_extraction.png)

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
