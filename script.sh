#!/bin/bash

# compilazione ed esecuzione
cd build/
cmake ..
cmake --build .
cd ..
./cluster_extraction pointcloud.pcd

# stringa comando pcl_viewer con argomenti cloud_cluster*
CONCAT="pcl_viewer"

# ordina tutti i file cloud trovati e li salva dentro cloud_cluster.txt        
for i in $( ls cloud*pcd | sort -V); do

   echo "$i" >> cloud_cluster.txt
done

# concatena ogni file cluster nella stringa CONCAT
for line in $(cat cloud_cluster.txt); do

	CONCAT+=" $line" 
done

# esegue il comando concat
eval "$CONCAT"

# rimuove i file 
rm cloud_cluster*
