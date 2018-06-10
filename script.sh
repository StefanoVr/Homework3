#!/bin/bash

CONCAT="pcl_viewer"
        
for i in $( ls cloud*pcd | sort -V); do
   #echo item: $i
   echo "$i" >> test.txt
done

for line in $(cat test.txt); do
	#echo "$line"
	CONCAT+=" $line" 
done

#echo "$CONCAT"

eval "$CONCAT"

rm test.txt
rm cloud_cluster*
