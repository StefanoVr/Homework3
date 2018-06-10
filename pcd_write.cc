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
