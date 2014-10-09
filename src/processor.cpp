// Implementation of the processor library.
// See processor.h for documentation.

// See processor_node.cpp for explanations of what each header provides.
#include "pcl_sample/processor.h"

#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"

#include <math.h>

namespace pcl_sample {
void Processor::Average(const pcl::PointCloud<pcl::PointXYZRGB> cloud,
    pcl::PointXYZRGB* average) { 
  double average_x = 0;
  double average_y = 0;
  double average_z = 0;
  long average_r = 0;
  long average_g = 0;
  long average_b = 0;
  for (const pcl::PointXYZRGB& point : cloud.points) {
    // Some points may have NaN values for position.
    if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
      continue;
    }
    average_x += point.x;
    average_y += point.y;
    average_z += point.z;
    average_r += point.r;
    average_g += point.g;
    average_b += point.b;
  }
  average->x = average_x / cloud.points.size();
  average->y = average_y / cloud.points.size();
  average->z = average_z / cloud.points.size();
  average->r = (double) average_r / cloud.points.size();
  average->g = (double) average_g / cloud.points.size();
  average->b = (double) average_b / cloud.points.size();
}
}
