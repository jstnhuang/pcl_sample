#ifndef PCL_SAMPLE_PROCESSOR_H
#define PCL_SAMPLE_PROCESSOR_H

// See processor_node.cpp for explanations of what each header provides.
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"

namespace pcl_sample {
// Contains methods for processing a point cloud.
class Processor {
 public:
  // Computes the average point in the given point cloud, and stores the result
  // in the given PointXYZRGB.
  static void Average(const pcl::PointCloud<pcl::PointXYZRGB> cloud,
    pcl::PointXYZRGB* average);
  // This type signature is an example of a particular C++ convention used by
  // some. Input parameters are first, followed by outputs. Inputs are generally
  // const references or const pointers, indicating that they cannot be changed.
  // Small parameters like ints are passed in by value as usual. Outputs are
  // usually pointers, indicating that they may change. A function that has just
  // one small output can return it as usual.
};
};

#endif
