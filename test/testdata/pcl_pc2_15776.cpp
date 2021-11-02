/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: example_OrganizedPointCloud.cpp 4258 2012-02-05 15:06:20Z daviddoria $
 *
 */

// STL
#include <iostream>
#include <fstream>
#include <assert.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/conversions.h>


char* readdata (int* length) {

  std::ifstream is ("_PCLPointCloud2_15776.dat", std::ifstream::binary);
  // int length = 0
  if (is) {
    // get length of file:
    is.seekg (0, is.end);
    *length = is.tellg();
    is.seekg (0, is.beg);

    char * buffer = new char [*length];

    std::cout << "Reading " << *length << " characters... ";
    // read data as a block:
    is.read (buffer,*length);

    if (is)
      std::cout << "all characters read successfully." << std::endl;
    else
      std::cout << "error: only " << is.gcount() << " could be read" << std::endl;
    is.close();

    // ...buffer contains the entire file...

    // delete[] buffer;
      return buffer;
  }
  char* dummy;
  return dummy;
}

int 
main ()
{
  // load binary data
  int length = 0;

  // load test data from file, and check its length
  char* buffer = readdata(&length);
  std::cout << std::endl << "length is " << length << std::endl;
  assert(length == 15776);

    // hand generate PCLPointCloud2
  pcl::PCLPointCloud2 pc2;

  // load data first
  for (int i = 0; i < length; i++) {
    pc2.data.push_back(buffer[i]);
  }

  unsigned char* buffer_ = reinterpret_cast<unsigned char*>(buffer);
  // sizeof arr
  for (int i = 0; i < 5; i++) {
    printf(" %02x", buffer_[i]);
    printf("&%02x", pc2.data[i]);
  }
  std::cout << " ... ";
  for (int i = length-6; i < length ; i++) {
    printf(" %02x", buffer_[i]);
    printf("&%02x", pc2.data[i]);
  }

  std::cout << std::endl << "pc2.data size is " << pc2.data.size() << std::endl;


  pcl::PCLHeader header;
  header.seq = 92147;
  header.frame_id = "velodyne";
  header.stamp = 1603389805080336;

  pcl::PCLPointField pf_x;
  pf_x.name = "x";
  pf_x.offset = 0x00000000;
  pf_x.datatype = pcl::PCLPointField::FLOAT32;
  pf_x.count = 1;
  pc2.fields.push_back(pf_x);

  pcl::PCLPointField pf_y;
  pf_y.name = "y";
  pf_y.offset = 0x00000004;
  pf_y.datatype = pcl::PCLPointField::FLOAT32;
  pf_y.count = 1;
  pc2.fields.push_back(pf_y);

  pcl::PCLPointField pf_z;
  pf_z.name = "z";
  pf_z.offset = 0x00000008;
  pf_z.datatype = pcl::PCLPointField::FLOAT32;
  pf_z.count = 1;
  pc2.fields.push_back(pf_z);

  pcl::PCLPointField pf_i;
  pf_i.name = "intensity";
  pf_i.offset = 0x00000010;
  pf_i.datatype = pcl::PCLPointField::FLOAT32;
  pf_i.count = 1;
  pc2.fields.push_back(pf_i);

  pc2.header = header;
  pc2.height = 1;
  pc2.width = 493;
  pc2.point_step = 32;
  pc2.row_step = 15776;
  pc2.is_bigendian = false;
  pc2.is_dense = 1;

  std::cout << "pc2 with header" << std::endl;

  // =============================================================================
  // Setup the cloud
  using PointType = pcl::PointXYZ;
  using CloudType = pcl::PointCloud<PointType>;
  CloudType::Ptr cloud (new CloudType);
  
  // do the conversion
  pcl::fromPCLPointCloud2(pc2, *cloud);

  std::cout << "sizeof(PointT) = " << sizeof(PointType) << std::endl;

  // // Make the cloud a 10x10 grid
  // cloud->height = 10;
  // cloud->width = 10;
  // cloud->is_dense = true;
  // cloud->resize(cloud->height * cloud->width);
  
  // Output the (0,0) point
  std::cout << (*cloud)(0,0) << std::endl;
  std::cout << cloud->isOrganized() << std::endl;
  for (int i=0; i< 10; i++)
    std::cout << cloud->points[i] << std::endl;
  // // Set the (0,0) point
  // PointType p; p.x = 1; p.y = 2; p.z = 3;
  // (*cloud)(0,0) = p;
  
  // // Confirm that the point was set
  // std::cout << (*cloud)(0,0) << std::endl;
  
  delete[] buffer;
  
  return (0);
}
