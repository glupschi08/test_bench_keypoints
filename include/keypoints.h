#pragma once
#ifndef KEYPOINTS_H
#define KEYPOINTS_H

#endif
double compute_cloud_resolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr iss3d( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud );
pcl::PointCloud<pcl::PointXYZI>::Ptr harris3d( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::ResponseMethod method, float set_radius, float set_radius_search );


