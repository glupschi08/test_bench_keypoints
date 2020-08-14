#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/keypoints/iss_3d.h>

#include "keypoints.h"

/// compute point cloud resolution
///
/// cloud       -- input point cloud
///
double compute_cloud_resolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); ++i) {
        if (! pcl_isfinite ((*cloud)[i].x)) {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2) {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0) {
        res /= n_points;
    }
    //pcl::console::print_value( "compute_cloud_resolution: %.3f\n",res );
    return res;
}


/// Harris3d keypoints detector
///
/// cloud       -- input point cloud
///
pcl::PointCloud<pcl::PointXYZI>::Ptr harris3d( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::ResponseMethod method, float set_radius, float set_radius_search ) {
    pcl::console::TicToc tt;
    tt.tic();

    // detector
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr harris_detector(
            new pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>( method ) );

    harris_detector->setNonMaxSupression( true );
    harris_detector->setRadius( set_radius );//orig 0.03f   worked with 0.45 for CURVATURE
    harris_detector->setRadiusSearch( set_radius_search );//orig 0.03f   worked with 0.45 for CURVATURE


    pcl::PointCloud<pcl::PointXYZI>::Ptr kpts( new pcl::PointCloud<pcl::PointXYZI>() );

    harris_detector->setInputCloud( cloud );
    harris_detector->compute( *kpts );
    double t = tt.toc();
    std::string method_name;
    switch( method ) {
        case pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZ>::HARRIS:
            method_name = "HARRIS";
            break;
        case pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZ>::TOMASI:
            method_name = "TOMASI";
            break;
        case pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZ>::NOBLE:
            method_name = "NOBLE";
            break;
        case pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZ>::LOWE:
            method_name = "LOWE";
            break;
        case pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZ>::CURVATURE:
            method_name = "CURVATURE";
            break;
    }
    pcl::console::print_value("Harris3D_%s %.3f keypoints %d\n", method_name.c_str(), t, (int)kpts->size() );

    return kpts;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr iss3d( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int SalientRad_muliplier, int NonMaxMultiplier, double Threshold21, double Threshold32, int setMinNeighbors, int setNumberOfThreads ) {
    pcl::console::TicToc tt;
    tt.tic();
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZRGB>());

    double cloud_resolution = compute_cloud_resolution( cloud );

    iss_detector.setSearchMethod (tree);

    iss_detector.setSalientRadius (SalientRad_muliplier * cloud_resolution);
    iss_detector.setNonMaxRadius (NonMaxMultiplier * cloud_resolution);

    iss_detector.setThreshold21 (Threshold21);//0.975
    iss_detector.setThreshold32 (Threshold32);
    iss_detector.setMinNeighbors (setMinNeighbors);
    iss_detector.setNumberOfThreads (setNumberOfThreads);
/*
    iss_detector.setSalientRadius (6 * cloud_resolution);
    iss_detector.setNonMaxRadius (4 * cloud_resolution);

    iss_detector.setThreshold21 (0.99);//0.975
    iss_detector.setThreshold32 (0.99);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (1);
    */
    iss_detector.setInputCloud (cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts( new pcl::PointCloud<pcl::PointXYZRGB>() );
    iss_detector.compute(*kpts);

    double t = tt.toc();
    pcl::console::print_value("ISS3D %.3f keypoints %d\n", t, (int)kpts->size() );

    return kpts;
}
