
#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include <ctime>
#include <vector>
#include <sstream>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>


#include <pcl/correspondence.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>


#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

//own added
#include <random>
#include <chrono>
#include "overlap.h"
#include <pcl/keypoints/harris_3d.h>
#include "keypoints.h"
#include <cxxopts.hpp> //for arg input
#include <pcl/common/transforms.h>

#include <fstream> //write to file
#include <cmath> //to get absolute values
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/pcd_io.h>


//new keypoint methods
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/harris_6d.h>

using namespace std;

//nice results with in 7s
//#define normal_radius 5.
//#define feature_radius 5.25


// Hyper parameters
#define LEAF_SIZE .1    //for the sampling of the cloud -> 0.1 no filtering applied

//for computing the normals
#define normal_radius 5//1.2 //0.25  -> very good results with 0.25, 5, 10 and feature radius10.25

//  for compute_PFHRGB_features
// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
#define feature_radius 5.25//3.25  //0.25


#define RANSAC_Inlier_Threshold 3.//1.5 //0.1
#define RANSAC_Iterations 5000
#define CorrRejDist_Maximum_Distance 5
#define ICP_Max_Correspondence_Distance 0.15

// Parameters for sift computation
/*
#define min_scale 0.2
#define nr_octaves 4
#define nr_scales_per_octave 5
#define min_contrast 0.25
*/
 /*
#define min_scale 0.2
#define nr_octaves 4
#define nr_scales_per_octave 5
#define min_contrast 0.25
*/


inline bool exists_file (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

void detect_keypoints_SIFT(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &points,
    pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints_out,
    double min_scale, int nr_octaves, int nr_scales_per_octave, double min_contrast) {

    pcl::SIFTKeypoint <pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;

    // Use a FLANN-based KdTree to perform neighbourhood searches
    pcl::search::KdTree <pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZRGB>);
    sift_detect.setSearchMethod(tree);

    // Set the detection parameters
    sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    sift_detect.setMinimumContrast(min_contrast);

    // Set the input
    sift_detect.setInputCloud(points);

    // Detect the keypoints and store them in "keypoints.out"
    sift_detect.compute(*keypoints_out);
}

std::tuple<uint8_t, uint8_t, uint8_t> jet(double x){
    const double rone = 0.8;
    const double gone = 1.0;
    const double bone = 1.0;
    double r, g, b;

    x = (x < 0 ? 0 : (x > 1 ? 1 : x));

    if (x < 1. / 8.) {
        r = 0;
        g = 0;
        b = bone * (0.5 + (x) / (1. / 8.) * 0.5);
    } else if (x < 3. / 8.) {
        r = 0;
        g = gone * (x - 1. / 8.) / (3. / 8. - 1. / 8.);
        b = bone;
    } else if (x < 5. / 8.) {
        r = rone * (x - 3. / 8.) / (5. / 8. - 3. / 8.);
        g = gone;
        b = (bone - (x - 3. / 8.) / (5. / 8. - 3. / 8.));
    } else if (x < 7. / 8.) {
        r = rone;
        g = (gone - (x - 5. / 8.) / (7. / 8. - 5. / 8.));
        b = 0;
    } else {
        r = (rone - (x - 7. / 8.) / (1. - 7. / 8.) * 0.5);
        g = 0;
        b = 0;
    }
    return std::make_tuple(uint8_t(255.*r), uint8_t(255.*g), uint8_t(255.*b));
}


//does the prescalling for jet -> maps z to [0-1]:[1-0] in the area between 0 and threshold
//e.g. points along a linear line in z direction would get be: blue, green, yellow, red, yellow, green, blue, green,...
std::tuple<uint8_t, uint8_t, uint8_t> stacked_jet(double z, double threshold){
    pcl::PointXYZRGB pointrgb;
    std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
    double r, g, b, val;
    if(z<=0){
        while(z<0){
            z+=threshold;
        }
    }else{
        while(z>threshold){
            z-=threshold;
        }
    }
    if(z>threshold/2){
        z-=(threshold/2);
        val=-((z/(threshold/2))-1);
    }else{
        val=z/(threshold/2);
    }
    return jet(val);
}


void compute_normals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &points,
     pcl::PointCloud <pcl::Normal>::Ptr &normals_out) {

    pcl::NormalEstimation <pcl::PointXYZRGB, pcl::Normal> norm_est;
    // Use a FLANN-based KdTree to perform neighbourhood searches
    norm_est.setSearchMethod(pcl::search::KdTree <pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree <pcl::PointXYZRGB>));

    norm_est.setRadiusSearch(normal_radius);
    norm_est.setInputCloud(points);
    norm_est.compute(*normals_out);
}

void add_noise_normal_distributedvoid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double mean, double stdv, double x_o, double y_o, double z_o, int red, int green, int blue){

    //generate seed and def dist. generator
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> distribution(mean,stdv);

    //add noise
    double avg=mean;
    for (std::size_t i = 0; i < cloud->points.size (); ++i){
        double noise = distribution(generator);
        cloud->points[i].z = cloud->points[i].z + noise + z_o;
        cloud->points[i].x = cloud->points[i].x + x_o;  //add an offset to the x var
        cloud->points[i].y = cloud->points[i].y + y_o;  //add an offset to the x var

        avg=(avg+noise)/2;
    }
    //todo remove after testing for color
    if(red!=0 && green!=0 && blue!=0){
        for (std::size_t i = 0; i < cloud->points.size (); ++i) {
            cloud->points[i].r = red;
            cloud->points[i].g = green;
            cloud->points[i].b = blue;
        }
    }
    std::cout << "avg: " << avg << std::endl;
    std::cout << "normal_distributed noise added("<< mean << ","<< stdv << "):" << std::endl;
}



void compute_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &tgt,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints_src_visualize_temp,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints_tgt_visualize_temp,
    std::string keypoints_meth,
    double min_scale_SIFT, int nr_octaves_SIFT, int nr_scales_per_octave_SIFT, double min_contrast_SIFT,
    float set_radius_harris, float set_radius_search_harris, std::string HarrisRosponseMethod,
    int SalientRad_muliplier_ISS, int NonMaxMultiplier_ISS, double Threshold21_ISS, double Threshold32_ISS, int setMinNeighbors_ISS, int setNumberOfThreads_ISS) {


    // ESTIMATING KEY POINTS
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_src(new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_tgt(new pcl::PointCloud<pcl::PointWithScale>);


    //  COMPUTING NORMALS
    pcl::PointCloud <pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud <pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>);

    compute_normals(src, src_normals);
    compute_normals(tgt, tgt_normals);

    if(keypoints_meth=="ISS"){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_iss_src( new pcl::PointCloud<pcl::PointXYZRGB>() );
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_iss_tgt( new pcl::PointCloud<pcl::PointXYZRGB>() );
        cout << "chosen Method is ISS" << endl;
        keypoints_iss_src=  iss3d(src, SalientRad_muliplier_ISS, NonMaxMultiplier_ISS, Threshold21_ISS, Threshold32_ISS, setMinNeighbors_ISS, setNumberOfThreads_ISS);
        keypoints_iss_tgt=  iss3d(tgt, SalientRad_muliplier_ISS, NonMaxMultiplier_ISS, Threshold21_ISS, Threshold32_ISS, setMinNeighbors_ISS, setNumberOfThreads_ISS);

        pcl::copyPointCloud(*keypoints_iss_src, *keypoints_src);
        pcl::copyPointCloud(*keypoints_iss_tgt, *keypoints_tgt);
        cout << "No of ISS points in the src are " << keypoints_iss_src->points.size() << endl;
        cout << "No of ISS points in the tgt are " << keypoints_iss_tgt->points.size() << endl;
    }else if(keypoints_meth=="Harris"){
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_harris_scr( new pcl::PointCloud<pcl::PointXYZI>() );
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_harris_tgt( new pcl::PointCloud<pcl::PointXYZI>() );
        //float set_radius =1.7,set_radius_search=1.7;
        cout << "chosen Method is Harris3D" << endl;
        //keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
        //keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
        //cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
        //cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;

        cout << "HarrisRosponseMethod: " << HarrisRosponseMethod << endl;
        if (HarrisRosponseMethod=="Harris") {
            keypoints_harris_scr = harris3d(src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS, set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt = harris3d(tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS, set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else if("CURVATURE"){
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::CURVATURE ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::CURVATURE ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else if("NOBLE"){
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::NOBLE ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::NOBLE ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else if("TOMASI"){
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else if("LOWE"){
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else{
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }

        pcl::copyPointCloud(*keypoints_harris_scr, *keypoints_src);
        pcl::copyPointCloud(*keypoints_harris_tgt, *keypoints_tgt);
    }else if(keypoints_meth=="NARF"){

        //not tested
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_narf_scr( new pcl::PointCloud<pcl::PointXYZI>() );
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_narf_tgt( new pcl::PointCloud<pcl::PointXYZI>() );
        cout << "chosen Method is Narf" << endl;

        cout << "No of NARF points in the src are " << keypoints_narf_scr->points.size() << endl;
        cout << "No of NARF points in the tgt are " << keypoints_narf_tgt->points.size() << endl;

        pcl::copyPointCloud(*keypoints_narf_scr, *keypoints_src);
        pcl::copyPointCloud(*keypoints_narf_tgt, *keypoints_tgt);
    }else if(keypoints_meth=="AGAS"){

    }else if(keypoints_meth=="HARRIS6D"){
        /*
        harris6D_detector_src* detector = new HarrisKeypoint6D(HarrisKeypoint::HARRIS);
        harris6D_detector_tgt* detector = new HarrisKeypoint6D(HarrisKeypoint::HARRIS);
        //pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr harris6D_detector_src(new pcl::HarrisKeypoint6D<pcl::PointXYZI);
        //pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr harris6D_detector_tgt(new pcl::HarrisKeypoint6D<pcl::PointXYZRGB);

        harris6D_detector_src->setNonMaxSupression( true );
        harris6D_detector_src->setRadius( set_radius );//orig 0.03f   worked with 0.45 for CURVATURE
        harris6D_detector_src->setRadiusSearch( set_radius_search );//orig 0.03f   worked with 0.45 for CURVATURE

        harris6D_detector_tgt->setNonMaxSupression( true );
        harris6D_detector_tgt->setRadius( set_radius );//orig 0.03f   worked with 0.45 for CURVATURE
        harris6D_detector_tgt->setRadiusSearch( set_radius_search );//orig 0.03f   worked with 0.45 for CURVATURE

        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_harris6d_scr( new pcl::PointCloud<pcl::PointXYZI>() );
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_harris6d_tgt( new pcl::PointCloud<pcl::PointXYZI>() );

        harris6D_detector_src->setInputCloud( src );
        harris6D_detector_tgt->setInputCloud( tgt );
        harris6D_detector_src->compute( *keypoints_harris6d_scr );
        harris6D_detector_tgt->compute( *keypoints_harris6d_tgt );
    */

    }else if(keypoints_meth=="TRAJCOVIC") {

    }else if(keypoints_meth=="SUSAN"){

        pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>* susan3D_src = new  pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>;
        pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>* susan3D_tgt = new  pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>;
        susan3D_src->setInputCloud(src);
        susan3D_tgt->setInputCloud(tgt);
        susan3D_src->setNonMaxSupression(true); //true
        susan3D_tgt->setNonMaxSupression(true);  //true
        //susan3D_src->setNormals(src_normals);   //new
        //
        float set_radius_susan =1.7,set_radius_search_susan=1.7;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_src (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_tgt (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        susan3D_src->setSearchMethod(tree_src);
        susan3D_tgt->setSearchMethod(tree_tgt);
        susan3D_src->setRadius(set_radius_susan);
        susan3D_tgt->setRadius(set_radius_susan);
        susan3D_src->setRadiusSearch(5.);
        susan3D_tgt->setRadiusSearch(5.);

        susan3D_src->setDistanceThreshold (15.0);   //new
        susan3D_tgt->setDistanceThreshold (15.0);   //new
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_susan_scr (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_susan_tgt (new pcl::PointCloud<pcl::PointXYZRGB> ());
        susan3D_src->compute(*keypoints_susan_scr);
        susan3D_tgt->compute(*keypoints_susan_tgt);

        cout << "No of SUSAN points in the src are " << keypoints_susan_scr->points.size() << endl;
        cout << "No of SUSAN points in the tgt are " << keypoints_susan_tgt->points.size() << endl;

        pcl::copyPointCloud(*keypoints_susan_scr, *keypoints_src);
        pcl::copyPointCloud(*keypoints_susan_tgt, *keypoints_tgt);
    }else{
        cout << "chosen Method is SWIFT" << endl;
        detect_keypoints_SIFT(src, keypoints_src, min_scale_SIFT, nr_octaves_SIFT, nr_scales_per_octave_SIFT, min_contrast_SIFT);
        detect_keypoints_SIFT(tgt, keypoints_tgt, min_scale_SIFT, nr_octaves_SIFT, nr_scales_per_octave_SIFT, min_contrast_SIFT);

        cout << "No of SIFT points in the src are " << keypoints_src->points.size() << endl;
        cout << "No of SIFT points in the tgt are " << keypoints_tgt->points.size() << endl;
    }

    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::copyPointCloud(*keypoints_src, *keypoints_src_visualize_temp);
    pcl::copyPointCloud(*keypoints_tgt, *keypoints_tgt_visualize_temp);
}

void keypoint_evaluation(Mapsample& submap_overlap, const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1, const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2){
    double overlap_rate=0, total_overlap_rate=0;
    int keypoint_Overlap_cnt=0, keypoint_Overlaparea_fit_cnt=0, keypoints_1_inOverlap=0, keypoints_2_inOverlap=0;

    //calculate the overlapping points
    //visualize the keypoints off bodyFiltered_1 with lines
    for (size_t i = 0; i < keypoints_2->size(); ++i) {
        for (size_t ii = 0; ii < keypoints_1->size(); ++ii) {
            //check if a coresponding keypoint exists
            //only check x and y since noise and offset is added to the second cloud/submap
            if( (keypoints_2->points[i].x==keypoints_1->points[ii].x) && (keypoints_2->points[i].y==keypoints_1->points[ii].y)){
                keypoint_Overlap_cnt++;
            }
            //is in area and overlapping
            if( (keypoints_1->points[i].x > submap_overlap.minX) && (keypoints_1->points[i].x < submap_overlap.maxX) && (keypoints_1->points[i].y > submap_overlap.minY) && (keypoints_1->points[i].x < submap_overlap.maxY)){
                if( (keypoints_2->points[i].x==keypoints_1->points[ii].x) && (keypoints_2->points[i].y==keypoints_1->points[ii].y)){
                    keypoint_Overlaparea_fit_cnt++; //the points in the overalapping area which have the same coordinates
                }
            }
        }
    }

    for (size_t counter = 0; counter < keypoints_1->size(); ++counter) {
        if( (keypoints_1->points[counter].x > submap_overlap.minX) && (keypoints_1->points[counter].x < submap_overlap.maxX) && (keypoints_1->points[counter].y > submap_overlap.minY) && (keypoints_1->points[counter].x < submap_overlap.maxY)){
            keypoints_1_inOverlap++;
        }
    }
    for (size_t counter = 0; counter < keypoints_2->size(); ++counter) {
        if( (keypoints_2->points[counter].x > submap_overlap.minX) && (keypoints_2->points[counter].x < submap_overlap.maxX) && (keypoints_2->points[counter].y > submap_overlap.minY) && (keypoints_2->points[counter].x < submap_overlap.maxY)){
            keypoints_2_inOverlap++;
        }
    }
    std::cout << "----------------------------------------------------------------"  << std::endl;
    std::cout << "Num of Keypoints_1 in the overlapping area: " << keypoints_1_inOverlap << std::endl;
    std::cout << "Num of Keypoints_2 in the overlapping area: " << keypoints_2_inOverlap << std::endl;
    std::cout << "Total Keypoints 1: " <<  keypoints_1->size() << std::endl;
    std::cout << "Total Keypoints 2: " <<  keypoints_2->size() << std::endl;

    if(keypoints_1_inOverlap<keypoints_2_inOverlap){
        overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1_inOverlap;
    }else{
        overlap_rate=(double) keypoint_Overlap_cnt/ (double) keypoints_2_inOverlap;
    }
    if(keypoints_1->size()<keypoints_2->size()){
        total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1->size();
    }else{
        total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_2->size();
    }

    std::cout << "total Num of Overlapping Keypoints (coordinates): " << keypoint_Overlap_cnt << std::endl;
;
    std::cout << "Num Keypoints_1 in the region of interest fitting Keypoints_2 (coord): " << keypoint_Overlaparea_fit_cnt << std::endl;
    overlap_rate= (double) keypoint_Overlaparea_fit_cnt / (double) keypoints_1_inOverlap;
    std::cout << "Overlapping Rate of Keypoints 1: " << overlap_rate << std::endl;
    overlap_rate= (double) keypoint_Overlaparea_fit_cnt / (double) keypoints_2_inOverlap;
    std::cout << "Overlapping Rate of Keypoints 2: " << overlap_rate << std::endl;
    total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1->size();
    std::cout << "Total Overlapping Rate of Keypoints 1: " << total_overlap_rate << std::endl;
    total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_2->size();
    std::cout << "Total Overlapping Rate of Keypoints 2: " << total_overlap_rate << std::endl;
    std::cout << "----------------------------------------------------------------"  << std::endl;
}


int main(int argc, char** argv) {
    int roation_flag=0, z_rejection_flag=0;
    int red=0, green=0, blue=0, jet_flag=0, grid_flag=0;
    //parse input para
    double validation_radius_in=0.0, validation_radius_out=0.0;
    double noise_offset=0.0, noise_var=0.0, x_o=0.0, y_o=0.0, z_o=0.0,rot_alpha=0.0, rot_betha=0.0, rot_gamma=0.0;
    double visu_dist=0.0, jet_stacking_threshold=30.0;

    //Parameter for Keypoint methods
    double min_scale_SIFT=0.2;
    int nr_octaves_SIFT=4;
    int nr_scales_per_octave_SIFT=5;
    double min_contrast_SIFT =0.25;
    float set_radius_harris =1.7, set_radius_search_harris =1.7;
    int SalientRad_muliplier_ISS=6;
    int NonMaxMultiplier_ISS=4;
    double Threshold21_ISS=0.99, Threshold32_ISS=0.99;
    int setMinNeighbors_ISS=5, setNumberOfThreads_ISS=1;

    cxxopts::Options options("test", "A brief description");
    options.add_options()
            ("e,validation_in", "The radius in which a feature point will count as (exact) correct if not fitting excatly", cxxopts::value<double>(validation_radius_in)->default_value("0.0"))
            ("v,validation_out", "The radius in which a feature point will count as correct if not fitting excatly", cxxopts::value<double>(validation_radius_out)->default_value("0.0"))
            ("q,grid", "set for minimal terminnal output in e.g. grid search ", cxxopts::value<int>(grid_flag)->default_value("0"))

            ("f,output_filename", "filename to save the measurement data in", cxxopts::value<std::string>())
            ("input_filename_1", "input file 1 with multibeam data", cxxopts::value<std::string>())
            ("input_filename_2", "input file 2 with multibeam data", cxxopts::value<std::string>())
            ("w,z_rejection_flag", "enable a second rejection cycle", cxxopts::value<int>(z_rejection_flag)->default_value("0"))

            ("o,noiseoff", "Param foo", cxxopts::value<double>(noise_offset)->default_value("0.0"))
            ("n,noisestdv", "Param foo", cxxopts::value<double>(noise_var)->default_value("0.0"))
            ("x,xoffset", "Param foo", cxxopts::value<double>(x_o)->default_value("0.0"))
            ("y,yoffset", "Param foo", cxxopts::value<double>(y_o)->default_value("0.0"))
            ("z,zoffset", "Param foo", cxxopts::value<double>(z_o)->default_value("0.0"))
            ("d,visdistance", "visualization distance", cxxopts::value<double>(visu_dist)->default_value("0.0"))
            ("a,alpha", "roation around z axis (insert in radiant)", cxxopts::value<double>(rot_alpha)->default_value("0.0"))
            ("b,betha", "roation around y axis (insert in radiant)", cxxopts::value<double>(rot_betha)->default_value("0.0"))
            ("c,gamma", "roation around x axis (insert in radiant)", cxxopts::value<double>(rot_gamma)->default_value("0.0"))

            ("m,method", "method to search keypoints (ISS, Harris, otherwise SWIFT)", cxxopts::value<std::string>())
            ("min_scale_SIFT", "input parameter min_scale_SIFT for SIFT", cxxopts::value<double>(min_scale_SIFT)->default_value("0.2"))
            ("nr_octaves_SIFT", "input parameter nr_octaves_SIFT for SIFT", cxxopts::value<int>(nr_octaves_SIFT)->default_value("4"))
            ("nr_scales_per_octave_SIFT", "input parameter nr_scales_per_octave_SIFT for SIFT", cxxopts::value<int>(nr_scales_per_octave_SIFT)->default_value("5"))
            ("min_contrast_SIFT", "input parameter min_contrast_SIFT for SIFT", cxxopts::value<double>(min_contrast_SIFT)->default_value("0.25"))

            ("set_radius_harris", "input parameter set_radius_harris for 3DHarris", cxxopts::value<float>(set_radius_harris)->default_value("1.7"))
            ("set_radius_search_harris", "input parameter set_radius_search_harris for 3DHarris", cxxopts::value<float>(set_radius_search_harris)->default_value("1.7"))
            ("HarrisRosponseMethod", "Response method for Harris, choose betweeen HARRIS,TOMASI,NOBLE,CURVATURE and LOWE (default)", cxxopts::value<std::string>())

            ("SalientRad_muliplier_ISS", "input parameter SalientRad_muliplier_ISS for ISS", cxxopts::value<int>(SalientRad_muliplier_ISS)->default_value("6"))
            ("NonMaxMultiplier_ISS", "input parameter NonMaxMultiplier_ISS for ISS", cxxopts::value<int>(NonMaxMultiplier_ISS)->default_value("4"))
            ("Threshold21_ISS", "input parameter min_scale_SIFT for ISS", cxxopts::value<double>(Threshold21_ISS)->default_value("0.99"))
            ("Threshold32_ISS", "input parameter Threshold32_ISS for ISS", cxxopts::value<double>(Threshold32_ISS)->default_value("0.99"))
            ("setMinNeighbors_ISS", "input parameter setMinNeighbors_ISS for ISS", cxxopts::value<int>(setMinNeighbors_ISS)->default_value("5"))
            ("setNumberOfThreads_ISS", "input parameter setNumberOfThreads_ISS for ISS", cxxopts::value<int>(setNumberOfThreads_ISS)->default_value("1"))

            ("r,red", "set one color red", cxxopts::value<int>(red)->default_value("0"))
            ("g,green", "set one color green", cxxopts::value<int>(green)->default_value("0"))
            ("u,blue", "set one color blue", cxxopts::value<int>(blue)->default_value("0"))
            ("j,jet", "apply jet function to color", cxxopts::value<int>(jet_flag)->default_value("0"))
            ("i,staked_height", "the height for the stacked color", cxxopts::value<double>(jet_stacking_threshold)->default_value("30.0"))
            ("h,help", "Print usage")
            ;
    auto result = options.parse(argc, argv);

    std::cout << "validation_radius_in: " << validation_radius_in << std::endl;
    std::cout << "validation_radius_out: " << validation_radius_out << std::endl;
    if (result.count("n") || result.count("o") ){
        std::cout << "noise offest: " << noise_offset << std::endl;
        std::cout << "noise stdv: " << noise_var << std::endl;
    }

    if (result.count("x") || result.count("y") || result.count("z")){
        std::cout << "x_o: " << x_o << std::endl;
        std::cout << "y_o: " << y_o << std::endl;
        std::cout << "z_o: " << z_o << std::endl;
    }

    if (result.count("a") || result.count("b") || result.count("c")){
        std::cout << "rot_alpha: " << rot_alpha << std::endl;
        std::cout << "rot_betha: " << rot_betha << std::endl;
        std::cout << "rot_gamma: " << rot_gamma << std::endl;
        roation_flag=1;
    }
    std::cout << "grid_flag: " << grid_flag << std::endl;
    std::string keypoint_method;
    if (result.count("method"))    {
        std::cout << "keypoint method = " << result["method"].as<std::string>() << std::endl;
        keypoint_method=result["method"].as<std::string>();
    }else{
        keypoint_method="SWIFT";
    }
    std::string HarrisRosponseMethod;
    if (result.count("HarrisRosponseMethod"))    {

        std::cout << "HarrisRosponseMethod = " << result["HarrisRosponseMethod"].as<std::string>() << std::endl;
        HarrisRosponseMethod=result["HarrisRosponseMethod"].as<std::string>();
    }else{
        HarrisRosponseMethod="LOWE";
    }
    if (result.count("i") ){
        std::cout << "Stacked height threshold: " << jet_stacking_threshold << std::endl;
    }
    std::string measure_file_str;
    if (result.count("output_filename"))    {
        std::cout << "Output filename for measurement = " << result["output_filename"].as<std::string>() << std::endl;
        measure_file_str=result["output_filename"].as<std::string>();
    }else{
        measure_file_str = "measurement.csv";
    }
    string src_file;
    if (result.count("input_filename_1"))    {
        std::cout << "filename for input 1 = " << result["input_filename_1"].as<std::string>() << std::endl;
        src_file=result["input_filename_1"].as<string>();
    }else {
        src_file = "../../data/cloud_nocoor.pcd";
    }
    string tgt_file;
    if (result.count("input_filename_2"))    {
        std::cout << "filename for input 2 = " << result["input_filename_2"].as<std::string>() << std::endl;
        tgt_file=result["input_filename_2"].as<string>();
    }else {
        tgt_file = "../../data/cloud_overlap__big_2.pcd";
    }

    if(keypoint_method=="SWIFT"){
        std::cout << "SIFT----Parameter--- " << std::endl;
        std::cout << "min_scale_SIFT: " << min_scale_SIFT << std::endl;
        std::cout << "nr_octaves_SIFT: " << nr_octaves_SIFT << std::endl;
        std::cout << "nr_scales_per_octave_SIFT: " << nr_scales_per_octave_SIFT << std::endl;
        std::cout << "min_contrast_SIFT: " << min_contrast_SIFT << std::endl;
        std::cout << "----End of Parameter--- " << std::endl;
    }
    if(keypoint_method=="Harris"){
        std::cout << "Harris----Parameter--- " << std::endl;
        std::cout << "set_radius_harris: " << set_radius_harris << std::endl;
        std::cout << "set_radius_search_harris: " << set_radius_search_harris << std::endl;
        std::cout << "----End of Parameter--- " << std::endl;
    }
    if(keypoint_method=="ISS"){
        std::cout << "ISS----Parameter--- " << std::endl;
        std::cout << "SalientRad_muliplier_ISS: " << SalientRad_muliplier_ISS << std::endl;
        std::cout << "NonMaxMultiplier_ISS: " << NonMaxMultiplier_ISS << std::endl;
        std::cout << "Threshold21_ISS: " << Threshold21_ISS << std::endl;
        std::cout << "Threshold32_ISS: " << Threshold32_ISS << std::endl;
        std::cout << "setMinNeighbors_ISS: " << setMinNeighbors_ISS << std::endl;
        std::cout << "setNumberOfThreads_ISS: " << setNumberOfThreads_ISS << std::endl;
        std::cout << "----End of Parameter--- " << std::endl;
    }

    // Time start (main function)
    time_t start_computation, end_computation, start_total, end_total;
    time(&start_total);
    time(&start_computation);

    // READ SOURCE AND TARGET FILES
    //string src_file = "Plate_no_change_500000_scaled.pcd";
    //string tgt_file = "Plate_change_500000.pcd";
    //string src_file = "../data1.pcd";
    //string tgt_file = "../data2.pcd";


    //string src_file = "../submap_part10_4.pcd";

    //string src_file = "../../data/cloud_nocoor.pcd";        //<--
    //string src_file = "../cloud_overlap_1.pcd";
    //string tgt_file = "../cloud_overlap_2.pcd";
    //string src_file = "../cloud_overlap_sbig_1.pcd";
    //string tgt_file = "../../data/cloud_overlap__big_2.pcd";//<--

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_original(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_original(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    //if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(src_file, *src_original) == -1 || pcl::io::loadPCDFile<pcl::PointXYZRGB>(tgt_file, *tgt_original) == -1)
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(src_file, *src_original) == -1 || pcl::io::loadPCDFile<pcl::PointXYZRGB>(tgt_file, *tgt_original) == -1  || pcl::io::loadPCDFile<pcl::PointXYZRGB>(tgt_file, *tgt_transformed) == -1)
    {
        PCL_ERROR("Couldn't read src or tgt file");
        return -1;
    }
    //make a copy of the original target data
    //pcl::copyPointCloud(*tgt_original, *tgt_transformed);
    cout << "Src points: " << src_original->points.size() << endl;
    cout << "Tgt points: " << tgt_transformed->points.size() << endl;

    add_noise_normal_distributedvoid(tgt_transformed, noise_offset, noise_var, x_o, y_o, z_o, red, green, blue);

    if(jet_flag==1  || jet_flag==2){
        double total_min_z, total_max_z;
        pcl::PointXYZRGB minPt1, maxPt1,minPt2, maxPt2;;
        pcl::getMinMax3D (*tgt_transformed, minPt1, maxPt1);
        pcl::getMinMax3D (*src_original, minPt2, maxPt2);

        total_min_z = minPt1.z<minPt2.z ? minPt1.z : minPt2.z;
        total_max_z = maxPt1.z>maxPt2.z ? maxPt1.z : maxPt2.z;

        std::cout << "Total min z: " << total_min_z << std::endl;
        std::cout << "Total max z: " << total_max_z << std::endl;

        // Fill the cloud with some points
        if(jet_flag==2){
            //regular stacked jet section (was chosen this style to not loop over the if)
            //double jet_stacking_threshold=30;
            for (std::size_t i = 0; i < src_original->points.size (); ++i){
                pcl::PointXYZRGB pointrgb;
                std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
                colors_rgb = stacked_jet( src_original->points[i].z, jet_stacking_threshold);

                std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                                     static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                                     static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
                pointrgb.rgb = *reinterpret_cast<float*>(&rgb);
                src_original->points[i].r = pointrgb.r;
                src_original->points[i].g = pointrgb.g;
                src_original->points[i].b = pointrgb.b;
            }
            // Fill the cloud with some points
            for (std::size_t i = 0; i < tgt_transformed->points.size (); ++i){
                pcl::PointXYZRGB pointrgb;
                std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
                colors_rgb = stacked_jet( tgt_transformed->points[i].z, jet_stacking_threshold);
                //colors_rgb = jet(( tgt_original->points[i].z -  total_min_z)/(total_max_z - total_min_z));
                std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                                     static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                                     static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
                pointrgb.rgb = *reinterpret_cast<float*>(&rgb);
                tgt_transformed->points[i].r = pointrgb.r;
                tgt_transformed->points[i].g = pointrgb.g;
                tgt_transformed->points[i].b = pointrgb.b;
            }
        }else{
            //regular section
            for (std::size_t i = 0; i < src_original->points.size (); ++i){
                pcl::PointXYZRGB pointrgb;
                std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
                colors_rgb = jet(( src_original->points[i].z -  total_min_z)/(total_max_z - total_min_z));
                std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                                     static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                                     static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
                pointrgb.rgb = *reinterpret_cast<float*>(&rgb);
                src_original->points[i].r = pointrgb.r;
                src_original->points[i].g = pointrgb.g;
                src_original->points[i].b = pointrgb.b;
            }
            // Fill the cloud with some points
            for (std::size_t i = 0; i < tgt_transformed->points.size (); ++i){
                pcl::PointXYZRGB pointrgb;
                std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
                colors_rgb = jet(( tgt_transformed->points[i].z -  total_min_z)/(total_max_z - total_min_z));
                std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                                     static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                                     static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
                pointrgb.rgb = *reinterpret_cast<float*>(&rgb);
                tgt_transformed->points[i].r = pointrgb.r;
                tgt_transformed->points[i].g = pointrgb.g;
                tgt_transformed->points[i].b = pointrgb.b;
            }
        }
    }



    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_decimated(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(src_original);
    sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    sor.filter(*src_decimated);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_decimated(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.setInputCloud(tgt_transformed);
    sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    sor.filter(*tgt_decimated);

    cerr << "Src PointCloud after decimation: " << src_decimated->width * src_decimated->height
        << " data points (" << pcl::getFieldsList(*src_decimated) << ")." << endl;
    cerr << "Tgt PointCloud after decimation: " << tgt_decimated->width * tgt_decimated->height
        << " data points (" << pcl::getFieldsList(*tgt_decimated) << ")." << endl;

    // Filtered point cloud copy
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
    src = src_decimated;
    tgt = tgt_decimated;


    //get the keypoints
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_tgt_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
    compute_keypoints(src, tgt, keypoints_src_visualize_temp, keypoints_tgt_visualize_temp, keypoint_method,
            min_scale_SIFT, nr_octaves_SIFT, nr_scales_per_octave_SIFT, min_contrast_SIFT,
            set_radius_harris, set_radius_search_harris,HarrisRosponseMethod,
            SalientRad_muliplier_ISS, NonMaxMultiplier_ISS, Threshold21_ISS, Threshold32_ISS, setMinNeighbors_ISS, setNumberOfThreads_ISS);

    // Time end (main computation)
    time(&end_computation);
    double time_elapsed_computation = difftime(end_computation, start_computation);
    cout << "Elasped computation time in seconds: " << time_elapsed_computation << endl;

    //get the most outer points of the cloud and calc the overlapping area
    pcl::PointXYZRGB minPt1, maxPt1, minPt2, maxPt2;;
    pcl::getMinMax3D (*src, minPt1, maxPt1);
    pcl::getMinMax3D (*tgt, minPt2, maxPt2);
    Mapsample mapsample_1, mapsample_2, submap_overlap;
    mapsample_1.set_values(minPt1.x, minPt1.y, minPt1.z,maxPt1.x, maxPt1.y, maxPt1.z);
    mapsample_2.set_values(minPt2.x, minPt2.y, minPt2.z,maxPt2.x, maxPt2.y, maxPt2.z);
    get_overlap(mapsample_1, mapsample_2, submap_overlap);

    //output the overlapping region
    if(0){
        std::cout << "Overlap details: " << "Submap 1 details: "<< "Submap 1 details: " << std::endl;
        std::cout << "Max x: " << submap_overlap.maxX << "   Max x: " << maxPt1.x << "   Max x: " << maxPt2.x << std::endl;
        std::cout << "Max y: " << submap_overlap.maxY << "   Max y: " << maxPt1.y << "   Max x: " << maxPt2.y << std::endl;
        std::cout << "Max z: " << submap_overlap.maxZ << "   Max z: " << maxPt1.z << "   Max x: " << maxPt2.z <<std::endl;
        std::cout << "Min x: " << submap_overlap.minX << "  Min x: " << minPt1.x << "  Min x: " << minPt2.x <<std::endl;
        std::cout << "Min y: " << submap_overlap.minY << "  Min y: " << minPt1.y << "  Min y: " << minPt2.y <<std::endl;
        std::cout << "Min z: " << submap_overlap.minZ << "  Min z: " << minPt1.z << "  Min z: " << minPt2.z <<std::endl;
    }
    if(grid_flag!=1){
        keypoint_evaluation(submap_overlap, keypoints_src_visualize_temp, keypoints_tgt_visualize_temp); //potherwise done later
    }


    // Visualization of keypoints along with the original cloud
    pcl::visualization::PCLVisualizer corresp_viewer("Correspondences Viewer");
    if(grid_flag!=1){
        corresp_viewer.setBackgroundColor(0, 0, 0);
        corresp_viewer.addPointCloud(src, "Src cloud");
        corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Src cloud");
        corresp_viewer.addPointCloud<pcl::PointXYZ>(keypoints_src_visualize_temp, "keypoints_src_corresp_viewer");
        corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints_src_corresp_viewer");
    }

    //generate the evaluation cloud, which is transformed back to the original position to evalute the distance of the original point (only rotation and transformation, z-noise stays inside)
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_transformed_eval(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*keypoints_tgt_visualize_temp, *tgt_transformed_eval);
    std::cout << "Derotation evaluation is on"  << std::endl;
    /*
    if(roation_flag==1){
        pcl::transformPointCloud (*tgt_transformed_eval, *tgt_transformed_eval, transform_1_inverse);
    }
     */
    for (std::size_t i = 0; i < tgt_transformed_eval->points.size (); ++i) {
        tgt_transformed_eval->points[i].x -=x_o;
        tgt_transformed_eval->points[i].y -=y_o;
        tgt_transformed_eval->points[i].z -=z_o;
    }


    //double evaluation_distances [10]; //distances between bins e.g. 0-<=0.1;0.1-<=0.2;.....
    double evaluation_distances[9] = {0.01, 0.025, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 1};
    int distance_bin_results[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


    int counter_correctFmatch=0, counter_wrongFmatch=0, counter_validationR=0;
    double x_diff=0, y_diff=0;
    double r =0, g=0, b=0;
    pcl::PointXYZ tgt_transformed_tmp;  //descripes the target point in with the applied transformation in the new coordinate system


    //add the virtual distance to the between the two clouds to make them better visual distinguishable
    for (std::size_t i = 0; i < tgt->points.size (); ++i) {
        tgt->points[i].z +=visu_dist;
    }
    for (std::size_t i = 0; i < keypoints_tgt_visualize_temp->points.size (); ++i) {
        keypoints_tgt_visualize_temp->points[i].z +=visu_dist;
    }
    std::cout << "Take care!!! for visualizaion reasons the whole target cloud got shifted around: " <<  visu_dist << " m"   << std::endl;
    if(grid_flag!=1) {
        corresp_viewer.addPointCloud(tgt, "Tgt cloud");
        corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Tgt cloud");
        corresp_viewer.addPointCloud<pcl::PointXYZ>(keypoints_tgt_visualize_temp, "keypoints_tgt_corresp_viewer");
        corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"keypoints_tgt_corresp_viewer");
    }


    //write kez information to the measurement file in case e.g. for grid search
    if(grid_flag==1){
        std::cout << "write to file " << std::endl;
        std::ofstream out;

        //do the keyoint evaluation here instead
        double overlap_rate=0, total_overlap_rate=0;
        int keypoint_Overlap_cnt=0, keypoint_Overlaparea_fit_cnt=0, keypoints_1_inOverlap=0, keypoints_2_inOverlap=0;

        //calculate the overlapping points
        //visualize the keypoints off bodyFiltered_1 with lines
        for (size_t i = 0; i < keypoints_tgt_visualize_temp->size(); ++i) {
            pcl::PointXYZ & tgt_idx = keypoints_tgt_visualize_temp->points[i];
            double distance_shortest=20;    //initalize big ang update the closest point if smaller

            if( (keypoints_tgt_visualize_temp->points[i].x > submap_overlap.minX) && (keypoints_tgt_visualize_temp->points[i].x < submap_overlap.maxX) && (keypoints_tgt_visualize_temp->points[i].y > submap_overlap.minY) && (keypoints_tgt_visualize_temp->points[i].x < submap_overlap.maxY)){
                for (size_t ii = 0; ii < keypoints_src_visualize_temp->size(); ++ii) {
                    pcl::PointXYZ &src_idx = keypoints_src_visualize_temp->points[ii];
                    //check if a coresponding keypoint exists
                    //only check x and y since noise and offset is added to the second cloud/submap
                    if ((keypoints_tgt_visualize_temp->points[i].x == keypoints_src_visualize_temp->points[ii].x) &&
                        (keypoints_tgt_visualize_temp->points[i].y == keypoints_src_visualize_temp->points[ii].y)) {
                        keypoint_Overlap_cnt++;
                    }

                    //is in area and overlapping
                    if ((keypoints_src_visualize_temp->points[i].x > submap_overlap.minX) &&
                        (keypoints_src_visualize_temp->points[i].x < submap_overlap.maxX) &&
                        (keypoints_src_visualize_temp->points[i].y > submap_overlap.minY) &&
                        (keypoints_src_visualize_temp->points[i].x < submap_overlap.maxY)) {

                        //calc the distnance to the other points and retrieve the closest point for evaluation of stability
                        x_diff = tgt_idx.x - (src_idx.x);
                        y_diff = tgt_idx.y - (src_idx.y);
                        double distance_tmp;
                        distance_tmp = std::sqrt(x_diff * x_diff + y_diff * y_diff);
                        if (distance_tmp < distance_shortest) {
                            distance_shortest = distance_tmp;
                        }

                        //is overlapping
                        if ((keypoints_tgt_visualize_temp->points[i].x == keypoints_src_visualize_temp->points[ii].x) &&
                            (keypoints_tgt_visualize_temp->points[i].y == keypoints_src_visualize_temp->points[ii].y)) {
                            keypoint_Overlaparea_fit_cnt++; //the points in the overalapping area which have the same coordinates
                        }
                    }
                }
                for (int counter = 0; counter < 10; counter++) {
                    if (distance_shortest < evaluation_distances[counter]) {
                        distance_bin_results[counter] += 1;
                        break;
                    } else if (counter == 9) {
                        distance_bin_results[9] += 1;
                    }
                }
            }
        }

        //output the bin results
        std::cout << "----------------------------------------------------------------"  << std::endl;
        std::cout << "bin results of the distance: " << std::endl;
        std::cout << "bin 0 [0<=" << evaluation_distances [0] << "] :" << distance_bin_results[0]<< std::endl;
        for(int counter=1;counter<9;counter++) {
            std::cout << "bin " << counter << " [" << evaluation_distances [counter-1] << "<=" << evaluation_distances [counter] << "] :" << distance_bin_results[counter]<< std::endl;
        }
        std::cout << "bin 9 [" << evaluation_distances [8] << "<=...] :" << distance_bin_results[9]<< std::endl;


        for (size_t counter = 0; counter < keypoints_src_visualize_temp->size(); ++counter) {
            if( (keypoints_src_visualize_temp->points[counter].x > submap_overlap.minX) && (keypoints_src_visualize_temp->points[counter].x < submap_overlap.maxX) && (keypoints_src_visualize_temp->points[counter].y > submap_overlap.minY) && (keypoints_src_visualize_temp->points[counter].x < submap_overlap.maxY)){
                keypoints_1_inOverlap++;
            }
        }
        for (size_t counter = 0; counter < keypoints_tgt_visualize_temp->size(); ++counter) {
            if( (keypoints_tgt_visualize_temp->points[counter].x > submap_overlap.minX) && (keypoints_tgt_visualize_temp->points[counter].x < submap_overlap.maxX) && (keypoints_tgt_visualize_temp->points[counter].y > submap_overlap.minY) && (keypoints_tgt_visualize_temp->points[counter].x < submap_overlap.maxY)){
                keypoints_2_inOverlap++;
            }
        }
        std::cout << "----------------------------------------------------------------"  << std::endl;
        std::cout << "Num of Keypoints_1 in the overlapping area: " << keypoints_1_inOverlap << std::endl;
        std::cout << "Num of Keypoints_2 in the overlapping area: " << keypoints_2_inOverlap << std::endl;
        std::cout << "Total Keypoints 1: " <<  keypoints_src_visualize_temp->size() << std::endl;
        std::cout << "Total Keypoints 2: " <<  keypoints_tgt_visualize_temp->size() << std::endl;

        if(keypoints_1_inOverlap<keypoints_2_inOverlap){
            overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1_inOverlap;
        }else{
            overlap_rate=(double) keypoint_Overlap_cnt/ (double) keypoints_2_inOverlap;
        }
        if(keypoints_src_visualize_temp->size()<keypoints_src_visualize_temp->size()){
            total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_src_visualize_temp->size();
        }else{
            total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_tgt_visualize_temp->size();
        }

        std::cout << "total Num of Overlapping Keypoints (coordinates): " << keypoint_Overlap_cnt << std::endl;
        std::cout << "Num Keypoints_1 in the region of interest fitting Keypoints_2 (coord): " << keypoint_Overlaparea_fit_cnt << std::endl;
        double overlap_rate_1, overlap_rate_2, total_overlap_rate_1, total_overlap_rate_2;
        overlap_rate_1= (double) keypoint_Overlaparea_fit_cnt / (double) keypoints_1_inOverlap;
        std::cout << "Overlapping Rate of Keypoints 1: " << overlap_rate_1 << std::endl;
        overlap_rate_2= (double) keypoint_Overlaparea_fit_cnt / (double) keypoints_2_inOverlap;
        std::cout << "Overlapping Rate of Keypoints 2: " << overlap_rate_2 << std::endl;
        total_overlap_rate_1= (double) keypoint_Overlap_cnt / (double) keypoints_src_visualize_temp->size();
        std::cout << "Total Overlapping Rate of Keypoints 1: " << total_overlap_rate_1 << std::endl;
        total_overlap_rate_2= (double) keypoint_Overlap_cnt / (double) keypoints_tgt_visualize_temp->size();
        std::cout << "Total Overlapping Rate of Keypoints 2: " << total_overlap_rate_2 << std::endl;
        std::cout << "----------------------------------------------------------------"  << std::endl;

        //build cum over previous
        int sum_distance_bins[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        for(int counter_out=0;counter_out<10;counter_out++){
            for(int counter_in=0;counter_in<10;counter_in++){
                if (counter_in<=counter_out){
                    sum_distance_bins[counter_out] =sum_distance_bins[counter_out]+distance_bin_results[counter_in];
                }
            }
        }
        /*
        //output the sum-bin results for test
        std::cout << "----------------------------------------------------------------"  << std::endl;
        std::cout << "sbin results of the distance: " << std::endl;
        std::cout << "sbin 0 [0<=" << evaluation_distances [0] << "] :" << sum_distance_bins[0]<< std::endl;
        for(int counter=1;counter<9;counter++) {
            std::cout << "sbin " << counter << " [0<=" << evaluation_distances [counter] << "] :" << sum_distance_bins[counter]<< std::endl;
        }
        std::cout << "sbin 9 [" << evaluation_distances [8] << "<=...] :" << sum_distance_bins[9]<< std::endl;
        */


        if (exists_file (measure_file_str) ){
            out.open(measure_file_str, std::ios::app);
        }else{
            out.open(measure_file_str, std::ios::app);
            //add the header in the csv file
            out << "src_file," << "tgt_file," << "Src_points," << "Tgt-points,"<< "noise_offest,"<<"noise_stdv,"<< "x_o," << "y_o," << "z_o,"<<"keypoint_method,"<<"Red,"<<"Green,"<<"Blue,"<<"Jetstatus,"<<"Stacked_height_threshold,";
            if(keypoint_method=="SWIFT") {
                out << "min_scale_SIFT," << "nr_octaves_SIFT," << "nr_scales_per_octave_SIFT," << "min_contrast_SIFT,---";
            }else if(keypoint_method=="Harris"){
                out << "set_radius_harris," << "set_radius_search_harris," << "HarrisResponseMethod,___,";
            }else if(keypoint_method=="ISS"){
                out << "SalientRad_muliplier_ISS,NonMaxMultiplier_ISS,Threshold21_ISS,Threshold32_ISS,setMinNeighbors_ISS,setNumberOfThreads_ISS,---";
            }else{
                out << "min_scale_SIFT," << "nr_octaves_SIFT," << "nr_scales_per_octave_SIFT," << "min_contrast_SIFT,";
                out << "set_radius_harris," << "set_radius_search_harris," << "HarrisResponseMethod,";
                out << "SalientRad_muliplier_ISS,NonMaxMultiplier_ISS,Threshold21_ISS,Threshold32_ISS,setMinNeighbors_ISS,setNumberOfThreads_ISS,---";
            }
            out << ",comp_time[s]," ;
            out <<"keypoints_1_inOverlap,"<<"keypoints_2_inOverlap,"<<"Total_Keypoints_1,"<<"Total_Keypoints_2,"<<"keypoint_Overlap_cnt,"<<"keypoint_Overlaparea_fit_cnt,"<<"overlap_rate_1,"<<"overlap_rate_2,"<<"total_overlap_rate_1,"<<"total_overlap_rate_2"<<",";
            /*
            out << "bin0[0<=" << evaluation_distances [0] << "]:" << distance_bin_results[0]<< ",";
            for(int counter=1;counter<9;counter++) {
                out << "bin" << counter << "[" << evaluation_distances [counter-1] << "<=" << evaluation_distances [counter] << "]"<< ",";
            }
            out << "bin9[" << evaluation_distances [8] << "<=...]"<< std::endl;
             */
            out << "bin0" << "," << "bin1" << "," << "bin2" << ","<< "bin3" << ","<< "bin4" << ","<< "bin5" << ","<< "bin6" << ","<< "bin7" << ","<< "bin8" << ","<< "bin9" <<",";
            out << "sbin0" << "," << "sbin1" << "," << "sbin2" << ","<< "sbin3" << ","<< "sbin4" << ","<< "sbin5" << ","<< "sbin6" << ","<< "sbin7" << ","<< "sbin8" << ","<< "sbin9" << std::endl;
        }

        //add measurment conditions
        out <<  src_file<< "," << tgt_file<<  ","<<src_original->points.size()<< "," << tgt_transformed->points.size() <<"," <<noise_offset<<"," <<noise_var<<"," <<x_o<<"," <<y_o<<"," <<z_o<<"," <<keypoint_method<<"," <<red<<"," <<green<<"," <<blue<<"," <<jet_flag<<"," <<jet_stacking_threshold<<",";
        if(keypoint_method=="SWIFT") {
            //out << "min_scale_SIFT," << "nr_octaves_SIFT," << "nr_scales_per_octave_SIFT," << "min_contrast_SIFT,---";
            out <<min_scale_SIFT<<"," <<nr_octaves_SIFT<<"," <<nr_scales_per_octave_SIFT<<"," <<min_contrast_SIFT<<",";
        }else if(keypoint_method=="Harris"){
            //out << "set_radius_harris," << "set_radius_search_harris," << "HarrisResponseMethod,___,";
            out<<set_radius_harris<<"," <<set_radius_search_harris<<","<<HarrisRosponseMethod<<",";
        }else if(keypoint_method=="ISS"){
            out << SalientRad_muliplier_ISS<<"," <<NonMaxMultiplier_ISS<<"," <<Threshold21_ISS<<"," <<Threshold32_ISS<<"," <<setMinNeighbors_ISS<<"," <<setNumberOfThreads_ISS<<",";
        }else{
            out <<min_scale_SIFT<<"," <<nr_octaves_SIFT<<"," <<nr_scales_per_octave_SIFT<<"," <<min_contrast_SIFT<<",";
            out<<set_radius_harris<<"," <<set_radius_search_harris<<","<<HarrisRosponseMethod<<",";
            out << SalientRad_muliplier_ISS<<"," <<NonMaxMultiplier_ISS<<"," <<Threshold21_ISS<<"," <<Threshold32_ISS<<"," <<setMinNeighbors_ISS<<"," <<setNumberOfThreads_ISS<<",";
        }


        out <<"___,";
        out << time_elapsed_computation << ","<< keypoints_1_inOverlap<< "," << keypoints_2_inOverlap <<","<< keypoints_src_visualize_temp->size()<< "," << keypoints_tgt_visualize_temp->size() <<","<<keypoint_Overlap_cnt<<","<< keypoint_Overlaparea_fit_cnt<<","<< overlap_rate_1<<"," << overlap_rate_2<<","<< total_overlap_rate_1<<","<< total_overlap_rate_2;

        //add perormance results
        //out << distance_bin_results[0];
        for(int counter=0;counter<9;counter++) {out << ","<< distance_bin_results[counter];}
        out << ","<< distance_bin_results[9];
        for(int counter=0;counter<9;counter++) {out << ","<< sum_distance_bins[counter];}
        out << ","<< sum_distance_bins[9]<< std::endl;




    }else{
        while (!corresp_viewer.wasStopped()) {
            corresp_viewer.spinOnce();
        }
        corresp_viewer.close();

        // Time end (main function)
        time(&end_total);
        double time_elapsed_total = difftime(end_total, start_total);
        cout << "Elasped total main function time in seconds: " << time_elapsed_total << endl;
    }
}



