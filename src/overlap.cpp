#include <stdlib.h>
#include <iostream>
#include "overlap.h"

void Mapsample::set_values (double a, double b, double c, double d, double e, double f) {
    minX = a;
    minY = b;
    minZ = c;
    maxY = d;
    maxX = e;
    maxZ = f;
}

void Mapsample::calc_area(double area_tmp){
    overlap_size = area_tmp;
}

void get_overlap(Mapsample submap_1, Mapsample submap_2, Mapsample& submap_overlap){

    double minX_overlap;
    double minY_overlap;
    double minZ_overlap;
    double maxX_overlap;
    double maxY_overlap;
    double maxZ_overlap;


    //calc overlapping area
    if(submap_1.minX>submap_2.minX){
        minX_overlap=submap_1.minX;
    }else{
        minX_overlap=submap_2.minX;
    }
    if(submap_1.minY>submap_2.minY){
        minY_overlap=submap_1.minY;
    }else{
        minY_overlap=submap_2.minY;
    }
    if(submap_1.maxX<submap_2.maxX){
        maxX_overlap=submap_1.maxX;
    }else{
        maxX_overlap=submap_2.maxX;
    }
    if(submap_1.maxY<submap_2.maxY){
        maxY_overlap=submap_1.maxY;
    }else{
        maxY_overlap=submap_2.maxY;
    }
    if(submap_1.maxZ>submap_2.maxZ){
        maxZ_overlap=submap_1.maxZ;
    }else{
        maxZ_overlap=submap_2.maxZ;
    }
    if(submap_1.minZ<submap_2.minZ){
        minZ_overlap=submap_1.minZ;
    }else{
        minZ_overlap=submap_2.minZ;
    }


    //set the values
    submap_overlap.set_values(minX_overlap,minY_overlap,minZ_overlap,maxX_overlap,maxY_overlap,maxZ_overlap);

    double x_diff, y_diff, area_overlap=0;
    x_diff=submap_overlap.maxX-submap_overlap.minX;
    y_diff=submap_overlap.maxY-submap_overlap.minY;
    if(x_diff>0 && y_diff>0 ){
        area_overlap=x_diff*y_diff;
        submap_overlap.calc_area(area_overlap);
    }
    //std::cout << "x_diff in: " << x_diff <<std::endl;
    //std::cout << "y_diff in: " << y_diff <<std::endl;
    //std::cout << "in area: " << area_overlap << "  submap_overlap.area_overlap in: " << submap_overlap.overlap_size <<std::endl;
}
