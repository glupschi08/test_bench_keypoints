#include <stdlib.h>
#include <iostream>
#include "overlap.h"

void Mapsample::set_values (float a, float b, float c, float d, float e, float f) {
    minX = a;
    minY = b;
    minZ = c;
    maxY = d;
    maxX = e;
    maxZ = f;
}


void get_overlap(Mapsample submap_1, Mapsample submap_2, Mapsample& submap_overlap){

    float minX_overlap;
    float minY_overlap;
    float minZ_overlap;
    float maxX_overlap;
    float maxY_overlap;
    float maxZ_overlap;


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

    /*
    std::cout << "Submap OVERLAP details: "<< std::endl;
    std::cout << "minX: " << minX_overlap << std::endl;
    std::cout << "minY: " << minY_overlap << std::endl;
    std::cout << "maxX: " << maxX_overlap << std::endl;
    std::cout << "maxY: " << maxY_overlap << std::endl;
    std::cout << "minZ: " << minZ_overlap << std::endl;
    std::cout << "maxZ: " << maxZ_overlap << std::endl;
*/

    //set the values
    submap_overlap.set_values(minX_overlap,minY_overlap,minZ_overlap,maxX_overlap,maxY_overlap,maxZ_overlap);
}
