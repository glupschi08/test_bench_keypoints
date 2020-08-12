#pragma once

#ifndef OVERLAP_H
#define OVERLAP_H

class Mapsample{
public:
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;
    void set_values(float,float,float,float,float,float);

};

#endif

void get_overlap(Mapsample submap_1, Mapsample submap_2, Mapsample& submap_overlap);

