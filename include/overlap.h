#pragma once

#ifndef OVERLAP_H
#define OVERLAP_H

class Mapsample{
public:
    double minX;
    double minY;
    double minZ;
    double maxX;
    double maxY;
    double maxZ;
    double overlap_size;
    void set_values(double,double,double,double,double,double);
    void calc_area(double area_tmp);
};

#endif

void get_overlap(Mapsample submap_1, Mapsample submap_2, Mapsample& submap_overlap);

