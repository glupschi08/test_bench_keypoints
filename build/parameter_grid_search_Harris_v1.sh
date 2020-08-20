#!/bin/bash
#Simple array
HarrisMethods=(HARRIS TOMASI NOBLE LOWE CURVATURE)
noise_level=(0.1)


set_radius_search_harris_array=(0.05 0.1 0.3 0.7 1.0 1.3 1.6 1.7 1.8 2.0 2.5 5.0 10.0 15.0 100.0)
set_radius_harris_array=(0.05 0.1 0.3 0.7 1.0 1.3 1.6 1.7 1.8 2.0 2.5 5.0 10.0 15.0 100.0)

for noise_level_counter in "${noise_level[@]}"
do
  for set_radius_search_harris_array_counter in "${set_radius_search_harris_array[@]}"
  do
    for set_radius_harris_array_counter in "${set_radius_harris_array[@]}"
    do
      for HarrisMethods_counter in "${HarrisMethods[@]}"
      do
      #echo $repeat_counter
        ./testkeypoints --noiseoff=0.0 --noisestdv=0.0 --method=Harris --HarrisRosponseMethod=$HarrisMethods_counter --grid=1 -j 0 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_1.pcd --output_filename=../measurements/Harris_parmeter_search_map_1_noise00.csv --set_radius_harris=$set_radius_harris_array_counter --set_radius_search_harris=$set_radius_search_harris_array_counter
        ./testkeypoints --noiseoff=0.0 --noisestdv=0.0 --method=Harris --HarrisRosponseMethod=$HarrisMethods_counter --grid=1 -j 0 --input_filename_1=../../data/cloud_nocoor.pcd --input_filename_2=../../data/cloud_overlap__big_2.pcd --output_filename=../measurements/Harris_parmeter_search_map_2_noise00.csv --set_radius_harris=$set_radius_harris_array_counter --set_radius_search_harris=$set_radius_search_harris_array_counter
        ./testkeypoints --noiseoff=0.0 --noisestdv=0.1 --method=Harris --HarrisRosponseMethod=$HarrisMethods_counter --grid=1 -j 0 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_1.pcd --output_filename=../measurements/Harris_parmeter_search_map_1_noise01.csv --set_radius_harris=$set_radius_harris_array_counter --set_radius_search_harris=$set_radius_search_harris_array_counter
        ./testkeypoints --noiseoff=0.0 --noisestdv=0.1 --method=Harris --HarrisRosponseMethod=$HarrisMethods_counter --grid=1 -j 0 --input_filename_1=../../data/cloud_nocoor.pcd --input_filename_2=../../data/cloud_overlap__big_2.pcd --output_filename=../measurements/Harris_parmeter_search_map_2_noise01.csv --set_radius_harris=$set_radius_harris_array_counter --set_radius_search_harris=$set_radius_search_harris_array_counter
      done
    done
  done
done



