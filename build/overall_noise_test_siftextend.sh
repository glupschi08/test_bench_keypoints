#!/bin/bash
#Simple array

#ISS Parameter
noise_level=(0.0)
noise_offset=(0.0)

noise_level=(0.125 0.15 0.175 0.2)
noise_offset=(0.0 0.1 0.25 0.5 1)

SalientRad_muliplier_ISS=(6 8)
NonMaxMultiplier_ISS=(3 5)
Threshold21_ISS=(0.5 0.8)
Threshold32_ISS=(0.8 0.95)
setMinNeighbors_ISS=(4 6)
setNumberOfThreads_ISS=(1)
#repeat=(1 2)
repeat=(1 2 3)



#Harris Parameter
HarrisMethods=(HARRIS TOMASI NOBLE LOWE CURVATURE)
set_radius_search_harris_array=(0.7 0.8 1)
set_radius_harris_array=(1.7 1.8)

#SIFT Parameter
min_scale_SIFT=(0.25 0.99)
nr_octaves_SIFT=(6)
nr_scales_per_octave_SIFT=(8 9)
min_contrast_SIFT=(0.3 0.7)
jetheight=(30)


for noise_level_counter in "${noise_level[@]}"
do
  for noise_offset_counter in "${noise_offset[@]}"
  do
      for jet_counter in "${jetheight[@]}"
      do
        for min_scale_SIFT_counter in "${min_scale_SIFT[@]}"
        do
          for nr_octaves_SIFT_counter in "${nr_octaves_SIFT[@]}"
          do
            for nr_scales_per_octave_SIFT_counter in "${nr_scales_per_octave_SIFT[@]}"
            do
              for min_contrast_SIFT_counter in "${min_contrast_SIFT[@]}"
              do
                #echo $min_contrast_SIFT_counter
                ./testkeypoints --noiseoff=$noise_offset_counter --noisestdv=$noise_level_counter --method=SWIFT --grid=1 -j 2 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_1.pcd --output_filename=../measurements/noise_test/SIFT_map1_2.csv  --min_scale_SIFT=$min_scale_SIFT_counter --nr_octaves_SIFT=$nr_octaves_SIFT_counter --nr_scales_per_octave_SIFT=$nr_scales_per_octave_SIFT_counter --min_contrast_SIFT=$min_contrast_SIFT_counter
                ./testkeypoints --noiseoff=$noise_offset_counter --noisestdv=$noise_level_counter --method=SWIFT --grid=1 -j 2 --input_filename_1=../../data/cloud_nocoor.pcd --input_filename_2=../../data/cloud_overlap__big_2.pcd --output_filename=../measurements/noise_test/SIFT_map2_2.csv  --min_scale_SIFT=$min_scale_SIFT_counter --nr_octaves_SIFT=$nr_octaves_SIFT_counter --nr_scales_per_octave_SIFT=$nr_scales_per_octave_SIFT_counter --min_contrast_SIFT=$min_contrast_SIFT_counter
              done
            done
          done
        done
      done
  done
done


for noise_level_counter in "${noise_level[@]}"
do
  for noise_offset_counter in "${noise_offset[@]}"
  do


      #here the ISS testing
      for SalientRad_muliplier_ISS_counter in "${SalientRad_muliplier_ISS[@]}"
      do
        for NonMaxMultiplier_ISS_counter in "${NonMaxMultiplier_ISS[@]}"
        do
          for Threshold21_ISS_counter in "${Threshold21_ISS[@]}"
          do
            for Threshold32_ISS_counter in "${Threshold32_ISS[@]}"
            do
              for setMinNeighbors_ISS_counter in "${setMinNeighbors_ISS[@]}"
              do
                for setNumberOfThreads_ISS_counter in "${setNumberOfThreads_ISS[@]}"
                do
                  for repeat_counter in "${repeat[@]}"
                  do
                      #echo $repeat_counter
                      ./testkeypoints --noiseoff=$noise_offset_counter --noisestdv=$noise_level_counter --method=ISS --grid=1 -j 0 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_1.pcd --output_filename=../measurements/noise_test/ISS_map1_2.csv  --SalientRad_muliplier_ISS=$SalientRad_muliplier_ISS_counter --NonMaxMultiplier_ISS=$NonMaxMultiplier_ISS_counter --Threshold21_ISS=$Threshold21_ISS_counter --Threshold32_ISS=$Threshold32_ISS_counter --setMinNeighbors_ISS=$setMinNeighbors_ISS_counter --setNumberOfThreads_ISS=$setNumberOfThreads_ISS_counter
                      ./testkeypoints --noiseoff=$noise_offset_counter --noisestdv=$noise_level_counter --method=ISS --grid=1 -j 0 --input_filename_1=../../data/cloud_nocoor.pcd --input_filename_2=../../data/cloud_overlap__big_2.pcd --output_filename=../measurements/noise_test/ISS_map2_2.csv  --SalientRad_muliplier_ISS=$SalientRad_muliplier_ISS_counter --NonMaxMultiplier_ISS=$NonMaxMultiplier_ISS_counter --Threshold21_ISS=$Threshold21_ISS_counter --Threshold32_ISS=$Threshold32_ISS_counter --setMinNeighbors_ISS=$setMinNeighbors_ISS_counter --setNumberOfThreads_ISS=$setNumberOfThreads_ISS_counter
                  done
                done
              done
            done
          done
        done
      done
   done
done

for noise_level_counter in "${noise_level[@]}"
do
  for noise_offset_counter in "${noise_offset[@]}"
  do
      for set_radius_search_harris_array_counter in "${set_radius_search_harris_array[@]}"
      do
        for set_radius_harris_array_counter in "${set_radius_harris_array[@]}"
        do
          for HarrisMethods_counter in "${HarrisMethods[@]}"
          do
            #echo $HarrisMethods_counter
            ./testkeypoints --noiseoff=$noise_offset_counter --noisestdv=$noise_level_counter --method=Harris --HarrisRosponseMethod=$HarrisMethods_counter --grid=1 -j 0 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_1.pcd --output_filename=../measurements/noise_test/Harris_map1_2.csv --set_radius_harris=$set_radius_harris_array_counter --set_radius_search_harris=$set_radius_search_harris_array_counter
            ./testkeypoints --noiseoff=$noise_offset_counter --noisestdv=$noise_level_counter --method=Harris --HarrisRosponseMethod=$HarrisMethods_counter --grid=1 -j 0 --input_filename_1=../../data/cloud_nocoor.pcd --input_filename_2=../../data/cloud_overlap__big_2.pcd --output_filename=../measurements/noise_test/Harris_map2_2.csv --set_radius_harris=$set_radius_harris_array_counter --set_radius_search_harris=$set_radius_search_harris_array_counter
          done
        done
      done
  done
done




