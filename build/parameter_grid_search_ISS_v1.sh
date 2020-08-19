#!/bin/bash
#Simple array

SalientRad_muliplier_ISS=(6 7 8)
NonMaxMultiplier_ISS=(3 4 5)
Threshold21_ISS=(0.5 0.8 0.99 1.2)
Threshold32_ISS=(0.5 0.8 0.99 1.2)
setMinNeighbors_ISS=(4 5 6 8)
setNumberOfThreads_ISS=(1)
repeat=(1 2 3 4 5 6 7 8 9 10)

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
                ./testkeypoints --noiseoff=0.0 --noisestdv=0.1 --method=ISS --grid=1 -j 0 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_1.pcd --output_filename=../measurements/ISS_tunning_2_map1.csv  --SalientRad_muliplier_ISS=$SalientRad_muliplier_ISS_counter --NonMaxMultiplier_ISS=$NonMaxMultiplier_ISS_counter --Threshold21_ISS=$Threshold21_ISS_counter --Threshold32_ISS=$Threshold32_ISS_counter --setMinNeighbors_ISS=$setMinNeighbors_ISS_counter --setNumberOfThreads_ISS=$setNumberOfThreads_ISS_counter
                ./testkeypoints --noiseoff=0.0 --noisestdv=0.1 --method=ISS --grid=1 -j 0 --input_filename_1=../../data/cloud_nocoor.pcd --input_filename_2=../../data/cloud_overlap__big_2.pcd --output_filename=../measurements/ISS_tunning_2_map1.csv  --SalientRad_muliplier_ISS=$SalientRad_muliplier_ISS_counter --NonMaxMultiplier_ISS=$NonMaxMultiplier_ISS_counter --Threshold21_ISS=$Threshold21_ISS_counter --Threshold32_ISS=$Threshold32_ISS_counter --setMinNeighbors_ISS=$setMinNeighbors_ISS_counter --setNumberOfThreads_ISS=$setNumberOfThreads_ISS_counter
            done
          done
        done
      done
    done
  done
done




