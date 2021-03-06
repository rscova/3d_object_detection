#!/bin/bash

cmake ..

make -j7

#./planar_segmentation ../scenes/snap_0point.pcd outputs/output1.pcd 0.1
#./planar_segmentation outputs/output1.pcd outputs/output2.pcd 0.05
#./planar_segmentation outputs/output2.pcd outputs/output3.pcd 0.01

#./find_keypoints outputs/output3.pcd keypoints/scene_keypoints
#./find_keypoints ../objects/s0_piggybank_corr.pcd keypoints/object_keypoints
#./find_features outputs/output3.pcd ../objects/s0_piggybank_corr.pcd keypoints/scene_keypoints_iss.pcd keypoints/object_keypoints_iss.pcd

#./evaluation outputs/output3.pcd ../objects/s0_piggybank_corr.pcd >> tests/cvfh_evaluate_piggy-sin_ICP-con_radio.txt
#./evaluation outputs/output3.pcd ../objects/s0_mug_corr.pcd   >> tests/cvfh_evaluate_mug-sin_ICP-con_radio.txt
#./evaluation outputs/output3.pcd ../objects/s0_plc_corr.pcd   >> tests/cvfh_evaluate_plc-sin_ICP-con_radio.txt
#./evaluation outputs/output3.pcd ../objects/s0_plant_corr.pcd >> tests/cvfh_evaluate_plant-sin_ICP-con_radio.txt

#./evaluation outputs/output3.pcd  ../objects/s0_piggybank_corr.pcd ../ground_truth/piggy_ground_truth.pcd >> cshot_piggy.txt
./evaluation outputs/output3.pcd  ../objects/s0_plant_corr.pcd ../ground_truth/plant_ground_truth.pcd >> cshot_plant_icp_rad.txt &
./evaluation outputs/output3.pcd  ../objects/s0_mug_corr.pcd ../ground_truth/mug_ground_truth.pcd >> cshot_mug_icp_rad.txt &
./evaluation outputs/output3.pcd  ../objects/s0_plc_corr.pcd ../ground_truth/plc_ground_truth.pcd >> cshot_plc_icp_rad.txt

