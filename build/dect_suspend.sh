#!/bin/bash

./evaluation outputs/output3.pcd  ../objects/s0_piggybank_corr.pcd ../ground_truth/piggy_ground_truth.pcd >> cshot_piggy_ipc_rad.txt

pm-suspend
