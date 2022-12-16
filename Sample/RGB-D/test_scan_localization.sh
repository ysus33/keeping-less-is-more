#!/bin/bash

pathLOG=./result_local
pathDataset=/mnt/data3/ScanNet/scene0000_01
pathGT=$pathDataset/gt.txt
PathRESULT=$pathLOG/result.csv

mkdir -p $pathLOG

echo "don't erase this line" >> $PathRESULT
echo "mode, total runtime, ate(m)" >> $PathRESULT

./rgbd_scan ../../Vocabulary/ORBvoc.txt ./setting/ScanNet0_locOri.yaml $pathDataset $PathRESULT $pathLOG 0 0 0
echo "" >> $PathRESULT
./rgbd_scan ../../Vocabulary/ORBvoc.txt ./setting/ScanNet0_locCull.yaml $pathDataset $PathRESULT $pathLOG 0 0 0

# scoring
python ../../scoring.py --dataset=kitti --gt-path=$pathGT --root-path=$pathLOG