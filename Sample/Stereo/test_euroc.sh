#!/bin/bash

pathLOG=./result
pathDataset=/mnt/data3/EuRoC/MH01
Data=$(basename $pathDataset)
    
mkdir -p $pathLOG
PathRESULT=$pathLOG/result.csv

echo " , , Tracking, , , , , , , Mapping, , , , , LBA, , , , , , , , , , Map Size, , Loop, , , Loop info, , GBA, , , , , , etc, , acc" >> $PathRESULT
echo "mode, total runtime, Stereo rect., ORB extract, Stereo match, Pose pred, LM Track, New KF dec, Total, KF Insert, MP Creation, LBA, Total, #LBA, #Edges, #KF optimized, #KF fixed, #MP, Culling time, Cull ratio, pure LBA time, #connection(avg.), frame difference(avg.), spatial diversity(avg.), #KFs, #MPs, Loop Fusion, Opt.Essention Graph, Total, # Detected loops, Loop size, Full BA, Map Update, Total, #GBA, BA size (# keyframes), BA size (# map points), #LOST, #culled Points, ATE(rot.), ATE(trans.)" >> $PathRESULT

thR=0.06
thT=0.5
./stereo_euroc ../../Vocabulary/ORBvoc.txt ./setting/EuRoC.yaml $pathDataset/cam0/data $pathDataset/cam1/data ./EuRoC_TimeStamps/$Data.txt $PathRESULT $pathLOG $thR $thT 0
./stereo_euroc ../../Vocabulary/ORBvoc.txt ./setting/EuRoC.yaml $pathDataset/cam0/data $pathDataset/cam1/data ./EuRoC_TimeStamps/$Data.txt $PathRESULT $pathLOG $thR $thT 1 100
