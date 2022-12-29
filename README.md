# Keeping Less is More: Point Sparsification for Visual SLAM
Authors: Yeonsoo Park, Soohyun Bae

This repository is an implementation of the paper [**Keeping Less is More: Point Sparsification for Visual SLAM**](https://arxiv.org/abs/2207.00225) based on ORB-SLAM2.

<img width="80%" src="https://github.com/ysus33/keeping-less-is-more/blob/main/figure/door.gif"/>

Our sparsification module is implemented on `MatchCuller.cc` and applied in local BA process of `Optimizer.cc`.
Several modifications are done from the original implementation for reliable test.
1. We changed the exiting multi-threaded process **to a single thread process** for
    * an objective evaluation of the total runtime
    * deterministic performance evaluation
    * disabling frame dropouts in optimization steps due to the processing delays
2. We adopted **deterministic keyframe insertion criterion** rather than using the original condition for deciding keyframes in ORB-SLAM2 since
    * original condition depends on the number of tracked points from the local mapping thread & the state of the local mapping thread
    * Therefore it makes the number of the local BA execution changes when the single threaded processing or point sparsification applied
    * We alternatively use a deterministic keyframe insertion criterion that depends on the amount of change in translation and rotation.
    * we chose threshold values for translation and rotation changes for each of data sequence that maintain the similar number of keyframes, mappoints, and number of local BA from the result of original version. A pair of threshold for sample data is provided on the `test_scan.sh` and `test_euroc.sh`.

See the [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) github page for details on the dataset and Prerequisites.

# Test Samples
Provides a script with several pairs of sequences and settings. [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) MH01 for stereo, [ScanNet](https://github.com/ScanNet/ScanNet)scene0000_00, scene0000_01 for RGB-D.


## SLAM
we used part of code from [**ORB-SLAM3**](https://github.com/UZ-SLAMLab/ORB_SLAM3) for statistics logging.
By running provided shell script `./Sample/RGB-D/test_scan.sh` or `./Sample/Stereo/test_euroc.sh`, you can get a result folder that contains stat files, `result.csv`, and estimated trajectory file of both original and sparsified version for comparison.
Change the Variable `$pathDataset` according to the directory of dataset.

## Localization
we used part of code from https://github.com/Alkaid-Benetnash/ORB_SLAM2 to load/save map for localization test.
To save map, uncomment the line `SLAM.SaveMap()` in `rgbd_scan.cc ` or `stereo_euroc.cc`.
To load map, add the line `Map.mapfile: /directory/map.bin` at the end of the .yaml file. If the map file is valid, system would run with an localization mode automatically.
We provide script `./Sample/RGB-D/test_scan_localization.sh` for localization test on ScanNet that performs localization on scene0000_01 against the two version of maps build from scene0000_00. Running this after `./Sample/RGB-D/test_scan.sh` will produce the compared localization results from original and sparsified map.

original map             |  sparsified map
:-------------------------:|:-------------------------:
![](https://github.com/ysus33/keeping-less-is-more/blob/main/figure/local_ori.gif)  |  ![](https://github.com/ysus33/keeping-less-is-more/blob/main/figure/local_cull.gif)

Video above is the visualization of localization process on scene0000 with pre-built `.pcd` file and `mappoints.txt` file.

## Evaluation
we use [**evo**](https://github.com/MichaelGrupp/evo) package to evaluate performance. Follow the link to get an instruction for installation. 
We provide a python script to efficiently log the score using evo package. 
By running command below, evaluation score will be recorded in the last two columns of `result.csv` in the `root_path`. For example,
* ScanNet
```bash
python scoring.py --dataset=kitti --gt-path='/directory/gt.txt' --root-path=./Sample/RGB-D/result
```
* EuRoC
```bash
python scoring.py --dataset=euroc --gt-path='/directory/data.csv' --root-path=./Sample/Stereo/result
```

or you can just run command below to check RMSE of ATE(m):
```bash
evo_ape kitti 'gt.txt' 'CameraTrajectory.txt' -r trans_part -a
```

# Citation
If you use sparsification module implemented in the code in an academic work, please cite:

    @article{park2022keeping,
      title={Keeping Less is More: Point Sparsification for Visual SLAM},
      author={Park, Yeonsoo and Bae, Soohyun},
      journal={arXiv preprint arXiv:2207.00225},
      year={2022}
    }

## Dependencies
Added to the original ORB-SLAM2's Dependencies.md to clarify any additional dependencies I've applied for my implementation here.
