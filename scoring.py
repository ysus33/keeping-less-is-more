import argparse
import csv
import os
from evo.tools import file_interface
from evo.core import sync
from evo.core.metrics import PoseRelation, Unit
import evo.main_ape as main_ape
import evo.common_ape_rpe as common

parser = argparse.ArgumentParser()
parser.add_argument('--gt-path', type=str)
parser.add_argument('--root-path', type=str)
parser.add_argument('--dataset', type=str)

args = parser.parse_args()

if __name__=="__main__":

    root = args.root_path
    gt_dir = args.gt_path

    f = open(os.path.join(root, 'result.csv'), 'r+', encoding='utf-8')
    rdr = csv.reader(f)

    lines = []

    if args.dataset=='kitti':
        for i, line in enumerate(rdr):
            if (i==0 or i==1): 
                lines.append(line)
                continue
            gt = file_interface.read_kitti_poses_file(gt_dir)

            name = line[0]
            est_dir = os.path.join(root, name+"CameraTrajectory.txt")
            print(est_dir)
            est = file_interface.read_kitti_poses_file(est_dir)
            
            result_ate_r = main_ape.ape(gt, est, pose_relation=PoseRelation.rotation_part, align=True)
            result_ate = main_ape.ape(gt, est, pose_relation=PoseRelation.translation_part, align=True)

            line+=['{:.5f}'.format(result_ate_r.stats['rmse']), '{:.5f}'.format(result_ate.stats['rmse'])]

            lines.append(line)

    if args.dataset=='euroc':
        for i, line in enumerate(rdr):
            if (i==0 or i==1): 
                lines.append(line)
                continue
            gt = file_interface.read_euroc_csv_trajectory(gt_dir)

            name = line[0]
            est_dir = os.path.join(root, name+"CameraTrajectory.txt")
            print(est_dir)
            est = file_interface.read_tum_trajectory_file(est_dir)
            
            traj_ref, traj_est = sync.associate_trajectories(gt, est, 0.01, 0.0)

            result_ate_r = main_ape.ape(traj_ref, traj_est, pose_relation=PoseRelation.rotation_part, align=True)
            result_ate = main_ape.ape(traj_ref, traj_est, pose_relation=PoseRelation.translation_part, align=True)

            line+=['{:.5f}'.format(result_ate_r.stats['rmse']), '{:.5f}'.format(result_ate.stats['rmse'])]

            lines.append(line)

    if args.dataset=='tum':
        for i, line in enumerate(rdr):
            if (i==0 or i==1): 
                lines.append(line)
                continue
            gt = file_interface.read_tum_trajectory_file(gt_dir)

            name = line[0]
            est_dir = os.path.join(root, name+"CameraTrajectory.txt")
            print(est_dir)
            est = file_interface.read_tum_trajectory_file(est_dir)
            
            traj_ref, traj_est = sync.associate_trajectories(gt, est, 0.01, 0.0)

            result_ate_r = main_ape.ape(traj_ref, traj_est, pose_relation=PoseRelation.rotation_part, align=True)
            result_ate = main_ape.ape(traj_ref, traj_est, pose_relation=PoseRelation.translation_part, align=True)

            line+=['{:.5f}'.format(result_ate_r.stats['rmse']), '{:.5f}'.format(result_ate.stats['rmse'])]

            lines.append(line)


    f.close()
    
    f = open(os.path.join(root, 'result_f.csv'), 'w')
    wr = csv.writer(f)
    wr.writerows(lines)
    
    f.close()