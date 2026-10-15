# MIT License
#
# Copyright (c) 2026 Saurabh Gupta.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import contextlib
import datetime
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
from pyquaternion import Quaternion
from tqdm import trange

from retro_slam.config import load_config, write_config
from retro_slam.metrics import sequence_error
from retro_slam.slam import RetroSLAM
from retro_slam.tools.pipeline_results import PipelineResults
from retro_slam.visualizers.visualizer import Viz


class SlamPipeline:
    def __init__(
        self,
        dataset,
        config_file: Optional[Path] = None,
        visualize: bool = False,
        n_scans: int = -1,
    ):
        self._dataset = dataset
        self._n_scans = n_scans if n_scans > 0 else len(self._dataset)
        self.slam_config = load_config(config_file)
        self.results_dir = None

        self.results = PipelineResults()
        self.times = np.zeros(self._n_scans)
        self.poses = np.zeros((self._n_scans, 4, 4))
        self.has_gt = hasattr(self._dataset, "gt_poses")
        self.gt_poses = self._dataset.gt_poses[0 : self._n_scans] if self.has_gt else None

        self.dataset_name = self._dataset.__class__.__name__
        self.dataset_sequence = (
            self._dataset.sequence_id
            if hasattr(self._dataset, "sequence_id")
            else os.path.basename(self._dataset.data_dir)
        )

        self.retro_slam = RetroSLAM(self.slam_config)

        self.viz = None
        if visualize:
            self.viz = Viz()

    def run(self):
        self._create_output_dir()
        self._run_pipeline()
        self._run_evaluation()
        self._evaluate_closures()
        self._write_result_poses()
        self._write_gt_poses()
        self._write_cfg()
        self._write_log()
        self._write_graph()
        self._write_closures()
        return self.results

    def _run_pipeline(self):
        for scan_idx in trange(0, self._n_scans, unit=" frames", dynamic_ncols=True):
            scan, timestamps = self._dataset[scan_idx]
            start_time = time.perf_counter_ns()
            self.retro_slam.process_scan(scan, timestamps)
            self.times[scan_idx] = time.perf_counter_ns() - start_time
            if self.viz:
                self.viz.log_slam_state(self.retro_slam, scan=scan, time_ns=timestamps[-1])
        self.retro_slam.exit()
        self.poses = self.retro_slam.poses

    def _run_evaluation(self):
        if self.has_gt:
            ate_trans, rpe_trans = sequence_error(self.gt_poses, self.poses)
            self.results.append(desc="RPE (translation)", units="%", value=rpe_trans)
            self.results.append(desc="ATE (translation)", units="m", value=ate_trans)

        fps = self._get_fps()
        avg_fps = int(np.floor(fps))
        avg_ms = int(np.ceil(1e3 / fps)) if fps > 0 else 0
        if avg_fps > 0:
            self.results.append(desc="Average Frequency", units="Hz", value=avg_fps, trunc=True)
            self.results.append(desc="Average Runtime", units="ms", value=avg_ms, trunc=True)

    def _evaluate_closures(self):
        self.results.append(
            desc="Number of loop closures found",
            units="closures",
            value=len(self.retro_slam.loop_closures),
        )

    def _write_closures(self):
        import matplotlib.pyplot as plt
        import numpy as np

        poses = np.asarray(self.poses)
        key_poses = self.retro_slam.keyposes

        fig, ax = plt.subplots()
        ax.scatter(poses[:, 0, 3], poses[:, 1, 3], s=0.1, color="black")
        for closure in self.retro_slam.loop_closures:
            i = closure.ref_scan_id
            j = closure.query_scan_id
            p_i = key_poses[i]
            p_j = key_poses[j]
            ax.plot([p_i[0, -1], p_j[0, -1]], [p_i[1, -1], p_j[1, -1]], color="red", linewidth=1)

        fig.savefig(os.path.join(self.results_dir, "trajectory.png"), dpi=300, bbox_inches="tight")
        plt.close(fig)

    def _write_graph(self):
        self.retro_slam.write_pose_graph(os.path.join(self.results_dir, "pose_graph.g2o"))

    @staticmethod
    def save_poses_kitti_format(filename: str, poses: np.ndarray):
        def _to_kitti_format(poses: np.ndarray) -> np.ndarray:
            return poses[:, :3].reshape(-1, 12)

        np.savetxt(fname=f"{filename}_kitti.txt", X=_to_kitti_format(poses))

    @staticmethod
    def save_poses_tum_format(filename, poses, timestamps):
        def _to_tum_format(poses, timestamps):
            tum_data = np.zeros((len(poses), 8))
            with contextlib.suppress(ValueError):
                for idx in range(len(poses)):
                    tx, ty, tz = poses[idx, :3, -1].flatten()
                    qw, qx, qy, qz = Quaternion(matrix=poses[idx], atol=0.01).elements
                    tum_data[idx] = np.r_[float(timestamps[idx]), tx, ty, tz, qx, qy, qz, qw]
                tum_data.flatten()
                return tum_data.astype(np.float64)

        np.savetxt(fname=f"{filename}_tum.txt", X=_to_tum_format(poses, timestamps), fmt="%.4f")

    def _calibrate_poses(self, poses):
        return (
            self._dataset.apply_calibration(poses)
            if hasattr(self._dataset, "apply_calibration")
            else poses
        )

    def _get_frames_timestamps(self):
        return (
            self._dataset.get_frames_timestamps()
            if hasattr(self._dataset, "get_frames_timestamps")
            else np.arange(0, self._n_scans, 1.0)
        )

    def _save_poses(self, filename: str, poses, timestamps):
        np.save(filename, poses)
        self.save_poses_kitti_format(filename, poses)
        self.save_poses_tum_format(filename, poses, timestamps)

    def _write_result_poses(self):
        self._save_poses(
            filename=f"{self.results_dir}/{self.dataset_sequence}_poses",
            poses=self._calibrate_poses(self.poses),
            timestamps=self._get_frames_timestamps(),
        )

    def _write_gt_poses(self):
        if not self.has_gt:
            return
        self._save_poses(
            filename=f"{self.results_dir}/{self._dataset.sequence_id}_gt",
            poses=self._calibrate_poses(self.gt_poses),
            timestamps=self._get_frames_timestamps(),
        )

    def _get_fps(self):
        times_nozero = self.times[self.times != 0]
        total_time_s = np.sum(times_nozero) * 1e-9
        return float(times_nozero.shape[0] / total_time_s) if total_time_s > 0 else 0

    def _write_log(self):
        if not self.results.empty():
            self.results.log_to_file(
                f"{self.results_dir}/result_metrics.log",
                f"Results for {self.dataset_name} Sequence {self.dataset_sequence}",
            )

    def _write_cfg(self):
        write_config(self.slam_config, os.path.join(self.results_dir, "config.yml"))

    @staticmethod
    def _get_results_dir(out_dir: str):
        def get_current_timestamp() -> str:
            return datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        results_dir = os.path.join(os.path.realpath(out_dir), get_current_timestamp())
        latest_dir = os.path.join(os.path.realpath(out_dir), "latest")
        os.makedirs(results_dir, exist_ok=True)
        os.unlink(latest_dir) if os.path.exists(latest_dir) or os.path.islink(latest_dir) else None
        os.symlink(results_dir, latest_dir)
        return results_dir

    def _create_output_dir(self):
        self.results_dir = self._get_results_dir(self.slam_config.out_dir)
