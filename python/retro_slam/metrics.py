# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
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
import copy
from typing import Tuple

import numpy as np
from evo.core import metrics
from evo.core.filters import FilterException
from evo.core.trajectory import PosePath3D
from evo.core.units import Unit


def sequence_error(gt_poses: np.ndarray, results_poses: np.ndarray) -> Tuple[float, float]:
    """Spits the sequence error for a given trajectory in camera coordinate frames."""
    traj_ref = PosePath3D(poses_se3=gt_poses)
    traj_est = PosePath3D(poses_se3=results_poses)
    traj_est_aligned = copy.deepcopy(traj_est)
    traj_est_aligned.align(traj_ref, correct_scale=False, correct_only_scale=False)

    # Compute APE
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref, traj_est_aligned))
    ape_rmse = ape_metric.get_statistic(metrics.StatisticsType.rmse)

    deltas = [100, 200, 300, 400, 500, 600, 700, 800]  # Deltas as class attribute
    rpe_means = []

    for delta in deltas:
        # Calculate RPE
        rpe_metric = metrics.RPE(
            metrics.PoseRelation.point_distance_error_ratio,
            delta=delta,
            delta_unit=Unit.meters,
            pairs_from_reference=False,
        )
        try:
            rpe_metric.process_data((traj_ref, traj_est_aligned))
        except FilterException:
            continue

        # Collect statistics for this delta
        rpe_stats = rpe_metric.get_all_statistics()
        rpe_means.append(rpe_stats["mean"])

    # Calculate overall statistics across all deltas
    rpe = np.mean(np.array(rpe_means)) if len(rpe_means) > 0 else "-"
    return ape_rmse, rpe
