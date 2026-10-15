# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
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
import numpy as np

try:
    import rerun
except ImportError:
    print("Please install rerun with `pip install rerun-sdk` to enable visualization.")


VIRIDIS = np.array(
    [
        [68, 1, 84],
        [71, 44, 122],
        [59, 81, 139],
        [44, 113, 142],
        [33, 144, 140],
        [39, 173, 129],
        [92, 200, 99],
        [170, 220, 50],
        [253, 231, 37],
    ],
    dtype=np.uint8,
)


def height_colors(points: np.ndarray, color_map: np.ndarray = VIRIDIS) -> np.ndarray:
    z = points[:, 2]
    z_min, z_max = np.percentile(z, 1), np.percentile(z, 99)
    z_clipped = np.clip(z, z_min, z_max)

    norm_z = np.zeros_like(z) if z_max == z_min else (z_clipped - z_min) / (z_max - z_min)

    idx = norm_z * (len(color_map) - 1)
    idx_low = np.floor(idx).astype(int)
    idx_high = np.clip(idx_low + 1, 0, len(color_map) - 1)
    alpha = idx - idx_low
    return (
        (1 - alpha)[:, None] * color_map[idx_low] + alpha[:, None] * color_map[idx_high]
    ).astype(np.uint8)


class Viz:
    def __init__(self):
        rerun.init("retro_slam")
        rerun.spawn(memory_limit="2GB")

    def log_slam_state(
        self, slam_object, scan: np.ndarray | None = None, time_ns: int | None = None
    ):
        state = slam_object.visualization_state
        global_trajectory = np.asarray(state["global_trajectory"], dtype=np.float64)
        local_trajectory = np.asarray(state["local_trajectory"], dtype=np.float64)
        keyposes = np.asarray(state["keyposes"], dtype=np.float64)
        latest_scan = np.asarray(state["latest_scan"], dtype=np.float64)
        latest_pose = np.asarray(state.get("latest_scan_pose", np.eye(4)), dtype=np.float64)
        loop_closures = state["loop_closures"]
        if scan is not None:
            latest_scan = np.asarray(scan, dtype=np.float64)
        if time_ns is None:
            time_ns = 0
        scan_time_s = time_ns * 1e-9
        rerun.set_time("data_time", timestamp=scan_time_s)
        rerun.log("world/view_anchor", rerun.Transform3D(translation=latest_pose[:3, 3]))

        if global_trajectory.size > 0:
            traj_positions = global_trajectory[:, :3, 3]
            rerun.log("world/global_trajectory", rerun.LineStrips3D([traj_positions]))

        if local_trajectory.size > 0:
            traj_positions = (keyposes[-1] @ local_trajectory)[:, :3, 3]
            rerun.log("world/local_trajectory", rerun.LineStrips3D([traj_positions]))

        if keyposes.size > 0:
            key_positions = keyposes[:, :3, 3]
            rerun.log(
                "world/submap_keyposes",
                rerun.Points3D(key_positions, radii=0.5, colors=[255, 184, 75]),
            )

        if latest_scan.size > 0:
            scan_world = (latest_pose[:3, :3] @ latest_scan.T).T + latest_pose[:3, 3]
            rerun.log(
                "world/latest_scan",
                rerun.Points3D(scan_world, radii=0.05, colors=height_colors(scan_world)),
            )
        if len(loop_closures) > 0:
            rerun.log(
                f"world/loop_closures",
                rerun.LineStrips3D(
                    [np.array([keyposes[i][:3, 3], keyposes[j][:3, 3]]) for i, j in loop_closures],
                    radii=0.5,
                    colors=[0, 255, 0],
                ),
            )
