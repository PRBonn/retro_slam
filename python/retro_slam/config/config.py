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
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

import yaml
from kiss_icp.config.config import (
    AdaptiveThresholdConfig,
    DataConfig,
    MappingConfig,
    RegistrationConfig,
)
from map_closures.config.config import MapClosuresConfig
from pydantic import BaseModel
from pydantic_settings import BaseSettings, SettingsConfigDict


class KissOdometryConfig(BaseModel):
    preprocessing: DataConfig = DataConfig()
    registration: RegistrationConfig = RegistrationConfig()
    mapping: MappingConfig = MappingConfig()
    adaptive_threshold: AdaptiveThresholdConfig = AdaptiveThresholdConfig()


class LoopCloserConfig(BaseModel):
    detector: MapClosuresConfig = MapClosuresConfig()
    fitness_threshold: float = 0.25


class LocalMapperConfig(BaseModel):
    submap_voxel_size: float = 0.5
    localmap_voxel_size: float = 0.5
    submap_splitting_distance: float = 25.0
    localmap_splitting_distance: float = 100.0


class PoseGraphOptimizerConfig(BaseModel):
    max_iterations: int = 10
    verbose: bool = False


class RetroSlamConfig(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="retro_slam_")
    out_dir: str = "slam_output"
    odometry: KissOdometryConfig = KissOdometryConfig()
    local_mapper: LocalMapperConfig = LocalMapperConfig()
    loop_closer: LoopCloserConfig = LoopCloserConfig()
    pose_graph_optimizer: PoseGraphOptimizerConfig = PoseGraphOptimizerConfig()


class ConfigDumper(yaml.Dumper):
    # HACK: insert blank lines between top-level objects
    # inspired by https://stackoverflow.com/a/44284819/3786245
    def write_line_break(self, data=None):
        super().write_line_break(data)

        if len(self.indents) == 1:
            super().write_line_break()


def _yaml_source(config_file: Optional[Path]) -> Dict[str, Any]:
    data = None
    if config_file is not None:
        with open(config_file) as cfg_file:
            data = yaml.safe_load(cfg_file)
    return data or {}


def load_config(config_file: Optional[Path]) -> RetroSlamConfig:
    """Load configuration from an Optional yaml file. Additionally, deskew and max_range can be
    also specified from the CLI interface"""

    config = RetroSlamConfig(**_yaml_source(config_file))

    # Use specified voxel size or compute one using the max range
    if config.odometry.mapping.voxel_size is None:
        config.odometry.mapping.voxel_size = float(config.odometry.preprocessing.max_range / 100.0)

    return config


def write_config(config: RetroSlamConfig = RetroSlamConfig(), filename: str = "retro_slam.yaml"):
    with open(filename, "w") as outfile:
        yaml.dump(
            config.model_dump(),
            outfile,
            Dumper=ConfigDumper,
            default_flow_style=False,
            sort_keys=False,
            indent=4,
        )
