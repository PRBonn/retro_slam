# RetroSLAM Python Package

Python bindings and utilities for the RetroSLAM SLAM system.

## Overview

This package provides:
- Python bindings to the C++ RetroSLAM core
- Dataset loaders for various formats (KITTI, Ouster, ROS bags, etc.)
- Pipeline orchestration
- **Rerun-based 3D visualization** (NEW!)

## Quick Start

### Installation

```bash
pip install -e .
```

Or using the Makefile:

```bash
make build-python
```

### Basic Usage

```python
from retro_slam import SlamPipeline
from retro_slam.datasets.ouster import OusterDataset

# Load dataset
dataset = OusterDataset("data/your_sequence", data_dir="path/to/data")

# Create and run pipeline
pipeline = SlamPipeline(
    dataset=dataset,
    config_file="config.yaml",
)

results = pipeline.run()
```

## Visualizer (NEW!)

RetroSLAM now includes real-time 3D visualization using [Rerun](https://rerun.io/).

### Installation

```bash
pip install 'rerun-sdk[open3d]'
```

### Usage

```python
from retro_slam.visualizers.rerun_visualizer import RetroSLAMVisualizer

# Create visualizer
visualizer = RetroSLAMVisualizer(app_name="RetroSLAM")

try:
    # Visualize during processing
    for scan_idx in range(n_scans):
        scan, timestamps = dataset[scan_idx]

        # Process...

        # Visualize
        visualizer.visualize_scan(
            point_cloud=scan,
            pose=pose_matrix,
            timestamp=timestamps[0],
            frame_id=f"frame_{scan_idx}",
        )

    # Show results
    visualizer.visualize_trajectory(poses=poses, timestamps=timestamps)
    visualizer.visualize_metrics(ate=0.15, rpe=2.5, fps=30)

finally:
    visualizer.finish()
```

See the `visualizers/` directory for complete documentation and examples.

## Directory Structure

```
python/
├── retro_slam/              # Main package
│   ├── config/             # Configuration
│   ├── datasets/           # Dataset loaders
│   ├── pybind/             # C++ bindings
│   └── tools/              # Utilities
├── visualizers/            # Rerun visualization (NEW!)
│   ├── rerun_visualizer.py    # Core visualizer
│   ├── example_usage.py       # Usage examples
│   ├── pipeline_integration.py # Pipeline integration
│   └── README.md              # Documentation
├── CMakeLists.txt          # Build configuration
└── pyproject.toml          # Python packaging
```

## Datasets Supported

- KITTI Odometry
- Ouster LiDAR
- ROS Bags
- Mulran
- NCD
- NCLT
- Apollo
- HeliPR
- MCAP

See `retro_slam/datasets/` for all dataset loaders.

## Configuration

Configuration is handled via YAML files. See the main README.md for configuration options.

## Development

### Building from Source

```bash
mkdir build && cd build
cmake ..
make
make install
```

### Running Tests

```bash
make test-python
```

## License

MIT License - see LICENSE file for details.
