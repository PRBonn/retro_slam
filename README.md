<div align="center">
    <h1>Retro-SLAM</h1>
    <a href="https://github.com/PRBonn/retro_slam/releases"><img src="https://img.shields.io/github/v/release/PRBonn/retro_slam?label=version" /></a>
    <a href="https://github.com/PRBonn/retro_slam/blob/main/LICENSE"><img src="https://img.shields.io/github/license/PRBonn/retro_slam" /></a>
    <a href="https://github.com/PRBonn/retro_slam/blob/main/"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/PRBonn/retro_slam/blob/main/"><img src="https://img.shields.io/badge/mac%20os-000000?&logo=apple&logoColor=white" /></a>
    <br />
    <br />
    <a href="https://github.com/PRBonn/retro_slam/blob/main/README.md#Install">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://www.ipb.uni-bonn.de/pdfs/gupta2026arxiv.pdf">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://github.com/PRBonn/retro_slam/issues>Contact Us</a>
  <br />
  <br />

Retro-SLAM: Improving Map Consistency in Graph-Based LiDAR SLAM Through Information-Aware Odometry and Retroactive Loop Closure

</div>

<hr />

## Install

```
pip install retro_slam
```

## Running the system
Next, follow the instructions on how to run the system by typing:
```
retro_slam_pipeline --help
```

This should print the following help message:

<img width="2531" height="585" alt="image" src="https://github.com/user-attachments/assets/7ab0ce96-9090-48b5-bb99-1f7485421fd7" />


### Config
You can generate a default `config.yaml` by typing:

```
retro_slam_dump_config
```

which will generate a `retro_slam.yaml` file. Now, you can modify the parameters and pass the file to the `--config` option when running the `retro_slam_pipeline`.

### Install Python API (developer mode)
For development purposes:

```
python3 -m pip install --upgrade pip
git clone https://github.com/PRBonn/retro-slam.git
cd retro-slam
make editable
```

### ROS2 support

```sh
cd ~/ros2_ws/src/ && git clone https://github.com/PRBonn/retro_slam && cd ~/ros2_ws/ && colcon build --packages-select retro_slam_ros
```
For more detailed instructions on the ROS wrapper, please visit this [README](retro_slam_ros/README.md)


## Citation
If you use this library for any academic work, please cite our original paper:
```bib
@article{gupta2026arxiv,
  author   = {S. Gupta and N. Trekel and L.Wiesmann and C. Stachniss},
  title    = {{Improving Map Consistency in Graph-Based LiDAR SLAM Through Information-Aware Odometry and Retroactive Loop Closure}},
  journal  = {arXiv preprint},
  year     = 2026,
  volume   = {arXiv:2607.13516},
  url      = {https://arxiv.org/pdf/2607.13516},
}
```

## Note
1. We use a fork of [KISS-ICP](https://github.com/saurabh1002/kiss-icp) for LiDAR odometry and the uncertainty estimation proposed in the Retro-SLAM manuscript is implemented in this fork as well.
2. We use the [MapClosures](https://github.com/PRBonn/MapClosures) repository for local-map-level loop closure detection in our work.

## Acknowledgements

This project builds on top of [KISS-SLAM](https://github.com/PRBonn/kiss-slam).

## Contributing

We envision this as a community-driven project. We love to see how the project is growing, thanks to the contributions from the community. We would love to see your face in the list below; open a Pull Request!

<a href="https://github.com/PRBonn/retro_slam/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=PRBonn/retro_slam" />
</a>

## Contact Us
For questions or feedback:
- GitHub Issues: https://github.com/PRBonn/retro_slam/issues
