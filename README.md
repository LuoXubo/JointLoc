# JointLoc

Code for "JointLoc: A Real-time Visual Localization Framework for Planetary UAVs Based on Joint Relative and Absolute Pose Estimation" (IROS 2024 Under review).

![JointLoc](/figs/pipeline.png)

## Update

### 2024.04.26

Most codes of the AbsLoc and RelLoc modules are uploaded. Meanwhile, the main files are deleted due to the privacy of the project. We will provide the main files and the test datset after the paper is accepted.

## Getting Started

### Installation

```bash
git clone https://github.com/LuoXubo/JointLoc.git
cd JointLoc
```

Setup the environment:

```bash
conda env create -f environment.yml
conda activate JointLoc
```

Besides the python packages, you also need to configure the OpenCV, Eigen, Ceres Solver, Redis and Protobuf. You can refer to the ORB-SLAM3 to install them.

## Quick Demo

### Download the dataset

Download the test dataset from [Google Drive](https://drive.google.com/file/d/1Q6Q6Q1Z9) and extract it to the `JointLoc` directory. The given dataset and your own dataset should have the following structure:

```
JointLoc
├── dataset
│   ├── uav
│   │   ├── 000000.png
│   │   ├── 000001.png
│   │   └── ...
│   ├── satellite
│   │   ├── 000000.png
│   │   ├── 000001.png
│   │   └── ...
│   ├── groundtruth.txt
│   └── ...
└── ...
```

### Run the demo

```bash
sh demo.sh
```

This script will open 3 temrminals:

1. The first terminal will open redis-server.
2. The second terminal runs the relative localization (SLAM) system in /RelLoc/ directory.
3. The third terminal runs the absolute localization system in /AbsLoc/ directory.

## Evaluation

The estimated 6-DoF poses of JointLoc will be saved in `results/poses.txt`. You can evaluate the performance by running the following script:

```bash
evo_ape tum dataset/groundtruth.txt results/poses.txt -pv
```

## Compared with ORB-SLAM2 and ORB-SLAM3

We also provide the code to run ORB-SLAM2 and ORB-SLAM3 on the same dataset. You can run the following script:

```bash
sh run_orb_slam2.sh
sh run_orb_slam3.sh
```

### Visualization

![ORB-SLAM2](figs/gif.gif)

## Acknowledgement

This work incorporates many open-source codes. We extend our gratitude to the authors of the software.

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [SuperPoint](https://github.com/rpautrat/SuperPoint)
- [LightGlue](https://github.com/cvg/LightGlue)

## Citation

If you find this repo useful for your research, please consider citing the paper

```
@misc{luo2024jointloc,
      title={JointLoc: A Real-time Visual Localization Framework for Planetary UAVs Based on Joint Relative and Absolute Pose Estimation},
      author={Xubo Luo and Xue Wan and Yixing Gao and Yaolin Tian and Wei Zhang and Leizheng Shu},
      year={2024},
      eprint={2405.07429},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=LuoXubo/JointLoc&type=Date)](https://star-history.com/#LuoXubo/JointLoc&Date)
