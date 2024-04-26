# RelLoc

## Introduction

This is the relative localization module of JointLoc. The module takes the UAV images as input, and outputs the relative 6-DoF poses of the UAV.

## Notice

The main file is deleted due to the privacy of the project. We will provide the main file after the paper is accepted.

## Installation

The RelLoc module relies on the following packages:

- Eigen
- Pangolin
- OpenCV
- Redis
- Protobuf

## Usage

### 1. Build this module:

```bash
sh build.sh
```

### 2. Run the RelLoc process:

```bash
sh run.sh
```

## Acknowledgement

This module is highly inspired by the following works:

- [ORB-SLAM2]
- [ORB-SLAM3]
