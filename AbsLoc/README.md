# AbsLoc

## Introduction

This is the absolute localization module of JointLoc. The module takes the UAV images and the satellite map as input, and outputs the absolute location of the UAV.

## Notice

The main file is deleted due to the privacy of the project. We will provide the main file after the paper is accepted.

## Installation

```bash
pip install redis
pip install numpy
pip install opencv-python
pip install torch
pip install torchvision

```

## Usage

### 1. Open the Redis client

### 2. Open the server

```bash
python server.py
```

### 3. Open the absolute localization module

```bash
sh run.sh
```

## Acknowledgement

This module is highly inspired by the following works:

- [SuperPoint]
- [LightGlue]
