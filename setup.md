# Details of the setup

## 1. Python packages

Setup the environment:

```bash
conda create -n JointLoc python=3.7 --yes
conda activate JointLoc
cd AbsLoc
pip install -r requirements.txt
```

## 2. C++ packages

Besides the python packages, you also need to configure the `OpenCV`, `Eigen`, `Pangolin`, and `Ceres`. You can refer to the [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) to install them.

The `Redis` and `Protobuf` are used to connect the absolute localization and relative localization modules.

### 2.1 Redis

```bash
wget https://download.redis.io/releases/redis-6.2.6.tar.gz
tar xzf redis-6.2.6.tar.gz
sudo mv redis-6.2.6 /usr/local/redis
cd /usr/local/redis
make
make install PREFIX=/usr/local/redis
```

```bash
wget https://github.com/redis/hiredis/archive/refs/tags/v1.1.0.zip
unzip hiredis-1.1.0.zip
cd hiredis-1.1.0
mkdir build && cd build
cmake ..
make -j8
sudo make install

wget https://github.com/sewenew/redis-plus-plus/archive/refs/tags/1.3.7.zip
unzip redis-plus-plus-1.3.7.zip
cd redis-plus-plus-1.3.7
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

### 2.2 Protobuf

```bash
wget https://github.com/protocolbuffers/protobuf/archive/refs/tags/v3.20.0.zip
unzip protobuf-3.20.0.zip
cd protobuf-3.20.0
./autogen.sh
./configure
make -j8
sudo make install
sudo ldconfig
```

**Notice: You may need to download the `googletest` to replace the `googletest` in the `protobuf/third_party/` folder.**

## 3. Compile the project

Use the `build.sh` in RelLoc to compile the project.

```bash
cd RelLoc
./build.sh
```

## 4. Quick Demo

Once you have installed all the dependencies, you can run the quick demo to test the system.

```bash
cd JointLoc
sh RelLoc/redis.sh      # run redis server
sh AbsLoc/server.sh     # publish images to redis
sh RelLoc/relloc.sh     # run relative localization
sh AbsLoc/absloc.sh     # run absolute localization
```
