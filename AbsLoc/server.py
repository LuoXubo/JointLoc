"""
@Description :   Send images to slam and global localization threads via redis.
@Author      :   Xubo Luo 
@Time        :   2023/08/27 15:34:20
"""
import os
import numpy as np
import redis
from tqdm import tqdm
import time

import pose_pb2, imageInfo_pb2

### init
root_dataset = '/home/xubo/Dataset/Mars/Orbit/'
gt_XY = root_dataset + 'uav/gt.txt'
uav_path = root_dataset + 'uav/images/'
uav_file_list = os.listdir(uav_path)
uav_file_list.sort()
with open(gt_XY, 'r') as f:
    gtlines = f.readlines()
f.close()


channel = 'server'
image = imageInfo_pb2.imageInfo()
conn = redis.Redis(host='127.0.0.1', port=6379)
r = redis.Redis(host='localhost', port=6379, db=0)
cnt = 0

for uav_file in tqdm(uav_file_list):

    uav = uav_path + '/' + uav_file
    gtline = gtlines[cnt].split('\n')[0]
    timestamp = gtline.split(' ')[0]  # 直接使用gt.txt里已经除以500后的时间戳
    image.timestamp = timestamp
    image.image_data = open(uav, 'rb').read()

    msg = image.SerializeToString()

    conn.publish(channel, msg)
    r.set('img%d'%cnt, msg)
    # time.sleep(0.5)
    cnt+=1

image.timestamp = '-1'
msg = image.SerializeToString()
conn.publish(channel, msg)