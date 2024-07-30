#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   estimate.py
@Time    :   2024/05/04 09:05:06
@Author  :   Xubo Luo 
@Version :   1.0
@Contact :   luoxubo@hotmail.com
@License :   (C)Copyright 2017-2018, Liugroup-NLPR-CASIA
@Desc    :   Accept images from server. Adaptively adjust the search radius.
             And send the results of the selected key frames to slam. 
'''
import os
import time

import cv2
import numpy as np
import redis

import pose_pb2, imageInfo_pb2
from lightglue import LightGlue, SuperPoint, DISK
from lightglue.utils import (coords, load_image, match_pair, transformation, byte2tensor)
from align import align
import argparse

cnt = 0
length, stepsize = 11, 250
central_coords_x = 500
central_coords_y = 500
pt_drone = np.matrix([int(central_coords_x/2), int(central_coords_y/2), 1])

root_dataset = '../Datasets/Mars/'


best_match, best_score, best_sub = 100, 1, 'test'

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='JointLoc')
    parser.add_argument('--terrain', type=str, default='Crater', help='Crater, Gravel, Mountain')
    args = parser.parse_args()

    terrain_type = args.terrain

    gt_XY = os.path.join(root_dataset, terrain_type, 'uav/gt.txt')
    subsats = os.path.join(root_dataset, terrain_type, 'satellite/')
    with open(gt_XY, 'r') as f:
        gtlines = f.readlines()
    f.close()

    num_uav = len(gtlines)
    sat_imgs = os.listdir(subsats)
    sat_imgs.sort(key=lambda x:int(x.split('_')[0]))

    regions = []
    for sat_img in sat_imgs:
        region = [sat_img]
        idx, x, y = sat_img.split('.')[0].split('_')
        idx = int(idx)
        x = int(x)
        y = int(y)
        
        neighbors = ['%d_%d_%d.png'%(idx-1, x-stepsize, y),
                '%d_%d_%d.png'%(idx+1, x+stepsize, y),
                '%d_%d_%d.png'%(idx-length, x, y-stepsize),
                '%d_%d_%d.png'%(idx+length, x, y+stepsize),
                '%d_%d_%d.png'%(idx+length+1, x+stepsize, y+stepsize),
                '%d_%d_%d.png'%(idx-length+1, x+stepsize, y-stepsize),
                '%d_%d_%d.png'%(idx+length-1, x-stepsize, y+stepsize),
                '%d_%d_%d.png'%(idx-length-1, x-stepsize, y-stepsize)
            ]
        for neighbor in neighbors:
            if neighbor in sat_imgs:
                region.append(neighbor)
        regions.append(region)

    if terrain_type == 'Crater':
        pre_region = regions[79]
    elif terrain_type == 'Gravel':
        pre_region = regions[61]
    else:
        pre_region = regions[107]
    print('Pre-processing finished ...')

    ### Load SuperPoint and LightGlue
    extractor = SuperPoint(max_num_keypoints=2048,
                        nms_radius=3).eval().cuda()  # load the extractor
    match_conf = {
        'width_confidence': 0.99,  # for point pruning
        'depth_confidence': 0.95,  # for early stopping,
    }
    matcher = LightGlue(pretrained='superpoint', **match_conf).eval().cuda()
    
    ### init Redis connection
    image = imageInfo_pb2.imageInfo()
    pose = pose_pb2.poseInfo()
    conn = redis.Redis(host='127.0.0.1', port=6379)
    Redis = redis.Redis(host='localhost', port=6379, db=0)
    sub = conn.pubsub()
    sub.subscribe('server')
    print('Begin to listen ...')

    timecost = 0
    for cnt in range(num_uav):
        time.sleep(0.1)
        key = 'img%d'%cnt
        msg = Redis.get(key)
        image.ParseFromString(msg)
        if image.timestamp == '-1':
            print('Exit ...')
            break
        
        if cnt%4 != 0:
            continue
        timestamp = image.timestamp
        image_data = image.image_data
        tensorA = byte2tensor(image_data)

        print('Begin to localize image with the timestamp of %s ...'%timestamp)
        
        """
        Begin to localization
        """
        start = time.time()
        best_score, best_match = 1.0, 0
        for satellite in pre_region:
            sat = subsats + '/' + satellite
            tensorB, scalesB = load_image(sat, grayscale=False)
            
            pred = match_pair(extractor, matcher, tensorA, tensorB)
            kpts0, kpts1, matches = pred['keypoints0'], pred['keypoints1'], pred['matches']
            m_kpts0, m_kpts1 = kpts0[matches[..., 0]], kpts1[matches[..., 1]]
            if(len(m_kpts0) < 4):
                continue
            H, _ = cv2.findHomography(m_kpts0.numpy(), m_kpts1.numpy(), cv2.RANSAC, 5.0)
            matching_score = pred['matching_scores'].numpy().mean()

            if len(matches) > best_match:
                best_match = len(matches)
                best_score = matching_score
                best_sub = satellite
                best_H = H
        
        
        gtline = gtlines[cnt].split('\n')[0]
        pre_timestamp = gtline.split(' ')[0]  # 直接使用gt.txt里已经除以500后的时间戳
        if pre_timestamp != timestamp:
            print("Warning!\n\n\n The timestamps are not aligned!\n\n")
        z, qw, qx, qy, qz = gtline.split(' ')[3:]
        x_gt, y_gt = gtline.split(' ')[1:3]

        startX, startY = best_sub.split('.')[0].split('_')[1], best_sub.split('.')[0].split('_')[2]
        pt_sate = transformation(pt_drone.T, best_H)
        x, y = coords(pt_sate)
        resX, resY = float(startX) + x, float(startY) + y
        end = time.time()

        print('Timestamp: %s        Position:{%f, %f}       matching points: %d     matchability:%f     timecost:%f s' % (
            timestamp, resX, resY, best_match, best_score, (end-start)))
        timecost += (end-start)
        """
        End of localization
        """
        
        
        """
        Begin to update the candidates
        """
        best_idx = int(best_sub.split('.')[0].split('_')[0])
        if best_score >= 0.7:
            pre_region = [regions[best_idx-1][0]]
        else:
            pre_region = regions[best_idx-1]
        """
        End of update the candidates
        """

        """
        Publish the pose
        """
        [resX, resY, z] = align([resX, resY, float(z)])
        pose.timestamp = timestamp
        pose.x = float(x_gt)
        pose.y = float(y_gt)
        pose.z = z
        pose.qw = float(qw)
        pose.qx = float(qx)
        pose.qy = float(qy)
        pose.qz = float(qz)
        pose.conf = best_score
        pose.image_data = image_data
        msg = pose.SerializeToString()

        print("g2l key frame id: %d"%cnt)

        conn.publish("g2l", msg)

        with open('log.txt', 'a+') as logfile:
            logfile.write('Timestamp: %s        Position:{%f, %f}       matching points: %d     matchability:%f     timecost:%f s       best sat:%s\n' % (
            timestamp, resX, resY, best_match, best_score, (end-start), best_sub))
    
        with open('./abs_res.txt', 'a+') as absfile:
            absfile.write('%s %f %f %f %s %s %s %s\n'%(timestamp, resX, resY, z, qw, qx, qy, qz))

    print('\n\nEnd of localization ...')
    print('Average time cost: %f s'%(timecost/num_uav))
    print('The experiment log is saved in log.txt')
    print('The absolute position result is saved in abs_res.txt')
    logfile.close()
    # absfile.close()
    pose.timestamp = '-1'
    msg = pose.SerializeToString()
    conn.publish('g2l', msg)