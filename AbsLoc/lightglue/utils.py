from pathlib import Path
import torch
import cv2
import numpy as np
import math
from typing import Union, List, Optional
# import  skimage.metrics as metrics
import time

def read_image(path: Path, grayscale: bool = False) -> np.ndarray:
    """Read an image from path as RGB or grayscale"""
    mode = cv2.IMREAD_GRAYSCALE if grayscale else cv2.IMREAD_COLOR
    image = cv2.imread(str(path), mode)
    if image is None:
        raise IOError(f'Could not read image at {path}.')
    if not grayscale:
        image = image[..., ::-1]
    return image


def numpy_image_to_torch(image: np.ndarray) -> torch.Tensor:
    """Normalize the image tensor and reorder the dimensions."""
    if image.ndim == 3:
        image = image.transpose((2, 0, 1))  # HxWxC to CxHxW
    elif image.ndim == 2:
        image = image[None]  # add channel axis
    else:
        raise ValueError(f'Not an image: {image.shape}')
    return torch.tensor(image / 255., dtype=torch.float)


def resize_image(image: np.ndarray, size: Union[List[int], int],
                 fn: str, interp: Optional[str] = 'area') -> np.ndarray:
    """Resize an image to a fixed size, or according to max or min edge."""
    h, w = image.shape[:2]

    fn = {'max': max, 'min': min}[fn]
    if isinstance(size, int):
        scale = size / fn(h, w)
        h_new, w_new = int(round(h*scale)), int(round(w*scale))
        scale = (w_new / w, h_new / h)
    elif isinstance(size, (tuple, list)):
        h_new, w_new = size
        scale = (w_new / w, h_new / h)
    else:
        raise ValueError(f'Incorrect new size: {size}')
    mode = {
        'linear': cv2.INTER_LINEAR,
        'cubic': cv2.INTER_CUBIC,
        'nearest': cv2.INTER_NEAREST,
        'area': cv2.INTER_AREA}[interp]
    return cv2.resize(image, (w_new, h_new), interpolation=mode), scale


def load_image(path: Path, grayscale: bool = False, resize: int = None,
               fn: str = 'max', interp: str = 'area') -> torch.Tensor:
    img = read_image(path, grayscale=grayscale)
    scales = [1, 1]
    if resize is not None:
        img, scales = resize_image(img, resize, fn=fn, interp=interp)
    return numpy_image_to_torch(img), torch.Tensor(scales)


def match_pair(extractor, matcher, image0, image1, scales0=None, scales1=None):
    device = image0.device
    data = {'image0': image0[None].cuda(), 'image1': image1[None].cuda()}
    img0, img1 = data['image0'], data['image1']
    tik = time.time()
    feats0, feats1 = extractor({'image': img0}), extractor({'image': img1})
    tok = time.time()
    # print('Feature extraction time cost: %f s'%(tok-tik))
    pred = {**{k+'0': v for k, v in feats0.items()},
            **{k+'1': v for k, v in feats1.items()},
            **data}
    tik = time.time()
    pred = {**pred, **matcher(pred)}
    tok = time.time()
    # print('Matching time cost: %f s'%(tok-tik))
    pred = {k: v.to(device).detach()[0] if
            isinstance(v, torch.Tensor) else v for k, v in pred.items()}
    if scales0 is not None:
        pred['keypoints0'] = (pred['keypoints0'] + 0.5) / scales0[None] - 0.5
    if scales1 is not None:
        pred['keypoints1'] = (pred['keypoints1'] + 0.5) / scales1[None] - 0.5
    del feats0, feats1
    torch.cuda.empty_cache()

    # create match indices
    kpts0, kpts1 = pred['keypoints0'], pred['keypoints1']
    matches0, mscores0 = pred['matches0'], pred['matching_scores0']
    valid = matches0 > -1
    matches = torch.stack([torch.where(valid)[0], matches0[valid]], -1)
    # m_kpts0, m_kpts1 = pred['keypoints0'][matches[..., 0]], pred['keypoints1'][matches[..., 1]]
    return {**pred, 'matches': matches, 'matching_scores': mscores0[valid]}

def transformation(pt_drone, H):
    return H @ pt_drone

def coords(mat):
    a = mat[0][0]/mat[2][0]
    b = mat[1][0]/mat[2][0]
    return float(a), float(b)

def sigmoid(x):
    return 1 / (1 + np.exp(-1 * x))

# def ssim_similarity(imgA, imgB):
#     return metrics.structural_similarity(imgA, imgB, channel_axis = 1)

def draw_matches(img_A, img_B, keypoints0, keypoints1):
    
    p1s = []
    p2s = []
    dmatches = []
    for i, (x1, y1) in enumerate(keypoints0):

        p1s.append(cv2.KeyPoint(x1, y1, 1))
        p2s.append(cv2.KeyPoint(keypoints1[i][0], keypoints1[i][1], 1))
        j = len(p1s) - 1
        dmatches.append(cv2.DMatch(j, j, 1))

    matched_images = cv2.drawMatches(cv2.cvtColor(img_A, cv2.COLOR_RGB2BGR), p1s,
                                     cv2.cvtColor(img_B, cv2.COLOR_RGB2BGR), p2s, dmatches, None)

    return matched_images

def rotate(image, angle, center=None, scale=1.0): #逆时针旋转
    (h, w) = image.shape[:2] #2
    if center is None: #3
        center = (w // 2, h // 2) #4
 
    M = cv2.getRotationMatrix2D(center, angle, scale) #5
 
    rotated = cv2.warpAffine(image, M, (w, h)) #6
    return rotated #7

def rotateP(angle,valuex,valuey,pointx,pointy):
    valuex = np.array(valuex)
    valuey = np.array(valuey)
    sRotatex = (valuex-pointx)*math.cos(angle) + (valuey-pointy)*math.sin(angle) + pointx
    sRotatey = (valuey-pointy)*math.cos(angle) - (valuex-pointx)*math.sin(angle) + pointy
    return sRotatex,sRotatey

def getR(score):
    x = score
    R = 4
    r = R * math.exp(-1 * math.pi * R**2 * x**2)
    return math.ceil(r)

def byte2tensor(image):
    decoded_image = cv2.imdecode(np.frombuffer(image, dtype=np.uint8),
                                 cv2.IMREAD_COLOR)
    return numpy_image_to_torch(decoded_image)