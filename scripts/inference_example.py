import torch
import onnxruntime
import os
import cv2
import numpy as np
import time

session = onnxruntime.InferenceSession('./end2end.onnx', 
        providers=[ 'CUDAExecutionProvider', 'CPUExecutionProvider'])

file_list = os.listdir('sample')
print(file_list)

cc = 0
MEAN = np.array([123.675,116.28,103.53])
STD = np.array([58.395,57.12,57.375])

for ff in file_list:
    mm =cv2.imread('sample/'+ff)
    mm = (mm.astype(np.float32) - MEAN)/STD
    mm = mm.transpose(2,0,1)[None]
    mm = mm[:,:,(768-512):768,:]
    mm = mm[:,::-1,:,:]
    inputs = {session.get_inputs()[0].name: mm.astype(np.float32)}
    outs = session.run(None, inputs)[0]
    print(outs.shape)

    # visualizataion
    color_map = np.array([
        [0, 0, 0],      #0
        [0, 255, 255],  #1
        [128, 128, 0],  #2
        [128, 0, 128],  #3
        [0, 128, 0],    #4
        [0, 0, 255]     #5
    ], dtype=np.uint8)
    mm_vis = color_map[outs[0,0]]
    cv2.imshow('mm_vis',mm_vis)
    cv2.waitKey(1)
    cc += 1
    print(time.time())
