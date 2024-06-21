from mmseg.apis import MMSegInferencer
import os

inferencer = MMSegInferencer(model='/home/zhouyuxuan/mmsegmentation/configs/segformer/segformer_whu.py',\
                             weights='/home/zhouyuxuan/mmsegmentation/work_dirs/segformer_whu/iter_160000.pth',\
                             device='cuda')

dirname = '/home/zhouyuxuan/data/2023_04/city_0412/cam0/281000'
filelist = sorted(os.listdir(dirname))
imgpaths = []
cc = 0
fp = open('stamp.txt','wt')
ss = ''
for ff in filelist:
    imgpaths.append(os.path.join(dirname,ff))
    cc += 1
    ss += ff+'\n'
    if cc % 10 == 0:
        inferencer(imgpaths[-10:], show=True,wait_time=0.001,out_dir='out',img_out_dir='')
        fp.writelines(ss)
        fp.flush()
        ss = ''