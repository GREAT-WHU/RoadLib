import struct
from tkinter import font
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from matplotlib.colors import ListedColormap,LinearSegmentedColormap
import mpl_toolkits

matplotlib.rcParams['mathtext.fontset'] = 'custom'
matplotlib.rcParams['mathtext.rm'] = 'Times New Roman'
matplotlib.rcParams['mathtext.it'] = 'Times New Roman:italic'
matplotlib.rcParams['mathtext.bf'] = 'Times New Roman:bold'
matplotlib.rcParams['font.family'] = 'Arial'

font1={'family':'Arial',
     'style':'normal',
    'weight':'bold',
      'color':'black',
      'size':7
}

font2={'family':'Arial',
     'style':'normal',
    'weight':'bold',
      'color':'black',
      'size':11
}

class Patch:
    def __init__(self):
        self.next_id = 0
        self.id = 0
        self.road_class = 0
        self.line_valid = False
        self.frozen = False
        self.merged = False
        self.valid_add_to_map = False
        self.mean_metric = []
        self.cov_metric = []
        self.direction_metric = []
        self.eigen_value_metric = []
        self.line_points=[]
        self.mean_uncertainty=[]
        self.line_points_uncertainty=[]
        self.percept_distance=0.0
        self.b_points=[[],[],[],[]]
        self.b_unc=[0.0,0.0,0.0,0.0]
all_patches={}

# fp = open(r'D:\Projects\road_vision_iros\build\b_map_geo.bin','rb')
fp = open(r'./build/map.bin','rb')
# fp = open(r'D:\Projects\road_vision_iros\build\map_merged.bin','rb')
buff = fp.read()
fp.close()

offset = 0

pos_ref = struct.unpack_from('ddd',buff,offset); offset+=8*3
count = struct.unpack_from('i',buff,offset)[0]; offset+=8*1
print(count)
for iclass in range(count):
    road_class = struct.unpack_from('i',buff,offset)[0]; offset+=4*1
    pcount = struct.unpack_from('i',buff,offset)[0]; offset+=8*1
    print(iclass,pcount)
    all_patches[road_class]=[]
    for ipatch in range(pcount):
        patch = Patch()
        patch.next_id = struct.unpack_from('q',buff,offset); offset+=8*1
        patch.id = struct.unpack_from('q',buff,offset); offset+=8*1
        patch.road_class = struct.unpack_from('i',buff,offset); offset+=4*1
        patch.line_valid = struct.unpack_from('?',buff,offset); offset+=1*1
        patch.frozen = struct.unpack_from('?',buff,offset); offset+=1*1
        patch.merged = struct.unpack_from('?',buff,offset); offset+=1*1
        patch.valid_add_to_map = struct.unpack_from('?',buff,offset); offset+=1*1
        patch.mean_metric = struct.unpack_from('fff',buff,offset); offset+=4*3
        patch.b_points[0] = struct.unpack_from('fff',buff,offset); offset+=4*3
        patch.b_points[1] = struct.unpack_from('fff',buff,offset); offset+=4*3
        patch.b_points[2] = struct.unpack_from('fff',buff,offset); offset+=4*3
        patch.b_points[3] = struct.unpack_from('fff',buff,offset); offset+=4*3
        patch.b_unc[0] = struct.unpack_from('f',buff,offset); offset+=4
        patch.b_unc[1] = struct.unpack_from('f',buff,offset); offset+=4
        patch.b_unc[2] = struct.unpack_from('f',buff,offset); offset+=4
        patch.b_unc[3] = struct.unpack_from('f',buff,offset); offset+=4
        lpcount = struct.unpack_from('i',buff,offset)[0]; offset+=8*1
        for ilp in range(lpcount):
            lp = struct.unpack_from('fff',buff,offset); offset+=4*3
            patch.line_points.append(lp)
        patch.mean_uncertainty = struct.unpack_from('fffffffff',buff,offset); offset+=4*9
        lpcount = struct.unpack_from('i',buff,offset)[0]; offset+=8*1
        print(lpcount)
        for ilp in range(lpcount):
            lp_uncertainty = struct.unpack_from('fffffffff',buff,offset); offset+=4*9
            patch.line_points_uncertainty.append(lp_uncertainty)
        patch.percept_distance = struct.unpack_from('d',buff,offset); offset+=8*1
        all_patches[road_class].append(patch)


plt.figure('map',figsize=[75/25.4*2,72/25.4*2])
def draw_all(all_patch):
    for p in all_patches[1]:
        #plt.scatter(p.mean_metric[0],p.mean_metric[1],s=0.3,c='black')
        p0 = p.b_points[0]
        p1 = p.b_points[1]
        p2 = p.b_points[2]
        p3 = p.b_points[3]
        plt.plot([p0[0],p1[0],p2[0],p3[0],p0[0]],[p0[1],p1[1],p2[1],p3[1],p0[1]],linewidth=0.3,c='orange')

    for p in all_patches[2]:
        #plt.scatter(p.mean_metric[0],p.mean_metric[1],s=0.3,c='black')
        p0 = p.b_points[0]
        p1 = p.b_points[1]
        p2 = p.b_points[2]
        p3 = p.b_points[3]
        plt.plot([p0[0],p1[0],p2[0],p3[0],p0[0]],[p0[1],p1[1],p2[1],p3[1],p0[1]],linewidth=0.3,c='blue')

    for p in all_patches[0]:
        #plt.scatter(p.mean_metric[0],p.mean_metric[1],s=0.3,c='black')
        x_series=[]
        y_series=[]
        for i in range(len(p.line_points)):
            x_series.append(p.line_points[i][0])
            y_series.append(p.line_points[i][1])
        plt.plot(x_series,y_series,linewidth=0.3,c='green')

    for p in all_patches[4]:
        #plt.scatter(p.mean_metric[0],p.mean_metric[1],s=0.3,c='black')
        x_series=[]
        y_series=[]
        for i in range(len(p.line_points)):
            x_series.append(p.line_points[i][0])
            y_series.append(p.line_points[i][1])
        plt.plot(x_series,y_series,linewidth=0.3,c='red')

draw_all(all_patches)
plt.grid(linestyle='-.',linewidth = 0.3)    
plt.tick_params(labelsize=7,direction='in')
plt.xlim([-400,800])
plt.ylim([-1400,-200])
plt.xlabel('East [m]',fontdict=font1)
plt.ylabel('North [m]',fontdict=font1)

plt.tight_layout()
plt.show()