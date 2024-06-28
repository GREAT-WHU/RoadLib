import geoFunc.trans as trans
import matplotlib.pyplot as plt
import numpy as np
import re
import math
import matplotlib
from matplotlib.colors import ListedColormap,LinearSegmentedColormap
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import quaternion


all_data={}

first_sod = 0
end_sod = 300000

fp = open(r'/home/zhouyuxuan/data/test0/gt.txt','rt')
while True:
    line = fp.readline().strip()
    if line == '':
        break
    if line[0]=='%' or line[0]=='#' :
        continue
    line = re.sub('\s\s+',' ',line)
    elem = line.split(' ')
    sod= float(elem[0])
    if sod< first_sod or sod>end_sod : continue
    if sod not in all_data.keys():
        all_data[sod]={}
    all_data[sod]['X0']=float(elem[1])
    all_data[sod]['Y0']=float(elem[2])
    all_data[sod]['Z0']=float(elem[3])
    qw = float(elem[4])
    qx = float(elem[5])
    qy = float(elem[6])
    qz = float(elem[7])
    R = quaternion.as_rotation_matrix(np.quaternion(qw,qx,qy,qz))
    att = np.array(trans.m2att(R))/math.pi*180 
    all_data[sod]['ATTX0'] = float(att[0])
    all_data[sod]['ATTY0'] = float(att[1])
    all_data[sod]['ATTZ0'] = float(att[2])
fp.close()

result_name = r"/home/zhouyuxuan/roadlib/build/result.txt"
fp = open(result_name,'rt')

while True:
    line = fp.readline().strip()
    if line == '':
        break
    if line[0]=='%' or line[0]=='#' :
        continue
    line = re.sub('\s\s+',' ',line)
    elem = line.split(' ')
    sod= float(elem[0])
    if sod< first_sod or sod>end_sod : continue
    all_data[sod]['X1']=float(elem[1])
    all_data[sod]['Y1']=float(elem[2])
    all_data[sod]['Z1']=float(elem[3])
    qw = float(elem[4])
    qx = float(elem[5])
    qy = float(elem[6])
    qz = float(elem[7])
    R = quaternion.as_rotation_matrix(np.quaternion(qw,qx,qy,qz))
    att = np.array(trans.m2att(R))/math.pi*180 
    all_data[sod]['ATTX1'] = float(att[0])
    all_data[sod]['ATTY1'] = float(att[1])
    all_data[sod]['ATTZ1'] = float(att[2])


t_series=[]
x_series=[]
y_series=[]
z_series=[]
m_series=[]
ax_series=[]
ay_series=[]
az_series=[]
e_series = []
n_series = []
u_series = []
xyz0 = None

for sod in all_data.keys():
    dd = all_data[sod]
    if 'X1' in dd.keys() and 'X0' in dd.keys():
        xyz1 = np.array([dd['X1'],dd['Y1'],dd['Z1']])
        if xyz0 is None:
            xyz0 = np.copy(xyz1)
        t_series.append(sod)
        dxyz = np.array([dd['X1']-dd['X0'],dd['Y1']-dd['Y0'],dd['Z1']-dd['Z0']])
        Ren = trans.Cen([dd['X1'],dd['Y1'],dd['Z1']])
        denu = np.matmul(Ren.T,dxyz)
        enu = np.matmul(Ren.T,xyz1-xyz0)
        x_series.append(denu[0])
        y_series.append(denu[1])
        z_series.append(denu[2])
        e_series.append(enu[0])
        n_series.append(enu[1])
        u_series.append(enu[2])
        ax_series.append(dd['ATTX1']-dd['ATTX0'])
        ay_series.append(dd['ATTY1']-dd['ATTY0'])
        az_series.append(dd['ATTZ1']-dd['ATTZ0'])


err = np.sqrt(np.power(np.array(x_series),2)+np.power(np.array(y_series),2))
print('>>> RMSE        (Horizontal): ', np.sqrt(np.sum(np.square(err))/len(err)))
print('>>> RMSE        (Vertical  ): ', np.sqrt(np.sum(np.square(np.array(z_series)))/len(err)))
print('>>> Avaiability (<  1m     ):',np.sum(err<1.0)/len(err)*100)
print('>>> Avaiability (<0.5m     ):',np.sum(err<0.5)/len(err)*100)

plt.figure('xyz')
plt.plot(t_series,x_series,c='r',linewidth=0.5)
plt.plot(t_series,y_series,c='g',linewidth=0.5)
plt.plot(t_series,z_series,c='b',linewidth=0.5)
plt.xlabel('Time [s]')
plt.ylabel('Position Error [m]')

plt.figure('bev',figsize=[3,3])
plt.axis('equal')
plt.scatter(e_series,n_series,c=err,cmap='jet',s=3)
plt.grid(linestyle='-.')
plt.colorbar()
plt.clim([0,1])
plt.xlabel('East [m]')
plt.ylabel('North [m]')
plt.tight_layout()
plt.savefig('result.svg')

plt.show()
