# -*- coding:utf-8 -*-
import matplotlib.pyplot as plt

VINS_mono = []
with open("slam_data/time.txt","r") as f:
    for line in f.readlines():
        line = line.strip('\n')
        VINS_mono.append(float(line))

sum=0
for num in VINS_mono:
    sum+=num
ava = sum/len(VINS_mono)
ava = round(ava,3)
print(ava)
title1 = 'average time:'+str(ava)+"ms"+"  "


#vins_mono_line, = plt.plot(VINS_mono,'ro',markersize=2.0)
#plt.legend(handles=[vins_mono_line],

vins_mono_line, = plt.plot(VINS_mono)
plt.legend(handles=[vins_mono_line],

labels=["NDT_OMP_1"],loc='best')
plt.title(title1)
plt.ylabel('ms')
plt.xlabel('frame')
#plt.ylim(0,50)
#plt.xlim(0,1600)
plt.show()

