import numpy as np
import matplotlib.pyplot as plt
with open ('data (1).txt',"r") as f:
    lines = f.readlines()[1:]
#f.close()
angle =[round(float(line.split(sep=",")[0]),2) for line in lines]
pidout =[round(float(line.split(sep=",")[1]),2) for line in lines]
speed =[round(float(line.split(sep=",")[2]),2) for line in lines]
fps =[round(float(line.split(sep=",")[3]),4) for line in lines]
elapsed = [round(float(line.split(sep=",")[4]),2) for line in lines]

pidout = [x/10 for x in pidout]
st = elapsed[0]
elapsed=[x-st for x in elapsed]
error=[round(90-a,2) for a in angle]
print(error)


#print(fps)
x = np.linspace(70,110,len(angle))
fig = plt.figure()
fig2 = plt.figure()
#fig2 = plt.figure()
#ay1 = fig2.add_subplot(111)
ax1 = fig.add_subplot(111)
ax2 = ax1.twinx()
bar = fig2.add_subplot(111)


ax1.plot(elapsed,angle, c='r', label='Pendulum Angle',linewidth=2.0)
ax1.set_ylabel('Angle (deg)',color = 'r')
ax1.set_xlabel('Time (s)')
ax1.tick_params('y',colors='r')
ax2.plot(elapsed,pidout, c='g',linewidth=2.0, label='PID Output')
ax1.set_ylim([80,100])
ax2.set_ylabel('PID Output (%)',color = 'g')
ax2.tick_params('y',colors='g')
#ax3.plot(elapsed,fps, c='r', label='FPS')
plt.text(1, -100, 'FPS ~ 64\nKp=13\nKi=1\nKd=8', style='italic',
        bbox={'facecolor':'w', 'alpha':0.5, 'pad':10})

plt.title('Inverted Pendulum \n Computer Vision Control \nHSV Colour Thresholding')

bar.bar(elapsed, error, width=0.8, bottom=None, align='center', data=None)
fig2.show()
plt.show()