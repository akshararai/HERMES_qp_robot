import numpy as np
import pylab as plt

FILE_NAME='imu_stream.log'
COLS=8

raw_data = np.fromfile(FILE_NAME, sep=" ")
rows=raw_data.size/COLS
data = np.reshape(raw_data[:rows*COLS], (rows,COLS))

plt.close('all')
f, axarr = plt.subplots(3, sharex=True, sharey=False)
axarr[0].set_title('Time Difference between consecutive Reads in Main Thread')
axarr[0].plot(data[1:,0]-data[:-1,0])
axarr[0].axhline(y=0.002, color='r')
axarr[0].axhline(y=0.000, color='r')
axarr[0].set_ylim([-0.0005, 0.0025])
axarr[0].set_ylabel("time [s]");

axarr[1].set_title('Time Difference between consecutive IMU Messages')
axarr[1].plot(data[1:,7]-data[:-1,7])
axarr[1].axhline(y=0.002, color='r')
axarr[1].axhline(y=0.000, color='r')
axarr[1].set_ylim([-0.0005, 0.0025])
axarr[1].set_ylabel("time [s]");

axarr[2].set_title('Difference between Main Thread Time and IMU Time')
axarr[2].plot(data[:,0]-data[:,7])
axarr[2].set_xlabel("time [ms]");
axarr[2].set_ylabel("time [s]");

plt.show()
