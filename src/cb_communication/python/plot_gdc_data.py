import numpy as np
import pylab as plt

FILE_NAME='log_file.dat'
COLS=23

raw_data = np.fromfile(FILE_NAME, sep=" ")
rows=raw_data.size/COLS
data = np.reshape(raw_data[:rows*COLS], (rows,COLS))

plt.close('all')
f, axarr = plt.subplots(2, sharex=True, sharey=False)
#axarr[0].set_title('Time Difference between consecutive Reads in Main Thread')
axarr[0].plot(data[1:,0]-data[:-1,0], label='loop')
axarr[0].plot(data[1:,1], label='write')
axarr[0].plot(data[0:-1,2], label='read')
axarr[0].plot(data[1:,3], label='log')
axarr[0].plot(data[1:,1]+data[0:-1,2]+data[1:,3], label='wrl')
#axarr[0].axhline(y=0.002, color='r')
#axarr[0].axhline(y=0.000, color='r')
#axarr[0].set_ylim([-0.0005, 0.0025])
axarr[0].set_ylabel("time [ns]");
axarr[0].legend()

#axarr[1].set_title('Time Difference between consecutive Reads in Main Thread')
n_msgs = np.zeros((data.shape[0]-1,1))
axarr[1].plot(n_msgs, label='number of messages read')
#axarr[1].axhline(y=0.002, color='r')
#axarr[1].axhline(y=0.000, color='r')
#axarr[1].set_ylim([-0.0005, 0.0025])
axarr[1].set_ylabel("time [ns]");
axarr[1].legend()

plt.show()
