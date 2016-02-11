import numpy as np
import pylab as plt

FILE_NAME='periodic_task.log'
COLS=3

raw_data = np.fromfile(FILE_NAME, sep=" ")
rows=raw_data.size/COLS
data = np.reshape(raw_data[:rows*COLS], (rows,COLS))

plt.close('all')
f, axarr = plt.subplots(1, sharex=True, sharey=False)
axarr.set_title('Time spent in periodic task')
axarr.plot(data[1:,0]-data[:-1,0], label='cycle duration')
axarr.plot(data[:-1,1], label='sleep duration')
axarr.set_ylabel("time [ns]");
axarr.legend()

print 'min cycle duration', min(data[1:,0]-data[:-1,0])
print 'max cycle duration', max(data[1:,0]-data[:-1,0])
#axarr[1].set_title('Overruns')
#axarr[1].plot(data[:,2])
#axarr[1].set_ylabel("overruns");

plt.show()
