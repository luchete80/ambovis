# run this in a Jupyter (IPython) Notebook!
# modified from http://www.lebsanft.org/?p=48
# http://pyserial.readthedocs.org/en/latest/pyserial_api.html
import serial
import numpy as np
from matplotlib import pyplot as plt
from time import time

# From https://github.com/pyserial/pyserial/issues/216
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

# If you're not using Linux, you'll need to change this
# check the Arduino IDE to see what serial port it's attached to
s = serial.Serial('com4', 115200)
#s = serial.Serial('com4', 115200)
serial = ReadLine(s)

# set plot to animated
plt.ion()

start_time = time()
timepoints = []
ydata = []
y2data = []
y3data = []
#yrange = [-0.1,5.1]
yrange = [0.,500.]
view_time = 4 # seconds of data to view at once
duration = 24 # total seconds to collect data

#fig1 = plt.figure()
fig1,axs = plt.subplots(3)

# http://matplotlib.org/users/text_props.html
# fig1.suptitle('live updated data', fontsize='18', fontweight='bold')
# plt.xlabel('time, seconds', fontsize='14', fontstyle='italic')
axs[0].set(xlabel='time', ylabel='Pressure [CMH2O]')
axs[1].set(xlabel='time', ylabel='Flux     [ml/s]')
axs[2].set(xlabel='time', ylabel='Volume   [ml]')

line1, = axs[0].plot(ydata,marker='o',markersize=4,linestyle='dotted',markerfacecolor='green') #ORIGINAL
line2, = axs[1].plot(y2data,marker='o',markersize=4,linestyle='dotted',markerfacecolor='blue') #ORIGINAL
line3, = axs[2].plot(y3data,marker='o',markersize=4,linestyle='dotted',markerfacecolor='red') #ORIGINAL


#subplot limits
# plt.ylim([-500,500]) #ORIGINAL
# plt.xlim([0,view_time]) #ORIGINAL

axs.flat[0].set_xlim(0,view_time)
axs.flat[0].set_ylim(-40,40)

axs.flat[1].set_xlim(0,view_time)
axs.flat[1].set_ylim(-1500,1500)

axs.flat[2].set_xlim(0,view_time)
axs.flat[2].set_ylim(-1000,1000)

axs.flat[0].legend([line1],["BMP280", "A0", "Honeywell (A1)"])
axs.flat[1].legend([line2],["Flux [ml/s]"])
axs.flat[2].legend([line3],["Volume [ml]"])


#for (m), subplot in numpy.ndenumerate(axs):
# subplot.set_xlim([0,view_time])
# subplot.set_ylim(yrange)
# subplot.set_xlim([0,view_time])
# subplot.set_ylim(500)

# ser.reset_input_buffer() # for pyserial 3.0+
run = True
last_plot = time()

# collect the data and plot a moving frame
while run:
    # Get the data from the serial device
    data = serial.readline().split(b' ')
    t = time()

    # sometimes the incoming data is garbage, so just 'try' to do this
    try:
        # store the entire dataset for later
        #ydata.append(float(data[0])*5.0/1024
        ydata.append(float(data[0]))
        y2data.append(float(data[1]))
        y3data.append(float(data[2]))
        timepoints.append(t - start_time)

    # if the try statement throws an error, just do nothing
    except:
        pass

    # update the plotted data 50 times per second
    if t - last_plot > 0.02:

        current_time = timepoints[-1]

        line1.set_xdata(timepoints)
        line1.set_ydata(ydata)

        line2.set_xdata(timepoints)
        line2.set_ydata(y2data)

        line3.set_xdata(timepoints)
        line3.set_ydata(y3data)

        # slide the viewing frame along
        if current_time > view_time:
            #plt.xlim([current_time-view_time,current_time])
            axs.flat[0].set_xlim(current_time-view_time,current_time)
            axs.flat[1].set_xlim(current_time-view_time,current_time)
            axs.flat[2].set_xlim(current_time-view_time,current_time)

        # when time's up, kill the collect+plot loop
        #if timepoints[-1] > duration: run=False

        last_plot = time()

    # update the plot
    fig1.canvas.draw()

# plot all of the data you collected

#fig2 = plt.figure() #ORIGINAL
#fig2,axs = plt.subplots(2)
# http://matplotlib.org/users/text_props.html
#fig2.suptitle('complete data trace', fontsize='18', fontweight='bold')
#plt.xlabel('time, seconds', fontsize='14', fontstyle='italic')
#plt.ylabel('potential, volts', fontsize='14', fontstyle='italic')
#plt.axes().grid(True)

#plt.plot(timepoints, ydata,marker='o',markersize=4,linestyle='none',markerfacecolor='red')
#plt.ylim(yrange)
#axs[0].plot(timepoints, ydata,marker='o',markersize=4,linestyle='none',markerfacecolor='red')
#axs[1].plot(timepoints, -ydata)
#axs[0].ylim(yrange)

#fig2.show()

s.close()
