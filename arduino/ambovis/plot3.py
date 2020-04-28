# run this in a Jupyter (IPython) Notebook!
# modified from http://www.lebsanft.org/?p=48
# http://pyserial.readthedocs.org/en/latest/pyserial_api.html
import serial
import numpy as np
from matplotlib import pyplot as plt
from time import time

# If you're not using Linux, you'll need to change this
# check the Arduino IDE to see what serial port it's attached to
#ser = serial.Serial('/dev/ttyACM0', 115200)
ser = serial.Serial('com4', 115200)

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
#plt.ylabel('Presion, CMH2O', fontsize='14', fontstyle='italic')
#plt.axes().grid(True)
# line1, = plt.plot(ydata,marker='o',markersize=4,linestyle='dotted',markerfacecolor='green') #ORIGINAL
# line2, = plt.plot(y2data,marker='o',markersize=4,linestyle='dotted',markerfacecolor='blue') #ORIGINAL
# line3, = plt.plot(y3data,marker='o',markersize=4,linestyle='dotted',markerfacecolor='red') #ORIGINAL

line1, = axs[0].plot(ydata,marker='o',markersize=4,linestyle='dotted',markerfacecolor='green') #ORIGINAL
line2, = axs[1].plot(y2data,marker='o',markersize=4,linestyle='dotted',markerfacecolor='blue') #ORIGINAL
line3, = axs[2].plot(y3data,marker='o',markersize=4,linestyle='dotted',markerfacecolor='red') #ORIGINAL


#subplot limits 
# plt.ylim([-500,500]) #ORIGINAL
# plt.xlim([0,view_time]) #ORIGINAL

axs.flat[0].set_xlim(0,view_time)
axs.flat[0].set_ylim(0,40)

axs.flat[1].set_xlim(0,view_time)
axs.flat[1].set_ylim(0,600)

axs.flat[2].set_xlim(0,view_time)
axs.flat[2].set_ylim(0,800)

axs.flat[0].legend([line1],["BMP280", "A0", "Honeywell (A1)"])
axs.flat[1].legend([line2],["Flux [ml/s]"])
axs.flat[2].legend([line3],["Volume [ml]"])


#for (m), subplot in numpy.ndenumerate(axs):
# subplot.set_xlim([0,view_time])
# subplot.set_ylim(yrange)
# subplot.set_xlim([0,view_time])
# subplot.set_ylim(500)

# flush any junk left in the serial buffer
ser.flushInput()
# ser.reset_input_buffer() # for pyserial 3.0+
run = True

# collect the data and plot a moving frame
while run:
    ser.reset_input_buffer()
    data = ser.readline().split(b' ')
    
    # sometimes the incoming data is garbage, so just 'try' to do this
    try:
        # store the entire dataset for later
        #ydata.append(float(data[0])*5.0/1024
        ydata.append(float(data[0]))
        y2data.append(float(data[1]))
        y3data.append(float(data[2]))
        
        timepoints.append(time()-start_time)
        current_time = timepoints[-1]
        
        # update the plotted data
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
    
    # if the try statement throws an error, just do nothing
    except: pass
    
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
axs[0].plot(timepoints, ydata,marker='o',markersize=4,linestyle='none',markerfacecolor='red')
axs[1].plot(timepoints, -ydata)
#axs[0].ylim(yrange)

fig2.show()

ser.close()