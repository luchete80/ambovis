#import serial
from serial import Serial
import matplotlib.pyplot as plt
from drawnow import *
import atexit

values = []

plt.ion()
cnt=0

#serialArduino = Serial('/dev/ttyACM0', 9600)
serialArduino = Serial('com8', 115200)

from matplotlib.ticker import ScalarFormatter

def plotValues():

    #ax = plt().xaxis
    #ax.set_major_formatter(ScalarFormatter())

    plt.ylim(0,50) 
    plt.title('Valores AMBOVIS')
    plt.grid(True)
    plt.ylabel('Pressure CMH20')
    plt.plot(values, 'rx-', label='values')
    plt.legend(loc='upper right')
    #plt.ticklabel_format(useOffset=False)  #Force matplotlib to NOT autoscale y axis

def doAtExit():
    serialArduino.close()
    print("Close serial")
    print("serialArduino.isOpen() = " + str(serialArduino.isOpen()))

atexit.register(doAtExit)

print("serialArduino.isOpen() = " + str(serialArduino.isOpen()))

#pre-load dummy data
for i in range(0,10):
    values.append(0)
    
while True:
    while (serialArduino.inWaiting()==0):
        pass
    print("readline()")
    valueRead = serialArduino.readline(500)

    #check if valid value can be casted
    #try:
    valueInInt = valueRead
#        valueInInt = int(valueRead)
	# print(valueInInt)
	# if valueInInt <= 1024:
		# if valueInInt >= 0:
    values.append(valueInInt)
#        values.pop(0)
    drawnow(plotValues)
		# else:
			# print("Invalid! negative number")
	# else:
		# print("Invalid! too large")
    plt.pause(.0001)                     #Pause Briefly. Important to keep drawnow from crashing
    cnt=cnt+1
    if(cnt>10):                            #If you have 50 or more points, delete the first one from the array
	#tempF.pop(0)                       #This allows us to just see the last 50 data points
	    values.pop(0)
		
    # except ValueError:
        # print("Invalid! cannot cast")