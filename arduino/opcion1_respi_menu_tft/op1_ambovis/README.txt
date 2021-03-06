RECOMENDACIONES MIT:

From the Clinical Guidance this document summarizes the minimum set of requirements for ventilation:

Patients must be under the management of a trained clinician.
The minimum controllable parameters in order to ventilate a patient include:

BPM (breaths per minute): between 8 � 40 BPM

Tidal Volume (TV) (air volume pushed into lung): between 200 � 800 mL based on patient weight

I/E Ratio (inspiratory/expiration time ratio): recommended to start around 1:2; best if adjustable between range of 1:1 � 1:4*

Assist Detection pressure. When a patient tries to inspire, they can cause a dip on teh order of 1 � 5 cm H2O, with respect to PEEP pressure (not necessarily = atmospheric).
Airway pressure must be monitored
Maximum pressure should be limited to 40 cm H2O at any time; Plateau pressure should be limited to max 30 cm H2O
The use of a passive mechanical blow-off valve fixed at 40 cm H2O is strongly recommended
Clinician require readings of plateau pressure and PEEP (refer to clinical documentation tab)
PEEP of 5-15 cm H2O required; many patients need 10-15 cmH2O
Failure conditions must permit conversion to manual clinician override, i.e. if automatic ventilation fails, the conversion to immediate ventilation must be immediate.
Ventilation on room air is better than no ventilation at all. Blending of oxygen and air gas mixture to adjust FiO2 is not important in an emergency scenario.  It is certainly nice to have that ability and can easily be implemented with a oxygen / air gas blender that some hospitals already have.
Covid-19 can get aerosolized (airborne), so HEPA filtration on the patient�s exhalation is required or between the ventilator unit and the patient (at the end of the endotracheal tube) to protect clinical staff from certain infection. In-line HEPA filters can usually be purchased alongside manual resuscitator bags.
Heat and moisture exchanger should be used in line with the breathing circuit.
Failure conditions must result in an alarm.

*******************
-----------------
LINKS
-----------------
*******************

https://instrumentacionycontrol.net/que-valores-iniciales-usar-en-un-pid/
TYP
P	0.5 gain
I   0.2 
D   0
Filtro
Muestreo
------------------
MENU CON ENCODER
https://giltesa.com/2017/07/29/menu-submenus-lcd-arduino-encoder
------------------

Bueno de driver y motores
https://www.makerguides.com/tb6600-stepper-motor-driver-arduino-tutorial/

Libreria AccelStepper
https://zaragozamakerspace.com/index.php/lessons/curso-de-arduino-y-robotica-libreria-accelstepper/
https://www.todopic.com.ar/foros/index.php?topic=46143.0

Cmbio d sentido de giro
https://sites.google.com/site/angmuz/home/proyecto-10-arduino-basics-steppers-motors

SENSORES-----------------------
Con honeywell ASDXRRX010ND7A5
https://www.instructables.com/id/Make-Your-Own-Spirometer/
https://trybotics.com/project/Make-Your-Own-Spirometer-80064
https://electronics.stackexchange.com/questions/181181/arduino-interface-with-i2c-pressure-sensor

//CONECTADO EN FORMA ANALOGICA
https://forum.arduino.cc/index.php?topic=614995.0
https://www.instructables.com/id/Low-Cost-Spirometer/


Sensores de presion
https://goughlui.com/2018/08/05/note-bosch-sensortec-bmp280-vs-bme280-sensor-confusion/

http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html

Resolution
https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp280-1.html
LCD 2 con display
http://www.zonnepanelen.wouterlood.com/logging-barometric-pressure-with-an-i2c-device-and-an-arduino-sd-shield/
MUY BUENO DISPLAY
https://diyi0t.com/lcd-display-tutorial-for-arduino-and-esp8266/

//INTERRUPTIONS IN ARDUINO MEGA
https://forum.arduino.cc/index.php?topic=418776.0

--------------------------------
Visualizacion
Con matplotlib
http://arduino-er.blogspot.com/2017/06/python-run-on-raspberry-pi-to-plot.html
http://arduino-er.blogspot.com/2015/04/python-to-plot-graph-of-serial-data.html
http://www.toptechboy.com/tutorial/python-with-arduino-lesson-11-plotting-and-graphing-live-data-from-arduino-with-matplotlib/

Separated with commas
https://www.instructables.com/id/Plotting-and-Graphing-Live-Data-from-Arduino-using/
Serial plotter
https://problemsolvingwithpython.com/11-Python-and-External-Hardware/11.04-Reading-a-Sensor-with-Python/

--------------------------------
Venturi
http://laplace.us.es/wiki/index.php/Tubo_de_Venturi
https://fisica.laguia2000.com/dinamica-clasica/efecto-venturi

--------------------------------
ANDROID OREO EMULATOR
https://liliputing.com/2019/01/android-x86-8-1-released-lets-you-run-oreo-on-desktop-pcs.html

---------------------------------
git config --global credential.helper 'cache --timeout 7200'

---------------------------------
ARDUINO MAPS para buscar
FORO: https://github.com/maniacbug/StandardCplusplus/blob/master/map

https://github.com/maniacbug/StandardCplusplus/blob/master/map
http://andybrown.me.uk/ws/2011/01/15/the-standard-template-library-stl-for-avr-with-c-streams/

tests 
http://www.gammon.com.au/forum/?id=11119

Otro: puede usarse el find_if:
https://poesiabinaria.net/2016/02/como-buscar-en-un-vector-o-una-lista-de-mapas-en-c/


----------------------------------
ONLINE SIMULATORS


Encoder
https://www.electronics-lab.com/project/rotary-encoder-arduino-mega-nokia-5110-display-tutorial/
https://www.reddit.com/r/arduino/comments/5eox8g/best_rotary_encoder_library_on_any_pin/

https://arduino.stackexchange.com/questions/43774/arduino-mega-2560-interrupt-pins-and-port-mapping-with-rotary-encoder

//INSTALL APP
https://stackoverflow.com/questions/7076240/install-an-apk-file-from-command-prompt


mpx5050

- 0 to 50 kPa (0 to 7.25 psi)
- 0.2 to 4.7 Volts Volts
- 2.5% Maximum Error over 0� to 85�
- Ideally suited for Microprocessor or Microcontroller�Based �System.
- Temperature Compensated Over � 40� to + 125�.


BMP 280
Size: 21mm x 18mm
 
1.71V to 3.6V Supply Voltage
 
Max I2C Speed: 3.4Mhz
 
Low power consumption - 2.7uA at 1Hz
 
I2C , SPI(4 Wire) , SPI(3 Wire) interface
 
Very low noise - up to 0.2Pa (1.7cm) and 0.01 temperature
 
Full calibrated
 
Pressure Range: 300hPa to 1100hPa (+9000m to -500m)

30 kpaa

https://stackoverflow.com/questions/18705363/arduino-uno-pwm-pins-conflict
