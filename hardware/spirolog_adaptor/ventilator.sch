EESchema Schematic File Version 4
LIBS:ventilator-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Amplifier_Operational:LM324 U1
U 1 1 5E93A91F
P 3150 2150
F 0 "U1" H 3150 2517 50  0000 C CNN
F 1 "LM324" H 3150 2426 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W10.16mm" H 3100 2250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 3200 2350 50  0001 C CNN
	1    3150 2150
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U1
U 2 1 5E93A961
P 5450 3050
F 0 "U1" H 5450 3417 50  0000 C CNN
F 1 "LM324" H 5450 3326 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W10.16mm" H 5400 3150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5500 3250 50  0001 C CNN
	2    5450 3050
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U1
U 3 1 5E93A9D0
P 8350 3150
F 0 "U1" H 8350 3517 50  0000 C CNN
F 1 "LM324" H 8350 3426 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W10.16mm" H 8300 3250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 8400 3350 50  0001 C CNN
	3    8350 3150
	1    0    0    -1  
$EndComp
$Comp
L Reference_Voltage:TL431LP D1
U 1 1 5E93AA71
P 2350 2800
F 0 "D1" V 2396 2731 50  0000 R CNN
F 1 "TL431LP" V 2305 2731 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2350 2650 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tl431.pdf" H 2350 2800 50  0001 C CIN
	1    2350 2800
	0    -1   -1   0   
$EndComp
$Comp
L Diode:1N4148 D2
U 1 1 5E93AB6C
P 6100 3050
F 0 "D2" H 6100 2834 50  0000 C CNN
F 1 "1N4148" H 6100 2925 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 6100 2875 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/1N4148_1N4448.pdf" H 6100 3050 50  0001 C CNN
	1    6100 3050
	-1   0    0    1   
$EndComp
$Comp
L Transistor_BJT:BD237 Q1
U 1 1 5E93AD70
P 4100 2150
F 0 "Q1" H 4292 2196 50  0000 L CNN
F 1 "BD237" H 4292 2105 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-126-3_Vertical" H 4300 2075 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BD/BD233.pdf" H 4100 2150 50  0001 L CNN
	1    4100 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5E93B0FB
P 2600 2050
F 0 "R4" V 2393 2050 50  0000 C CNN
F 1 "10k" V 2484 2050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2530 2050 50  0001 C CNN
F 3 "~" H 2600 2050 50  0001 C CNN
	1    2600 2050
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5E93B372
P 3450 2650
F 0 "R6" V 3657 2650 50  0000 C CNN
F 1 "10k" V 3566 2650 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3380 2650 50  0001 C CNN
F 3 "~" H 3450 2650 50  0001 C CNN
	1    3450 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2850 2250 2750 2250
Wire Wire Line
	2750 2250 2750 2650
Wire Wire Line
	2750 2650 3300 2650
Wire Wire Line
	2350 2700 2350 2500
Wire Wire Line
	2350 2050 2450 2050
Wire Wire Line
	2750 2050 2850 2050
$Comp
L Device:R R2
U 1 1 5E93B5B7
P 2100 2050
F 0 "R2" V 2307 2050 50  0000 C CNN
F 1 "440" V 2216 2050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2030 2050 50  0001 C CNN
F 3 "~" H 2100 2050 50  0001 C CNN
	1    2100 2050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5E93B62D
P 2100 2500
F 0 "R3" V 2307 2500 50  0000 C CNN
F 1 "1k3" V 2216 2500 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2030 2500 50  0001 C CNN
F 3 "~" H 2100 2500 50  0001 C CNN
	1    2100 2500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R7
U 1 1 5E93B863
P 3700 2150
F 0 "R7" V 3493 2150 50  0000 C CNN
F 1 "240" V 3584 2150 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3630 2150 50  0001 C CNN
F 3 "~" H 3700 2150 50  0001 C CNN
	1    3700 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	1850 3000 1850 2800
Wire Wire Line
	1850 2800 2250 2800
Wire Wire Line
	2350 3450 2350 2900
Wire Wire Line
	1850 3300 1850 3350
Wire Wire Line
	1850 3450 2100 3450
Connection ~ 2100 3450
Wire Wire Line
	2100 3450 2350 3450
Wire Wire Line
	2250 2500 2350 2500
Connection ~ 2350 2500
Wire Wire Line
	2350 2500 2350 2050
Wire Wire Line
	1950 2500 1850 2500
Wire Wire Line
	1850 2500 1850 2800
Connection ~ 1850 2800
Wire Wire Line
	2250 2050 2350 2050
Connection ~ 2350 2050
Wire Wire Line
	1950 2050 1800 2050
Wire Wire Line
	3850 2150 3900 2150
Wire Wire Line
	4200 1950 4200 1550
Wire Wire Line
	4200 1550 1800 1550
Wire Wire Line
	1800 1550 1800 2050
Connection ~ 1800 2050
Wire Wire Line
	3450 2150 3550 2150
Wire Wire Line
	4200 2350 4200 2650
Wire Wire Line
	3600 2650 4200 2650
Connection ~ 4200 2650
$Comp
L Device:R R11
U 1 1 5E93FEC1
P 4800 2850
F 0 "R11" V 4593 2850 50  0000 C CNN
F 1 "4k7" V 4684 2850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4730 2850 50  0001 C CNN
F 3 "~" H 4800 2850 50  0001 C CNN
	1    4800 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5E940015
P 4000 3450
F 0 "R8" H 4070 3496 50  0000 L CNN
F 1 "100" H 4070 3405 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3930 3450 50  0001 C CNN
F 3 "~" H 4000 3450 50  0001 C CNN
	1    4000 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 5E94041E
P 4000 3050
F 0 "L1" H 4053 3096 50  0000 L CNN
F 1 "L" H 4053 3005 50  0000 L CNN
F 2 "TerminalBlock_MetzConnect:TerminalBlock_MetzConnect_Type055_RT01502HDWU_1x02_P5.00mm_Horizontal" H 4000 3050 50  0001 C CNN
F 3 "~" H 4000 3050 50  0001 C CNN
	1    4000 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:L L2
U 1 1 5E940D1E
P 4400 3050
F 0 "L2" H 4453 3096 50  0000 L CNN
F 1 "L" H 4453 3005 50  0000 L CNN
F 2 "TerminalBlock_MetzConnect:TerminalBlock_MetzConnect_Type055_RT01502HDWU_1x02_P5.00mm_Horizontal" H 4400 3050 50  0001 C CNN
F 3 "~" H 4400 3050 50  0001 C CNN
	1    4400 3050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R10
U 1 1 5E940D74
P 4400 3450
F 0 "R10" H 4470 3496 50  0000 L CNN
F 1 "15" H 4470 3405 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4330 3450 50  0001 C CNN
F 3 "~" H 4400 3450 50  0001 C CNN
	1    4400 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5E940DC0
P 4200 4000
F 0 "R9" H 4270 4046 50  0000 L CNN
F 1 "27" H 4270 3955 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4130 4000 50  0001 C CNN
F 3 "~" H 4200 4000 50  0001 C CNN
	1    4200 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3700 4400 3600
Wire Wire Line
	4000 3700 4000 3600
Wire Wire Line
	4000 3300 4000 3250
Wire Wire Line
	4400 3300 4400 3250
Wire Wire Line
	4200 2900 4000 2900
Wire Wire Line
	4200 2250 4200 2350
Wire Wire Line
	4400 2900 4200 2900
Connection ~ 4200 2900
Wire Wire Line
	4000 3700 4200 3700
Wire Wire Line
	4200 3850 4200 3800
Connection ~ 4200 3700
Wire Wire Line
	4200 3700 4400 3700
$Comp
L power:GND #PWR03
U 1 1 5E945B50
P 4200 4250
F 0 "#PWR03" H 4200 4000 50  0001 C CNN
F 1 "GND" H 4205 4077 50  0000 C CNN
F 2 "" H 4200 4250 50  0001 C CNN
F 3 "" H 4200 4250 50  0001 C CNN
	1    4200 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4200 4200 4150
$Comp
L Switch:SW_DIP_x01 SW1
U 1 1 5E946ECB
P 3650 3800
F 0 "SW1" H 3650 4067 50  0000 C CNN
F 1 "SW_DIP_x01" H 3650 3976 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 3650 3800 50  0001 C CNN
F 3 "" H 3650 3800 50  0001 C CNN
	1    3650 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5E946F4F
P 3250 4000
F 0 "R5" H 3320 4046 50  0000 L CNN
F 1 "120" H 3320 3955 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3180 4000 50  0001 C CNN
F 3 "~" H 3250 4000 50  0001 C CNN
	1    3250 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 3850 3250 3800
Wire Wire Line
	3250 3800 3350 3800
Wire Wire Line
	3950 3800 4200 3800
Connection ~ 4200 3800
Wire Wire Line
	4200 3800 4200 3700
Wire Wire Line
	3250 4150 3250 4200
Wire Wire Line
	3250 4200 4200 4200
Connection ~ 4200 4200
Connection ~ 4200 2350
Wire Wire Line
	4200 2650 4200 2900
$Comp
L Device:R R12
U 1 1 5E949C08
P 4800 3250
F 0 "R12" V 4593 3250 50  0000 C CNN
F 1 "4k7" V 4684 3250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4730 3250 50  0001 C CNN
F 3 "~" H 4800 3250 50  0001 C CNN
	1    4800 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 2850 5050 2850
Wire Wire Line
	5050 2850 5050 2950
Wire Wire Line
	5050 2950 5150 2950
Wire Wire Line
	5150 3150 5050 3150
Wire Wire Line
	5050 3150 5050 3250
Wire Wire Line
	5050 3250 4950 3250
Wire Wire Line
	4650 3250 4400 3250
Connection ~ 4400 3250
Wire Wire Line
	4400 3250 4400 3200
Wire Wire Line
	4650 2850 3850 2850
Wire Wire Line
	3850 2850 3850 3250
Wire Wire Line
	3850 3250 4000 3250
Connection ~ 4000 3250
Wire Wire Line
	4000 3250 4000 3200
$Comp
L power:GND #PWR02
U 1 1 5E94C5C6
P 2100 3600
F 0 "#PWR02" H 2100 3350 50  0001 C CNN
F 1 "GND" H 2105 3427 50  0000 C CNN
F 2 "" H 2100 3600 50  0001 C CNN
F 3 "" H 2100 3600 50  0001 C CNN
	1    2100 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5E94D2C0
P 5450 2550
F 0 "R13" V 5243 2550 50  0000 C CNN
F 1 "12k" V 5334 2550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5380 2550 50  0001 C CNN
F 3 "~" H 5450 2550 50  0001 C CNN
	1    5450 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 2550 5050 2550
Wire Wire Line
	5050 2550 5050 2850
Connection ~ 5050 2850
$Comp
L Diode:1N4148 D3
U 1 1 5E94E189
P 6500 3050
F 0 "D3" H 6500 2834 50  0000 C CNN
F 1 "1N4148" H 6500 2925 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 6500 2875 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/1N4148_1N4448.pdf" H 6500 3050 50  0001 C CNN
	1    6500 3050
	-1   0    0    1   
$EndComp
$Comp
L Diode:1N4148 D4
U 1 1 5E94E1DD
P 6900 3050
F 0 "D4" H 6900 2834 50  0000 C CNN
F 1 "1N4148" H 6900 2925 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 6900 2875 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/1N4148_1N4448.pdf" H 6900 3050 50  0001 C CNN
	1    6900 3050
	-1   0    0    1   
$EndComp
$Comp
L Diode:1N4148 D5
U 1 1 5E94E231
P 7300 3050
F 0 "D5" H 7300 2834 50  0000 C CNN
F 1 "1N4148" H 7300 2925 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 7300 2875 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/1N4148_1N4448.pdf" H 7300 3050 50  0001 C CNN
	1    7300 3050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R14
U 1 1 5E94E3CA
P 7550 3350
F 0 "R14" H 7480 3304 50  0000 R CNN
F 1 "3k3" H 7480 3395 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7480 3350 50  0001 C CNN
F 3 "~" H 7550 3350 50  0001 C CNN
	1    7550 3350
	-1   0    0    1   
$EndComp
$Comp
L Device:R R15
U 1 1 5E94F48B
P 7800 3050
F 0 "R15" V 8007 3050 50  0000 C CNN
F 1 "1k2" V 7916 3050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7730 3050 50  0001 C CNN
F 3 "~" H 7800 3050 50  0001 C CNN
	1    7800 3050
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C1
U 1 1 5E94F551
P 8400 4050
F 0 "C1" V 8148 4050 50  0000 C CNN
F 1 "47nF" V 8239 4050 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm" H 8438 3900 50  0001 C CNN
F 3 "~" H 8400 4050 50  0001 C CNN
	1    8400 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	8550 3650 8700 3650
Wire Wire Line
	8700 3650 8700 4050
Wire Wire Line
	8700 4050 8550 4050
Connection ~ 8700 3650
Wire Wire Line
	8700 3650 8900 3650
Wire Wire Line
	8250 3650 8200 3650
Wire Wire Line
	7950 3650 7950 3700
Wire Wire Line
	8250 4050 8100 4050
Connection ~ 8100 3650
Wire Wire Line
	8100 3650 7950 3650
Wire Wire Line
	7950 3650 7950 3250
Wire Wire Line
	7950 3250 8050 3250
Connection ~ 7950 3650
Wire Wire Line
	7950 3050 8050 3050
Wire Wire Line
	7550 3200 7550 3050
Wire Wire Line
	7450 3050 7550 3050
Connection ~ 7550 3050
Wire Wire Line
	7550 3050 7650 3050
Wire Wire Line
	7950 4000 7950 4100
Wire Wire Line
	7550 3500 7550 4100
Wire Wire Line
	7550 4100 7800 4100
Connection ~ 7950 4100
Wire Wire Line
	7950 4100 7950 4250
Wire Wire Line
	9150 3550 9150 3500
Wire Wire Line
	9150 3150 8900 3150
Wire Wire Line
	8900 3150 8900 3650
Connection ~ 8900 3150
Wire Wire Line
	8900 3150 8650 3150
$Comp
L power:GND #PWR05
U 1 1 5E966A16
P 9150 4300
F 0 "#PWR05" H 9150 4050 50  0001 C CNN
F 1 "GND" H 9155 4127 50  0000 C CNN
F 2 "" H 9150 4300 50  0001 C CNN
F 3 "" H 9150 4300 50  0001 C CNN
	1    9150 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5E966A66
P 7950 4250
F 0 "#PWR04" H 7950 4000 50  0001 C CNN
F 1 "GND" H 7955 4077 50  0000 C CNN
F 2 "" H 7950 4250 50  0001 C CNN
F 3 "" H 7950 4250 50  0001 C CNN
	1    7950 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 2550 5850 2550
Wire Wire Line
	5850 2550 5850 3050
Wire Wire Line
	5850 3050 5950 3050
Wire Wire Line
	5750 3050 5850 3050
Connection ~ 5850 3050
Wire Wire Line
	6250 3050 6350 3050
Wire Wire Line
	6650 3050 6750 3050
Wire Wire Line
	7050 3050 7150 3050
Wire Wire Line
	1800 2050 1400 2050
$Comp
L power:+12V #PWR01
U 1 1 5E970EFF
P 1400 1950
F 0 "#PWR01" H 1400 1800 50  0001 C CNN
F 1 "+12V" H 1415 2123 50  0000 C CNN
F 2 "" H 1400 1950 50  0001 C CNN
F 3 "" H 1400 1950 50  0001 C CNN
	1    1400 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4200 4200 4250
Wire Wire Line
	1400 1950 1400 2050
Wire Wire Line
	2100 3450 2100 3600
Wire Wire Line
	9150 3850 9150 4300
$Comp
L Connector:Conn_01x02_Female J1
U 1 1 5E9897B8
P 1150 2150
F 0 "J1" H 1044 1825 50  0000 C CNN
F 1 "Conn_01x02_Female" H 1044 1916 50  0000 C CNN
F 2 "TerminalBlock_MetzConnect:TerminalBlock_MetzConnect_Type055_RT01502HDWU_1x02_P5.00mm_Horizontal" H 1150 2150 50  0001 C CNN
F 3 "~" H 1150 2150 50  0001 C CNN
	1    1150 2150
	-1   0    0    1   
$EndComp
Wire Wire Line
	1350 2050 1400 2050
Connection ~ 1400 2050
$Comp
L power:GND #PWR06
U 1 1 5E98B88A
P 1400 2250
F 0 "#PWR06" H 1400 2000 50  0001 C CNN
F 1 "GND" H 1405 2077 50  0000 C CNN
F 2 "" H 1400 2250 50  0001 C CNN
F 3 "" H 1400 2250 50  0001 C CNN
	1    1400 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 2150 1400 2150
Wire Wire Line
	1400 2150 1400 2250
$Comp
L Device:R_POT RV1
U 1 1 5E94D44C
P 1850 3150
F 0 "RV1" H 1780 3104 50  0000 R CNN
F 1 "R_POT" H 1780 3195 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266Y_Vertical" H 1850 3150 50  0001 C CNN
F 3 "~" H 1850 3150 50  0001 C CNN
	1    1850 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	1700 3150 1700 3350
Wire Wire Line
	1700 3350 1850 3350
Connection ~ 1850 3350
Wire Wire Line
	1850 3350 1850 3450
$Comp
L Device:R_POT RV4
U 1 1 5E94FBD1
P 9150 3700
F 0 "RV4" H 9080 3654 50  0000 R CNN
F 1 "R_POT" H 9080 3745 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266Y_Vertical" H 9150 3700 50  0001 C CNN
F 3 "~" H 9150 3700 50  0001 C CNN
	1    9150 3700
	-1   0    0    1   
$EndComp
$Comp
L Device:R_POT RV3
U 1 1 5E94FEF9
P 8400 3650
F 0 "RV3" V 8193 3650 50  0000 C CNN
F 1 "R_POT" V 8284 3650 50  0000 C CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266Y_Vertical" H 8400 3650 50  0001 C CNN
F 3 "~" H 8400 3650 50  0001 C CNN
	1    8400 3650
	0    1    1    0   
$EndComp
$Comp
L Device:R_POT RV2
U 1 1 5E94FFD9
P 7950 3850
F 0 "RV2" H 7880 3804 50  0000 R CNN
F 1 "R_POT" H 7880 3895 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266Y_Vertical" H 7950 3850 50  0001 C CNN
F 3 "~" H 7950 3850 50  0001 C CNN
	1    7950 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	8100 3650 8100 4050
Wire Wire Line
	7800 3850 7800 4100
Connection ~ 7800 4100
Wire Wire Line
	7800 4100 7950 4100
Wire Wire Line
	9000 3700 9000 3500
Wire Wire Line
	9000 3500 9150 3500
Connection ~ 9150 3500
Wire Wire Line
	9150 3500 9150 3150
Wire Wire Line
	8400 3800 8200 3800
Wire Wire Line
	8200 3800 8200 3650
Connection ~ 8200 3650
Wire Wire Line
	8200 3650 8100 3650
$EndSCHEMATC
