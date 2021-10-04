EESchema Schematic File Version 4
EELAYER 30 0
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
Wire Wire Line
	2250 2600 2250 2700
Wire Wire Line
	2150 2600 2150 2700
Wire Wire Line
	2150 2700 2250 2700
Wire Wire Line
	2350 2600 2350 2650
$Comp
L Device:C_Small C1
U 1 1 5CB7777C
P 3000 2950
F 0 "C1" V 2950 3000 50  0000 L CNN
F 1 "0.1uF" V 2950 2700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3000 2950 50  0001 C CNN
F 3 "~" H 3000 2950 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 3000 2950 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 3000 2950 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 3000 2950 50  0001 C CNN "Digikey_Number"
	1    3000 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5CB834D3
P 3800 900
F 0 "C9" V 3850 950 50  0000 L CNN
F 1 "10uF" V 3850 650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3800 900 50  0001 C CNN
F 3 "~" H 3800 900 50  0001 C CNN
F 4 "GRM188R61A106KE69D" H 3800 900 50  0001 C CNN "Manu_Number"
F 5 "Murata Electronics North America" H 3800 900 50  0001 C CNN "Manu_Name"
F 6 "490-10474-1-ND" H 3800 900 50  0001 C CNN "Digikey_Number"
	1    3800 900 
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5CB83BFF
P 3200 2950
F 0 "C3" V 3150 3000 50  0000 L CNN
F 1 "0.1uF" V 3150 2700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3200 2950 50  0001 C CNN
F 3 "~" H 3200 2950 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 3200 2950 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 3200 2950 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 3200 2950 50  0001 C CNN "Digikey_Number"
	1    3200 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 1050 2900 1050
Wire Wire Line
	3900 1050 3950 1050
Text Label 1100 2100 0    43   ~ 0
GNDD
Text Label 4000 950  0    43   ~ 0
GNDD
Text Label 3000 950  0    43   ~ 0
GNDA
Text Label 1100 2400 0    43   ~ 0
GNDA
Text Label 2250 2800 3    43   ~ 0
GNDA
Text Label 2350 2650 3    43   ~ 0
GNDD
Text Label 1100 2300 0    50   ~ 0
VddA
Text Label 1100 2200 0    50   ~ 0
VddD
Text Label 3500 1050 2    50   ~ 0
VddD
Text Label 2500 1050 2    50   ~ 0
VddA
Wire Wire Line
	2500 1050 2650 1050
$Comp
L Device:R R1
U 1 1 5CB8886F
P 4100 1850
F 0 "R1" V 4000 1850 50  0000 C CNN
F 1 "100" V 4100 1850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4030 1850 50  0001 C CNN
F 3 "~" H 4100 1850 50  0001 C CNN
F 4 "ESR03EZPJ102" V 4100 1850 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 4100 1850 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 4100 1850 50  0001 C CNN "Digikey_Number"
	1    4100 1850
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5CB8B362
P 2800 900
F 0 "C8" V 2850 950 50  0000 L CNN
F 1 "10uF" V 2850 650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2800 900 50  0001 C CNN
F 3 "~" H 2800 900 50  0001 C CNN
F 4 "GRM188R61A106KE69D" H 2800 900 50  0001 C CNN "Manu_Number"
F 5 "Murata Electronics North America" H 2800 900 50  0001 C CNN "Manu_Name"
F 6 "490-10474-1-ND" H 2800 900 50  0001 C CNN "Digikey_Number"
	1    2800 900 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2650 900  2700 900 
Connection ~ 2650 1050
Wire Wire Line
	2700 1050 2650 1050
Wire Wire Line
	2650 1050 2650 900 
Wire Wire Line
	2950 1050 2950 950 
Wire Wire Line
	2950 900  2900 900 
Wire Wire Line
	3000 950  2950 950 
Connection ~ 2950 950 
Wire Wire Line
	2950 950  2950 900 
Wire Wire Line
	3950 1050 3950 950 
Wire Wire Line
	3950 900  3900 900 
Wire Wire Line
	4000 950  3950 950 
Connection ~ 3950 950 
Wire Wire Line
	3950 950  3950 900 
Wire Wire Line
	3600 900  3700 900 
Wire Wire Line
	3600 1050 3700 1050
Connection ~ 3600 1050
Wire Wire Line
	3600 1050 3600 900 
Wire Wire Line
	3500 1050 3600 1050
Text Label 2850 1550 0    50   ~ 0
REFIN-
Text Label 2850 1450 0    50   ~ 0
REFIN+
Wire Wire Line
	1100 1600 1000 1600
Wire Wire Line
	1100 1500 1000 1500
Text Label 1100 1900 0    50   ~ 0
~IRQ
Text Label 1100 1500 0    50   ~ 0
~CS
Text Label 1100 1600 0    50   ~ 0
SCK
Text Label 1100 1800 0    50   ~ 0
SDO
Text Label 1100 1700 0    50   ~ 0
SDI
$Comp
L Connector:Conn_01x10_Male J3
U 1 1 5CBA6185
P 5200 2050
F 0 "J3" H 5172 1976 50  0000 R CNN
F 1 "Conn_01x08_Male" H 5173 2021 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 5200 2050 50  0001 C CNN
F 3 "~" H 5200 2050 50  0001 C CNN
	1    5200 2050
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x10_Male J2
U 1 1 5CBA7EE2
P 800 2000
F 0 "J2" H 772 1926 50  0000 R CNN
F 1 "Conn_01x08_Male" H 773 1971 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 800 2000 50  0001 C CNN
F 3 "~" H 800 2000 50  0001 C CNN
	1    800  2000
	1    0    0    1   
$EndComp
Wire Wire Line
	1100 2100 1000 2100
Wire Wire Line
	1100 2200 1000 2200
Wire Wire Line
	1100 1700 1000 1700
Wire Wire Line
	1100 1800 1000 1800
Wire Wire Line
	1100 1900 1000 1900
Wire Wire Line
	1100 2000 1000 2000
$Comp
L Device:C_Small C5
U 1 1 5CB7C8B4
P 3400 2950
F 0 "C5" V 3350 2800 50  0000 L CNN
F 1 "0.1uF" V 3450 2700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3400 2950 50  0001 C CNN
F 3 "~" H 3400 2950 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 3400 2950 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 3400 2950 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 3400 2950 50  0001 C CNN "Digikey_Number"
	1    3400 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 1450 2850 1450
Wire Wire Line
	2800 1550 2850 1550
$Comp
L Device:C_Small C7
U 1 1 5D97E1AE
P 3600 2950
F 0 "C7" V 3550 2800 50  0000 L CNN
F 1 "0.1uF" V 3650 2700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3600 2950 50  0001 C CNN
F 3 "~" H 3600 2950 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 3600 2950 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 3600 2950 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 3600 2950 50  0001 C CNN "Digikey_Number"
	1    3600 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5D97E1DE
P 3300 2600
F 0 "C4" V 3250 2450 50  0000 L CNN
F 1 "0.1uF" V 3350 2350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3300 2600 50  0001 C CNN
F 3 "~" H 3300 2600 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 3300 2600 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 3300 2600 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 3300 2600 50  0001 C CNN "Digikey_Number"
	1    3300 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C11
U 1 1 5D97E240
P 2800 1050
F 0 "C11" V 2850 850 50  0000 L CNN
F 1 "0.1uF" V 2850 1100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2800 1050 50  0001 C CNN
F 3 "~" H 2800 1050 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 2800 1050 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 2800 1050 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 2800 1050 50  0001 C CNN "Digikey_Number"
	1    2800 1050
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5D97E40B
P 3500 2600
F 0 "C6" V 3450 2450 50  0000 L CNN
F 1 "0.1uF" V 3550 2350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3500 2600 50  0001 C CNN
F 3 "~" H 3500 2600 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 3500 2600 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 3500 2600 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 3500 2600 50  0001 C CNN "Digikey_Number"
	1    3500 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5D97E414
P 3100 2600
F 0 "C2" V 3050 2450 50  0000 L CNN
F 1 "0.1uF" V 3150 2350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3100 2600 50  0001 C CNN
F 3 "~" H 3100 2600 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 3100 2600 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 3100 2600 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 3100 2600 50  0001 C CNN "Digikey_Number"
	1    3100 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C0
U 1 1 5D97E41D
P 2900 2600
F 0 "C0" V 2850 2450 50  0000 L CNN
F 1 "0.1uF" V 2950 2350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2900 2600 50  0001 C CNN
F 3 "~" H 2900 2600 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 2900 2600 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 2900 2600 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 2900 2600 50  0001 C CNN "Digikey_Number"
	1    2900 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C12
U 1 1 5D97E426
P 3800 1050
F 0 "C12" V 3850 850 50  0000 L CNN
F 1 "0.1uF" V 3850 1100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3800 1050 50  0001 C CNN
F 3 "~" H 3800 1050 50  0001 C CNN
F 4 "C0603C104J4RACTU" H 3800 1050 50  0001 C CNN "Manu_Number"
F 5 "KEMET" H 3800 1050 50  0001 C CNN "Manu_Name"
F 6 "399-1097-1-ND" H 3800 1050 50  0001 C CNN "Digikey_Number"
	1    3800 1050
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5D97FCF6
P 4100 2250
F 0 "R5" V 4000 2250 50  0000 C CNN
F 1 "100" V 4100 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4030 2250 50  0001 C CNN
F 3 "~" H 4100 2250 50  0001 C CNN
F 4 "ESR03EZPJ102" V 4100 2250 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 4100 2250 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 4100 2250 50  0001 C CNN "Digikey_Number"
	1    4100 2250
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5D97FDEE
P 3800 1950
F 0 "R2" V 3700 1950 50  0000 C CNN
F 1 "100" V 3800 1950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3730 1950 50  0001 C CNN
F 3 "~" H 3800 1950 50  0001 C CNN
F 4 "ESR03EZPJ102" V 3800 1950 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 3800 1950 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 3800 1950 50  0001 C CNN "Digikey_Number"
	1    3800 1950
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5D97FE52
P 3800 2350
F 0 "R6" V 3700 2350 50  0000 C CNN
F 1 "100" V 3800 2350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3730 2350 50  0001 C CNN
F 3 "~" H 3800 2350 50  0001 C CNN
F 4 "ESR03EZPJ102" V 3800 2350 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 3800 2350 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 3800 2350 50  0001 C CNN "Digikey_Number"
	1    3800 2350
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5D97FEA0
P 4100 2050
F 0 "R3" V 4000 2050 50  0000 C CNN
F 1 "100" V 4100 2050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4030 2050 50  0001 C CNN
F 3 "~" H 4100 2050 50  0001 C CNN
F 4 "ESR03EZPJ102" V 4100 2050 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 4100 2050 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 4100 2050 50  0001 C CNN "Digikey_Number"
	1    4100 2050
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5D97FEE8
P 4100 2450
F 0 "R7" V 4000 2450 50  0000 C CNN
F 1 "100" V 4100 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4030 2450 50  0001 C CNN
F 3 "~" H 4100 2450 50  0001 C CNN
F 4 "ESR03EZPJ102" V 4100 2450 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 4100 2450 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 4100 2450 50  0001 C CNN "Digikey_Number"
	1    4100 2450
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5D97FF3E
P 3800 2150
F 0 "R4" V 3700 2150 50  0000 C CNN
F 1 "100" V 3800 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3730 2150 50  0001 C CNN
F 3 "~" H 3800 2150 50  0001 C CNN
F 4 "ESR03EZPJ102" V 3800 2150 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 3800 2150 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 3800 2150 50  0001 C CNN "Digikey_Number"
	1    3800 2150
	0    1    1    0   
$EndComp
$Comp
L Device:R R0
U 1 1 5D97FF9E
P 3800 1750
F 0 "R0" V 3700 1750 50  0000 C CNN
F 1 "100" V 3800 1750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3730 1750 50  0001 C CNN
F 3 "~" H 3800 1750 50  0001 C CNN
F 4 "ESR03EZPJ102" V 3800 1750 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 3800 1750 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 3800 1750 50  0001 C CNN "Digikey_Number"
	1    3800 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	2800 1750 2900 1750
Wire Wire Line
	2800 1950 3100 1950
Wire Wire Line
	2800 2150 3300 2150
Wire Wire Line
	2800 2350 3500 2350
Wire Wire Line
	2800 1850 3000 1850
Wire Wire Line
	2800 2050 3200 2050
Wire Wire Line
	2800 2250 3400 2250
Wire Wire Line
	2800 2450 3600 2450
Wire Wire Line
	4250 1850 4500 1850
Wire Wire Line
	4250 2050 4500 2050
Wire Wire Line
	4250 2250 4500 2250
Wire Wire Line
	4250 2450 4500 2450
Wire Wire Line
	3950 1750 4500 1750
Wire Wire Line
	3950 1950 4500 1950
Wire Wire Line
	3950 2150 4500 2150
Wire Wire Line
	3950 2350 4500 2350
Wire Wire Line
	2900 2700 2900 3100
Wire Wire Line
	2900 3100 3000 3100
Wire Wire Line
	3600 3100 3600 3050
Wire Wire Line
	3400 3050 3400 3100
Connection ~ 3400 3100
Wire Wire Line
	3400 3100 3500 3100
Wire Wire Line
	3200 3050 3200 3100
Connection ~ 3200 3100
Wire Wire Line
	3200 3100 3300 3100
Wire Wire Line
	3000 3050 3000 3100
Connection ~ 3000 3100
Wire Wire Line
	3000 3100 3100 3100
Wire Wire Line
	3100 2700 3100 3100
Connection ~ 3100 3100
Wire Wire Line
	3100 3100 3200 3100
Wire Wire Line
	3300 2700 3300 3100
Connection ~ 3300 3100
Wire Wire Line
	3300 3100 3400 3100
Wire Wire Line
	3500 2700 3500 3100
Connection ~ 3500 3100
Wire Wire Line
	3500 3100 3600 3100
Wire Wire Line
	2250 3100 2900 3100
Connection ~ 2250 2700
Connection ~ 2900 3100
Wire Wire Line
	2250 2700 2250 3100
Text Label 1700 1750 2    50   ~ 0
~IRQ
Text Label 1700 2050 2    50   ~ 0
~CS
Text Label 1700 2150 2    50   ~ 0
SCK
Text Label 1700 2250 2    50   ~ 0
SDO
Text Label 1700 2350 2    50   ~ 0
SDI
Wire Wire Line
	1750 2050 1700 2050
Wire Wire Line
	1750 2150 1700 2150
Wire Wire Line
	1750 2250 1700 2250
Wire Wire Line
	1750 2350 1700 2350
Wire Wire Line
	1700 1750 1750 1750
Wire Wire Line
	1750 1850 1700 1850
Text Label 1700 1850 2    50   ~ 0
MCLK
Text Label 1950 1250 1    50   ~ 0
VddD
Text Label 2050 1250 1    50   ~ 0
VddA
Wire Wire Line
	1950 1250 1950 1300
Wire Wire Line
	2050 1300 2050 1250
Wire Wire Line
	2900 2500 2900 1750
Connection ~ 2900 1750
Wire Wire Line
	2900 1750 3650 1750
Wire Wire Line
	3000 2850 3000 1850
Connection ~ 3000 1850
Wire Wire Line
	3000 1850 3950 1850
Wire Wire Line
	3100 2500 3100 1950
Connection ~ 3100 1950
Wire Wire Line
	3100 1950 3650 1950
Wire Wire Line
	3200 2050 3200 2850
Connection ~ 3200 2050
Wire Wire Line
	3200 2050 3950 2050
Wire Wire Line
	3300 2500 3300 2150
Connection ~ 3300 2150
Wire Wire Line
	3300 2150 3650 2150
Wire Wire Line
	3400 2250 3400 2850
Connection ~ 3400 2250
Wire Wire Line
	3400 2250 3950 2250
Wire Wire Line
	3500 2500 3500 2350
Connection ~ 3500 2350
Wire Wire Line
	3500 2350 3650 2350
Wire Wire Line
	3600 2850 3600 2450
Connection ~ 3600 2450
Wire Wire Line
	3600 2450 3950 2450
Text Label 3500 1550 2    50   ~ 0
REFIN-
Text Label 3500 1650 2    50   ~ 0
REFIN+
Text Label 4500 2450 0    50   ~ 0
CH7
Text Label 4500 2350 0    50   ~ 0
CH6
Text Label 4500 2250 0    50   ~ 0
CH5
Text Label 4500 2150 0    50   ~ 0
CH4
Text Label 4500 2050 0    50   ~ 0
CH3
Text Label 4500 1950 0    50   ~ 0
CH2
Text Label 4500 1850 0    50   ~ 0
CH1
Text Label 4500 1750 0    50   ~ 0
CH0
$Comp
L Device:C_Small C13
U 1 1 5D9DF406
P 4300 2800
F 0 "C13" V 4400 2750 50  0000 L CNN
F 1 "10uF" V 4350 2550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4300 2800 50  0001 C CNN
F 3 "~" H 4300 2800 50  0001 C CNN
F 4 "GRM188R61A106KE69D" H 4300 2800 50  0001 C CNN "Manu_Number"
F 5 "Murata Electronics North America" H 4300 2800 50  0001 C CNN "Manu_Name"
F 6 "490-10474-1-ND" H 4300 2800 50  0001 C CNN "Digikey_Number"
	1    4300 2800
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C14
U 1 1 5D9DF4F4
P 4300 2950
F 0 "C14" V 4400 2850 50  0000 L CNN
F 1 "10uF" V 4350 3000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4300 2950 50  0001 C CNN
F 3 "~" H 4300 2950 50  0001 C CNN
F 4 "GRM188R61A106KE69D" H 4300 2950 50  0001 C CNN "Manu_Number"
F 5 "Murata Electronics North America" H 4300 2950 50  0001 C CNN "Manu_Name"
F 6 "490-10474-1-ND" H 4300 2950 50  0001 C CNN "Digikey_Number"
	1    4300 2950
	0    1    1    0   
$EndComp
Text Label 4100 2950 2    50   ~ 0
REFIN+
Text Label 4100 2800 2    50   ~ 0
REFIN-
Wire Wire Line
	4450 2950 4450 2800
Wire Wire Line
	4450 2800 4400 2800
Text Label 4500 2950 0    43   ~ 0
GNDA
Wire Wire Line
	4400 2950 4450 2950
Connection ~ 4450 2950
Wire Wire Line
	4450 2950 4500 2950
Wire Wire Line
	1000 2300 1100 2300
Wire Wire Line
	1000 2400 1100 2400
$Comp
L Device:R R8
U 1 1 5DA779BD
P 4100 1650
F 0 "R8" V 4000 1650 50  0000 C CNN
F 1 "100" V 4100 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4030 1650 50  0001 C CNN
F 3 "~" H 4100 1650 50  0001 C CNN
F 4 "ESR03EZPJ102" V 4100 1650 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 4100 1650 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 4100 1650 50  0001 C CNN "Digikey_Number"
	1    4100 1650
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 5DA77C05
P 3800 1550
F 0 "R9" V 3700 1550 50  0000 C CNN
F 1 "100" V 3800 1550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3730 1550 50  0001 C CNN
F 3 "~" H 3800 1550 50  0001 C CNN
F 4 "ESR03EZPJ102" V 3800 1550 50  0001 C CNN "Manu_Number"
F 5 "Rohm Semiconductor" V 3800 1550 50  0001 C CNN "Manu_Name"
F 6 "RHM1.0KDCT-ND" V 3800 1550 50  0001 C CNN "Digikey_Number"
	1    3800 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	3950 1650 3500 1650
Wire Wire Line
	3500 1550 3650 1550
Wire Wire Line
	4100 2800 4200 2800
Wire Wire Line
	4100 2950 4200 2950
Text Label 5000 1750 2    50   ~ 0
CH7
Text Label 5000 1850 2    50   ~ 0
CH6
Text Label 5000 1950 2    50   ~ 0
CH5
Text Label 5000 2050 2    50   ~ 0
CH4
Text Label 5000 2150 2    50   ~ 0
CH3
Text Label 5000 2250 2    50   ~ 0
CH2
Text Label 5000 2350 2    50   ~ 0
CH1
Text Label 5000 2450 2    50   ~ 0
CH0
Wire Wire Line
	4250 1650 5000 1650
Wire Wire Line
	3950 1550 5000 1550
Text Label 1100 2000 0    50   ~ 0
MCLK
$Comp
L MCP3564-Breakout:MCP3964 U1
U 1 1 615B8AF1
P 2300 1900
F 0 "U1" H 2275 2681 50  0000 C CNN
F 1 "MCP3964R" H 2275 2590 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-20-1EP_3x3mm_P0.45mm_EP1.6x1.6mm_ThermalVias" H 2750 1700 50  0001 C CNN
F 3 "" H 2750 1700 50  0001 C CNN
F 4 "150-MCP3564RT-E/NCCT-ND" H 2300 1900 50  0001 C CNN "Digikey_Number"
F 5 "MCP3564RT-E/NC" H 2300 1900 50  0001 C CNN "Manu_Number"
F 6 "Microchip Technology" H 2300 1900 50  0001 C CNN "Manu_Name"
	1    2300 1900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
