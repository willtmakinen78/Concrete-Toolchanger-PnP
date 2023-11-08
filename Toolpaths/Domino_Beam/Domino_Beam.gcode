; start START gcode

T-1

M104 S0
M140 S0

G28
G29 S1

set global.printing = true

; set concrete purging variables
set global.concretePrimeLayer = 1
set global.concretePrimeX = 205
set global.concretePrimeY = 125
set global.concretePrimeZ = 2.7
set global.concretePrimeLength = 70
set global.concretePrimeE = 1.27

T2

M109 S0
M190 S0

;M98 P"prime.g"

M106 S0

G21 ; set units to millimeters
G90 ; use absolute coordinates
M83 ; use relative distances for extrusion

; end START gcode
M98 P"concreteprime.g"
G0 F3000 Z2.7
G1 F500 X275.0 Y100.0 E0.255349
G1 X175.0 E1.020581
G1 Y96.0 E0.040823
G1 X275.0 E1.020581
G1 Y92.0 E0.040823
G1 X175.0 E1.020581
G1 Y88.0 E0.040823
G1 X275.0 E1.020581
G1 Y84.0 E0.040823
G1 X175.0 E1.020581
G1 Y80.0 E0.040823
G1 X275.0 E1.020581
G1 Y76.0 E0.040823
G1 X175.0 E1.020581
G1 Y72.0 E0.040823
G1 X275.0 E1.020581
G1 Y68.0 E0.040823
G0 F3000 Z5.4
G1 F500 Y72.0 E0.040823
G1 X175.0 E1.020581
G1 Y76.0 E0.040823
G1 X275.0 E1.020581
G1 Y80.0 E0.040823
G1 X175.0 E1.020581
G1 Y84.0 E0.040823
G1 X275.0 E1.020581
G1 Y88.0 E0.040823
G1 X175.0 E1.020581
G1 Y92.0 E0.040823
G1 X275.0 E1.020581
G1 Y96.0 E0.040823
G1 X175.0 E1.020581
G1 Y100.0 E0.040823
G1 X275.0 E1.020581
G1 F0.0 E0.0
G0 F6000.0 Z8.1
G0 F3000 Z8.1
G0 X283.0 Z13.5
G0 Y140.0
T1
G0 F3000 X225.0 Y86.0 Z10.8
M98 P"Domino_Wipe.g"
G0 F3000 Z30.8
T2
M98 P"concreteprime.g"
G0 F3000 X275.0 Y125.0
G1 F500 Y100.0 Z10.8 E0.326745
G1 X175.0 E1.020581
G1 Y96.0 E0.040823
G1 X275.0 E1.020581
G1 Y92.0 E0.040823
G1 X242.25 E0.33424
G1 Y88.0 E0.040823
G1 X275.0 E0.33424
G1 Y84.0 E0.040823
G0 F3000 X242.25
G1 F500 Y80.0 E0.040823
G1 X275.0 E0.33424
G1 Y76.0 E0.040823
G1 X175.0 E1.020581
G1 Y72.0 E0.040823
G1 X275.0 E1.020581
G1 Y68.0 E0.040823
G1 F0.0 E0.0
G0 F6000.0 Z13.5
G0 F3000 Z13.5
G0 X171.0
G0 Y80.0
G1 F0.0 E0.0
G0 F6000.0 Z10.8
G1 F5000.0 E0.3
G0 F3000 Z10.8
G0 X175.0
G1 F500 X207.75 E0.33424
G1 Y84.0 E0.040823
G1 X175.0 E0.33424
G1 Y88.0 E0.040823
G1 X207.75 E0.33424
G1 Y92.0 E0.040823
G1 X175.0 E0.33424
G1 Y96.0 E0.040823
G1 F0.0 E0.0
G0 F6000.0 Z13.5
G0 F3000 Z13.5
G0 X171.0
G0 Y104.0
G0 X275.0
G1 F5000.0 E0.3
G0 F3000 Y100.0
G1 F500 X175.0 E1.020581
G1 Y96.0 E0.040823
G1 X275.0 E1.020581
G1 Y92.0 E0.040823
G1 X175.0 E1.020581
G1 Y88.0 E0.040823
G1 X275.0 E1.020581
G1 Y84.0 E0.040823
G1 X175.0 E1.020581
G1 Y80.0 E0.040823
G1 X275.0 E1.020581
G1 Y76.0 E0.040823
G1 X175.0 E1.020581
G1 Y72.0 E0.040823
G1 X275.0 E1.020581
G1 Y68.0 E0.040823
G1 F0.0 E0.0
G0 F6000.0 Z16.2
G0 F3000 Z16.2
G1 F500 Y72.0 E0.040823
G1 X175.0 E1.020581
G1 Y76.0 E0.040823
G1 X275.0 E1.020581
G1 Y80.0 E0.040823
G1 X175.0 E1.020581
G1 Y84.0 E0.040823
G1 X275.0 E1.020581
G1 Y88.0 E0.040823
G1 X175.0 E1.020581
G1 Y92.0 E0.040823
G1 X275.0 E1.020581
G1 Y96.0 E0.040823
G1 X175.0 E1.020581
G1 Y100.0 E0.040823
G1 X275.0 E1.020581
G1 F0.0 E0.0
G0 F6000.0 Z18.9
G0 F3000 Z18.9
G0 X283.0 Z24.3
G0 Y140.0

; start END gcode

;Drop Bed
G91
G1 Z2 F1000
G90

; Drop off the tool
T-1

; Disable Mesh Compensation.
G29 S2

; Park
G1 X-20 Y200 F50000

set global.printing = false
set global.concretePrimeLayer = 1

M0
; end END gcode
