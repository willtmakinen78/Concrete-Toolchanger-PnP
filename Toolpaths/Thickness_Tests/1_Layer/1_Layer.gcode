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
G1 X225.0 E0.510291
G1 Y96.0 E0.040823
G1 X275.0 E0.510291
G1 Y92.0 E0.040823
G1 X225.0 E0.510291
G1 Y88.0 E0.040823
G1 X275.0 E0.510291
G1 Y84.0 E0.040823
G1 X225.0 E0.510291
G1 Y80.0 E0.040823
G1 X275.0 E0.510291
G1 Y76.0 E0.040823
G1 X225.0 E0.510291
G1 Y72.0 E0.040823
G1 X275.0 E0.510291
G1 Y68.0 E0.040823
G0 F3000 X283.0 Z8.1
G0 Y108.0

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
