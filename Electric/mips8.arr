; Pad Frame arrangement file
; David_Harris@hmc.edu
; Generate a padframe for a 64-pin MOSIS tinychip layout

; specify the cell library with the pads
celllibrary muddpads13_ami05.jelib

; create a top-level facet containing the padframe and core
cell chip

; place this facet as the "core"
core mips

; generate schematic and layout views
views lay sch

; set the alignment of the pads (specifying input and output port names)
align pad_corner      dvdd-1 dvdd
align pad_in          dvdd-1 dvdd
align pad_out         dvdd-1 dvdd
align pad_analog	    dvdd-1 dvdd
align pad_dvdd        dvdd-1 dvdd
align pad_dgnd        dvdd-1 dvdd

;; replace the pad_in and pad_out statements with the pad frame arrangement you want.
;; keep at least two pad_dvdd and pad_dgnd pads somewhere on the chip to supply
;; power and ground to the pads and core

; place the top edge of pads, starting with upper-right
place pad_corner
place pad_out dout=memread export pad=memread
place pad_out dout=memwrite export pad=memwrite
place pad_in din=ph1 export pad=ph1
place pad_in din=ph2 export pad=ph2
place pad_in din=memdata[7] export pad=memdata[7]
place pad_in din=memdata[6] export pad=memdata[6]
place pad_in din=memdata[5] export pad=memdata[5]
place pad_in din=memdata[4] export pad=memdata[4]
place pad_in din=memdata[3] export pad=memdata[3]
place pad_in din=memdata[2] export pad=memdata[2]
place pad_dgnd
place pad_dvdd
place pad_dgnd
place pad_dvdd
place pad_dgnd
place pad_dvdd

; place the left edge of pads
rotate cc
place pad_corner
place pad_in din=memdata[1] export pad=memdata[1]
place pad_in din=memdata[0] export pad=memdata[0]
place pad_out dout=writedata[7] export pad=writedata[7]
place pad_out dout=writedata[6] export pad=writedata[6]
place pad_out dout=writedata[5] export pad=writedata[5]
place pad_out dout=writedata[4] export pad=writedata[4]
place pad_out dout=writedata[3] export pad=writedata[3]
place pad_out dout=writedata[2] export pad=writedata[2]
place pad_out dout=writedata[1] export pad=writedata[1]
place pad_out dout=writedata[0] export pad=writedata[0]
place pad_dgnd
place pad_dvdd
place pad_dgnd
place pad_dvdd
place pad_dgnd
place pad_dvdd

; place the bottom edge of pads
rotate cc
place pad_corner
place pad_out dout=adr[7] export pad=adr[7]
place pad_out dout=adr[6] export pad=adr[6]
place pad_out dout=adr[5] export pad=adr[5]
place pad_out dout=adr[4] export pad=adr[4]
place pad_out dout=adr[3] export pad=adr[3]
place pad_out dout=adr[2] export pad=adr[2]
place pad_out dout=adr[1] export pad=adr[1]
place pad_out dout=adr[0] export pad=adr[0]
place pad_dvdd 
place pad_dgnd
place pad_dgnd
place pad_dvdd
place pad_dgnd
place pad_dvdd
place pad_dgnd
place pad_dvdd


; place the right edge of pads
rotate cc
place pad_corner
place pad_dgnd
place pad_dvdd 
place pad_dgnd
place pad_dvdd 
place pad_dgnd
place pad_dvdd 
place pad_dgnd
place pad_dgnd
place pad_dvdd
place pad_dgnd
place pad_dvdd
place pad_dgnd
place pad_dvdd
place pad_in din=reset export pad=reset
place pad_dgnd export pad = GND
place pad_dvdd export pad = VDD
