# CF-Repetier - Repetier with automatic bed compensation!

** Note: This is a work in progress. The proof of concept has been completed (relevel.py)
and works! Now the algorithm is being integrated into Repetier for fully automatic compensation. **

## Who needs calibration!?

We all do, but that doesn't mean we can't do more with imperfect printers and operators!

Having a correctly levelled and flat print surface, and in the case of delta printers, correctly calibrated delta parameters is extremely important. Without a properly levelled and flat print surface it is impossible to print large objects as the first layer will either not stick properly, become scraped by the print head, or both.

Proper physical print bed levelling is difficult and in many cases cannot be acheived. Specialist tools are required to determine that a surface is exactly perpendicular to the Z-axis and even more specialist expertise and equipment to correct this. It is not realistic to expect those building their own home 3D printer much less those buying off-the-shelf home 3D printers to have the tools or expertise to ensure a print surface is flat to within 1mm/m and level with the same tolerances. Excellent first layer quality requires this kind of precision.

Both Marlin and Repetier support some basic techniques to mitigate printer imprecision.

* Endstops to provide a reliable idea of where home is even if the axis are not exactly the designed lengths, or if an axis has missed steps in the past.
* Automatic bed 'levelling'. Using a Z-probe the firmware probes the bed to determine the plane-equation for the bed. When printing the model is rotated to match the rotation of the print surface relative to the notional 'true bed'. This way even if the bed is not quite level with respect to the printer's Z-axis the first layer will still be correct. Marlin's implementation is somewhat better as it can perform 'detailed' probing of the print surface and then compute the plane of the bed that most accurately matches the observed probe data (using a least-squares approximation). Fundamentally Repetier does the same kind of correction (by rotating the print commands according to the known rotation of the print surface) but calculates the bed's plane-equation by probing only three points. While only three points are needed to derive a plane equation any error at all in the probed points is is amplified far away from the probe locations and does not cope with warped/non-flat print surfaces at all gracefully.
* A variant of Marlin ([Here](https://github.com/RichCattell/Marlin)) also attemps to automatically calibrate some delta printer parameters using the z-probe. This depends on the assumption that the print suface is perfectly flat, the delta push rods are all equal length and the frame is square. Some of these assumptions are harder to deliver on than others.

All of these techniques so far depend on a perfectly flat print surface even if it's a little unlevel. In many cases (and for both printers I've built) this turned out to be unreasonable. Here's a few reasons why:

* Printed circuit boards as print beds are not flat at all.
* Aluminium sheets (unless machined or lapped) are not very flat, especially if you were to buy cheap guillotined sheets from eBay. (Although aluminium is an excellent surface for heated print beds in other respects.)
* Even with a perfectly flat surface such as a glass sheet, some printers with a moving bed may still have an effectively curved surface if the linear motion guides for the bed are not parallel to one another. (So the left of the bed is higher than the right at the endstop but the right side higher when at the full opposite position, for example)
* With delta printers having an incorrect value for the 'delta radius' (the length of the horizontal edge of the right angle triangle formed where the delta push rods are the hypotenuse and the linear motion element is the virtical edge). In this case the printer will move in a bowl or dome shape when requested to move in purely X or Y. The perfect bed in this case would be exactly as curved as the bowl/dome error to ensure a good first layer. Of course it'd be best to have a a perfectly calibrated delta but it's not practical advice to say "Just be better". Sometimes and for complex reasons it's just not possible.

## So what's this?

This firmware will allow any printer with any geometric defect to print first layers perfectly. It does this by warping the requested print moves according to a detailed map of the print surface measured by the printer. This could (if you have imagination) even allow you to print onto deliberately curved surfaces for special applications. Importantly, this warps the print by the *measured* map of the bed, not the bed itself. This means even delta miscalibrations can be compensated for.

There's no such thing as a free lunch. The less flat, less level the print surface is the more distortion must be introduced to follow the print surface. This will reduce the accuracy of the print. Likewise, if this is used to correct for a badly calibrated delta the resulting print geometry will not necessarily be true and may have other exciting artefacts and wonk.

On a perfect printer, prints using this firmware won't be any different but on an imperfect printer it's the difference between frustration of popped-off prints, badly smashed beds, scraped first layers with poor finish quality, and a well printed (albeit inaccurate) object.

BUT AT LEAST IT PRINTS.

on any printer, no matter how badly setup. (almost)

So what *must* be correct with my printer?

* Extrusion, heating, etc are not considered here.
* A working, reliable Z-probe is needed. An unreliable Z-probe will undermine this approach. A microswitch based Z-probe should be fine in almost all cases. It's also important to know the Z-offset of the probe relative to the extruder.
* Sane motion; Steps per mm for each axis should be as close as possible to reality. This firmware won't work (just like any other) magically with wacky axis settings. For small errors this will probably only affect output geometric accuracy. Large errors and inconsistencies may entirely undermine this approach.
* An idea of the magnitude of error in the printer. In principle very large errors can be compensated for, but it's much faster to perform the bed survey if you know ahead of time that the largest error is <5mm, for example. It is probably best to do some preliminary experiments (possibly with this firmware) to get an idea of the printer's z-height first to minimise easy to correct error.
* Sharp bumps can't be corrected for. The sharpest correctable bumps are completely smooth and have a radius no smaller than the bed probe spacing parameter. (For example, a smooth bump fully 5cm across in the bed would need probe spacing absolutely no further apart than 25mm or so). This is the bare minimum spacing, or more appropriately the absolute minimum  size of the bumps/dents that are correctable. AVR-based arduinos simply don't have enough ram for much more detailed bed surveys, especially with large print beds. Additionally, more detailed bed surveys will require much more processing time during print for correction. This might be a problem for some printers with some AVRs (manifesting as slowdowns during print), but not with sensible settings. Maybe in the future we'll all use ARM-Arduinos.

## Features in Firmware:

Improvements over upstream:

* P option added to G29 to probe in a single location at the current (x,y) rather than 3 points. This is useful for updating in EEPROM overall printer height based on a single reference rather than an average.
* G35 implemented to perform a bed survey. This will probe the bed in a grid pattern and emit a map of the Z-offsets at each probed point.
* (in progress) G36 to perform a bed survey, generate a compensation mesh in RAM and enable bed compensation. Params: Zxx set the height to probe the bed from. (eg: z10 to probe from +10mm, default: 5mm) This should be greater than bed's error to make sure your printhead does not crash during probing. Pxx set the probe spacing in whole mm. This is the space between probed points in mm, Default 30mm. Ixx set distortion factor (The maximum fraction a line can be compressed by in order to gradually correct the print. 0.9 means a line can be compressed by 90%, or to 1/10th its original thickness), default 0.5. Jxx the height by which the print will be corrected. This is calculated automatically from the I parameter and the bed topography by default. If Jxx is specified it will override the I parameter. J5 would ensure the print use undistorted geometry by 5mm into the print.
* M336 Sx to enable/disable/reset bed compensation. I and J parameters can also be specified to change the height at which the print will no longer be distorted. (See G36)
* NOT AN IMPROVEMENT: G-code arcs are not supported (and will be disabled) do not use them!

When bed compensation is enabled all move G-codes will be adjusted by the printer to be correct relative to the Z-offset of the print bed at the current (x,y) point.

Until the firmware modification is complete the python script 'relevel.py' provides a proof of concept for this idea. Use G35 on the printer to produce a map of the bed and then relevel.py can reprocess any input gcode to perfectly sit on top of the print surface. This is a little awkward but works a treat and is fine to do until the firmware supports it transparently.

What needs configuring?

* You must make sure your firmware has enough free memory. With the default settings of 30mm mesh spacing, on a delta with bed diameter 230mm, 2KiB of RAM is needed for the bed mesh! To fit this, you'll need to reduce your MOVE_CACHE_SIZE to something like 10, (also set MOVE_CACHE_LOW to something like 6), and also reduce MAX_DELTA_SEGMENTS_PER_LINE to 20. These reductions will make the printer more prone to stalls on very high detail areas (such as small circles) but only if the gcode is far too detailed or the host PC/serial line can't keep up.
* OPTIONAL: An appropriate probe spacing. 30mm seems sensible to me, quite detailed but also will fit into RAM on an Arduino Mega 2560 (see above)
* OPTIONAL: An allowable distortion factor. The distortion factor is the maximum change in layer height the firmware can make in order to gradually correct a print. The idea is that even if a print starts very distorted for the print bed, it will eventually correct and become completely true again. (assuming other print parameters are correct). This value ranges from 0 (meaning never modify layer height, so the whole print is distorted.) to 1 (meaning that layers can be completely removed if needed to correct print geometry as fast as possible.). A value of 0.5 seems sensible meaning that some areas can be compressed by up to 50% to gradually correct print geometry. If the maximum bed deviation found during the survey was found to be 2mm, and the distortion factor is 0.5 then the print will have true geometry by Z=4mm. For printers with perfect surfaces but bad delta prameters a value of 0 is probably sensible.
* OPTIONAL: Your printers Z-height should be set such that no point on the bed has a negative Z offset. This is to avoid the G-code re-writer engine asking for moves to Z<0 positions which may hit software endstops and ignore the move. This will be done automatically during a G36 if needed.

Anything else?

Bed compensation is volatile and should be done immediately before any print and ideally while everything is hot. Especially with heated PCB surfaces the curvature is highly temperature sensitive. This is just acheived by performing adding a "G36 S1" command to your preamble gcode in your slicer after preheating your extruder and bed, failing this, just adding this to your printers startup gcode will probably be good enough and save time for each print.

## How does this work?

1. Use "G36 S1" to perform a detailed survey of the print bed. This is done by probing the bed in a grid pattern according to the configured probe spacing. If a probed point is outside the reachable area (such as the corners of a printer with a circular build surface) then the closest (x,y) grid point that is within the build ares is used.
2. The Z-height of the printer is adjusted so that the lowest probed point becomes the 0mm on the Z-axis. (So if the lowest probed point were -1.5mm then the Z-height of the printer is increased by 1.5mm)
3. Using the probe points a mesh of right angle triangles is constructed to represent the height map of the bed. The plane equation coefficients for each trianglular section are stored in RAM for use during moves. Bed compensation is now enabled and will affect all G0 and G1 moves now.
4. When a G0/G1 is issued the start and end Z positions are considered, if they're above the point where all print distortion has been corrected then the gcode is passed onto the planner as usual.
5. If the Z-height needs correcting then it is calculated by looking up height of the bed at position (x,y) and then calculating the appropriate height for the requested Z position. If the layer is compressed or expanded then the extrusion amount is also adjusted to ensure the same overall extrusion 'density'.
6. Long moves are cut into smaller sections to make sure that even single moves can still follow the contours of the print surface. Moves shorter than ~~3mm are always done as one section. If the move is split the split boundaries occur exactly on the edges of the bed height mesh. This could also be thought of as when the line representing the move crosses from one triangle of the mesh to another.
7. Move G-codes that were altered are fed into the planner in the same way as without the compensation stage. This means compensation can be used at the same time as normal bed autolevelling, and also that the planner can pick appropriate accelerations after accounting for the z-moves over the bed 'terrain'. It also means that an LCD connected to the printer will show the position after adjustment for the bed topology, not the input gcode positions.


## ... Below is from from Repetier Firmware...


# Repetier-Firmware - the fast and user friendly firmware

## Installation

Please use your new at [http://www.repetier.com/firmware/v091](http://www.repetier.com/firmware/v091)
for easy and fast configuration. You get the complete sources you need to compile back.
This system also allows it to upload configurations created with this tool and modify
the configuration.

## Version 0.91 released 2013-12-30

Improvements over old code:

* Works with CodeBlocks for Arduino http://www.arduinodev.com/codeblocks/#download
  which can replace the ArduinoIDE with a much better one on windows systems. Load the
  Repetier.cdb project file for this.
* Better readable code.
* Long filename support (from Glenn Kreisel).
* Animated menu changes.
* Separation of logic and hardware access to allow different processor architectures
  by changing the hardware related files.
* z-leveling support.
* Mirroring of x,y and z motor.
* Ditto printing.
* Faster and better delta printing.
* New heat manager (dead time control).
* Removed OPS handling.
* Full graphic display support.
* Many bug fixes.
* many other changes.

## Documentation

For documentation please visit [http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/)

## Developer

The sources are managed by the Hot-World GmbH & Co. KG
It was initially based on the Sprinter firmware from Kliment, but the code has run
through many changes since them.
Other developers:
- Glenn Kreisel (long filename support)
- Martin Croome (first delta implementation)
- John Silvia (Arduino Due port)
- sdavi (first u8glib code implementation)
- plus several small contributions from other users.

## Introduction

Repetier-Firmware is a firmware for RepRap like 3d-printer powered with
an arduino compatible controller.
This firmware is a nearly complete rewrite of the sprinter firmware by kliment
which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
Some ideas were also taken from Teacup, Grbl and Marlin.

Supported boards:

The following boards are supported by setting the proper motherboard type.Other boards
require a matching pin definition.

* MEGA/RAMPS up to 1.2       = 3
* RAMPS 1.3/RAMPS 1.4        = 33
* Azteeg X3                  = 34
* Gen6                       = 5 
* Gen6 deluxe                = 51
* Sanguinololu up to 1.1     = 6
* Sanguinololu 1.2 and above = 62
* Melzi board                = 63
* Gen7 1.1 till 1.3.x        = 7
* Gen7 1.4.1 and later       = 71
* Teensylu (at90usb)         = 8 // requires Teensyduino
* Printrboard (at90usb)      = 9 // requires Teensyduino
* Foltyn 3D Master           = 12
* MegaTronics                = 70
* Megatronics 2.0            = 701
* RUMBA                      = 80  // Get it from reprapdiscount
* Rambo                      = 301

## Features

- RAMP acceleration support.
- Path planning for higher print speeds.
- Trajectory smoothing for smoother lines.
- Ooze prevention system for faster anti ooze then slicer can do,
- Nozzle pressure control for improved print quality with RAMPS.
- Fast - 40000 Hz and more stepper frequency is possible with a 16 MHz AVR. 
- Multiple extruder supported (max. 6 extruder).
- Standard ASCII and improved binary (Repetier protocol) communication.
- Autodetect the command protocol, so it will work with any host software.
- Continuous monitoring of one temperature.
- Important parameters are stored in EEPROM and can easily be modified without
  recompilation of the firmware.
- Stepper control is handled in an interrupt routine, leaving time for
  filling caches for next move.
- PID control for extruder/heated bed temperature.
- Interrupt based sending buffer (Arduino library normally waits for the
  recipient to receive written data)
- Small RAM memory print, resulting in large caches.
- Supports SD-cards.
- mm and inches can be used for G0/G1
- Arc support
- Works with Skeinforge 41, all unknown commands are ignored.
- Dry run : Execute yout GCode without using the extruder. This way you can
  test for non-extruder related failures without actually printing.

## Controlling firmware

Also you can control the firmware with any reprap compatible host, you will only get
the full benefits with the following products, which have special code for this
firmware:

* [Repetier-Host for Windos/Linux](http://www.repetier.com/download/)
* [Repetier-Host for Mac](http://www.repetier.com/download/)
* [Repetier-Server](http://www.repetier.com/repetier-server-download/)

## Installation

For documentation and installation please visit 
[http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/).

## Changelog

See changelog.txt
