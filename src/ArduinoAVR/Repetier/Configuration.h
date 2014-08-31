/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

//Make sure we bring in the autoconfigured config
#include <Configuration-auto.h>

//Now for our new config parameters:

/*
 Experimental bed flatness compensation support
 
 For this feature to be effective the printer z-height must be set a little bit too large to allow G0 moves to go slightly negative into
 troughs in the print bed.
 
 Enabling this feature adds G35 to perform a detailed probe of the print bed.
 This map of the bed can then be used by a python script to transform gcode to match the bed perfectly.
 This also enables G36 and M336 for firmware hosted print correction.
 */
#define BEDCOMPENSATION

#ifdef BEDCOMPENSATION

//Gcode Arcs are not supported with bed compensation enabled.
#undef ARC_SUPPORT

//The height all bed probes will start at.
#define BEDCOMPENSATION_PROBEHEIGHT 5
//When a position cannot be probed (as it's outside the real print bed area) this value is returned.
#define BEDCOMPENSATION_INVALIDPOINT -99

//This area close to the edge of the bed will not be probed (as it might go beyond the reachable area after rotation due to bed levelling)
#define BEDCOMPENSATION_MARGIN 5

#define BEDCOMPENSATION_DEFAULT_SPACING 30.0

#endif