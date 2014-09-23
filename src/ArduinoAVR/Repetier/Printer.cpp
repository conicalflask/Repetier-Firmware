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

#include "Repetier.h"

#if defined(USE_ADVANCE)
uint8_t Printer::minExtruderSpeed;            ///< Timer delay for start extruder speed
uint8_t Printer::maxExtruderSpeed;            ///< Timer delay for end extruder speed
volatile int Printer::extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
//uint8_t Printer::extruderAccelerateDelay;     ///< delay between 2 speec increases
#endif
uint8_t Printer::unitIsInches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
float Printer::axisStepsPerMM[4] = {XAXIS_STEPS_PER_MM,YAXIS_STEPS_PER_MM,ZAXIS_STEPS_PER_MM,1}; ///< Number of steps per mm needed.
float Printer::invAxisStepsPerMM[4]; ///< Inverse of axisStepsPerMM for faster conversion
float Printer::maxFeedrate[4] = {MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z}; ///< Maximum allowed feedrate.
float Printer::homingFeedrate[3] = {HOMING_FEEDRATE_X, HOMING_FEEDRATE_Y, HOMING_FEEDRATE_Z};
#ifdef RAMP_ACCELERATION
//  float max_start_speed_units_per_second[4] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
float Printer::maxAccelerationMMPerSquareSecond[4] = {MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
float Printer::maxTravelAccelerationMMPerSquareSecond[4] = {MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
/** Acceleration in steps/s^3 in printing mode.*/
unsigned long Printer::maxPrintAccelerationStepsPerSquareSecond[4];
/** Acceleration in steps/s^2 in movement mode.*/
unsigned long Printer::maxTravelAccelerationStepsPerSquareSecond[4];
#endif
#if NONLINEAR_SYSTEM
long Printer::currentDeltaPositionSteps[4];
uint8_t lastMoveID = 0; // Last move ID
#endif
signed char Printer::zBabystepsMissing = 0;
uint8_t Printer::relativeCoordinateMode = false;  ///< Determines absolute (false) or relative Coordinates (true).
uint8_t Printer::relativeExtruderCoordinateMode = false;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

long Printer::currentPositionSteps[4];
float Printer::currentPosition[3];
float Printer::lastCmdPos[3];
long Printer::destinationSteps[4];
float Printer::coordinateOffset[3] = {0,0,0};
uint8_t Printer::flag0 = 0;
uint8_t Printer::flag1 = 0;
uint8_t Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
uint8_t Printer::stepsPerTimerCall = 1;
uint8_t Printer::menuMode = 0;

#if FEATURE_AUTOLEVEL
float Printer::autolevelTransformation[9]; ///< Transformation matrix
#endif
unsigned long Printer::interval;           ///< Last step duration in ticks.
unsigned long Printer::timer;              ///< used for acceleration/deceleration timing
unsigned long Printer::stepNumber;         ///< Step number in current move.
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
long Printer::advanceExecuted;             ///< Executed advance steps
#endif
int Printer::advanceStepsSet;
#endif
#if NONLINEAR_SYSTEM
long Printer::maxDeltaPositionSteps;
floatLong Printer::deltaDiagonalStepsSquaredA;
floatLong Printer::deltaDiagonalStepsSquaredB;
floatLong Printer::deltaDiagonalStepsSquaredC;
float Printer::deltaMaxRadiusSquared;
long Printer::deltaAPosXSteps;
long Printer::deltaAPosYSteps;
long Printer::deltaBPosXSteps;
long Printer::deltaBPosYSteps;
long Printer::deltaCPosXSteps;
long Printer::deltaCPosYSteps;
long Printer::realDeltaPositionSteps[3];
int16_t Printer::travelMovesPerSecond;
int16_t Printer::printMovesPerSecond;
#endif
#if FEATURE_Z_PROBE || MAX_HARDWARE_ENDSTOP_Z || NONLINEAR_SYSTEM
long Printer::stepsRemainingAtZHit;
#endif
#if DRIVE_SYSTEM==3
long Printer::stepsRemainingAtXHit;
long Printer::stepsRemainingAtYHit;
#endif
#ifdef SOFTWARE_LEVELING
long Printer::levelingP1[3];
long Printer::levelingP2[3];
long Printer::levelingP3[3];
#endif
float Printer::minimumSpeed;               ///< lowest allowed speed to keep integration error small
float Printer::minimumZSpeed;
long Printer::xMaxSteps;                   ///< For software endstops, limit of move in positive direction.
long Printer::yMaxSteps;                   ///< For software endstops, limit of move in positive direction.
long Printer::zMaxSteps;                   ///< For software endstops, limit of move in positive direction.
long Printer::xMinSteps;                   ///< For software endstops, limit of move in negative direction.
long Printer::yMinSteps;                   ///< For software endstops, limit of move in negative direction.
long Printer::zMinSteps;                   ///< For software endstops, limit of move in negative direction.
float Printer::xLength;
float Printer::xMin;
float Printer::yLength;
float Printer::yMin;
float Printer::zLength;
float Printer::zMin;
float Printer::feedrate;                   ///< Last requested feedrate.
int Printer::feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
unsigned int Printer::extrudeMultiply;     ///< Flow multiplier in percdent (factor 1 = 100)
float Printer::maxJerk;                    ///< Maximum allowed jerk in mm/s
#if DRIVE_SYSTEM!=3
float Printer::maxZJerk;                   ///< Maximum allowed jerk in z direction in mm/s
#endif
float Printer::offsetX;                     ///< X-offset for different extruder positions.
float Printer::offsetY;                     ///< Y-offset for different extruder positions.
unsigned int Printer::vMaxReached;         ///< Maximumu reached speed
unsigned long Printer::msecondsPrinting;            ///< Milliseconds of printing time (means time with heated extruder)
float Printer::filamentPrinted;            ///< mm of filament printed since counting started
uint8_t Printer::wasLastHalfstepping;         ///< Indicates if last move had halfstepping enabled
#if ENABLE_BACKLASH_COMPENSATION
float Printer::backlashX;
float Printer::backlashY;
float Printer::backlashZ;
uint8_t Printer::backlashDir;
#endif
#ifdef DEBUG_STEPCOUNT
long Printer::totalStepsRemaining;
#endif
#if FEATURE_MEMORY_POSITION
float Printer::memoryX;
float Printer::memoryY;
float Printer::memoryZ;
float Printer::memoryE;
float Printer::memoryF;
#endif
#ifdef XY_GANTRY
int8_t Printer::motorX;
int8_t Printer::motorY;
#endif
#ifdef DEBUG_SEGMENT_LENGTH
    float Printer::maxRealSegmentLength = 0;
#endif
#ifdef DEBUG_REAL_JERK
    float Printer::maxRealJerk = 0;
#endif
#ifdef DEBUG_PRINT
int debugWaitLoop = 0;
#endif

#ifdef BEDCOMPENSATION

    //The X axis position in mm of the first mesh probe point
    float Printer::meshOffsetX;

    //The Y axis position in mm of the first mesh probe point
    float Printer::meshOffsetY;

    //The distance between probe points in mm
    float Printer::meshSpacing;

    //The number of mesh squares in a row of the mesh. It is assumed the mesh is square. This is the number of probed points across, minus 1.
    char Printer::meshWidth;

    //The height at which the bed is probed.
    float Printer::bedCompensationProbeHeight;

    //The mesh (if constructed) of the print surface
    struct meshTriangle* Printer::mesh = 0;

    //The Z height in MM by which the geometry of the print is no longer distorted to match the bed 
    float Printer::correctedByZ;

    //The maximum Z offset of all probed points. This is used if M336 is used to readjust bed compensation parameters.
    float Printer::maxProbedZ;

    //The status of the bed compensation system. 0 is disabled.
    char Printer::bedCompensationStatus = 0 ;

    //So, we need to know the firmware's idea of the E position in native units. (or at least until I apply my brain to this properly.)
    float Printer::Eposition;

	//This value is mostly for fun. It's the ((sum of squares of the probed bed offsets) divided by the number of probe points) * 100 (to make it a more sensible range as good printers will be <0.1 otherwise.)
	float Printer::bedBadnessScore;

    //The most recently requested (from source GCode) Z position. Needed as the currentPosition[Z_AXIS] is post-mangling.
    float Printer::currentPositionZgCode;

#endif


void Printer::constrainDestinationCoords()
{
    if(isNoDestinationCheck()) return;
#if min_software_endstop_x == true
    if (destinationSteps[X_AXIS] < xMinSteps) Printer::destinationSteps[X_AXIS] = Printer::xMinSteps;
#endif
#if min_software_endstop_y == true
    if (destinationSteps[Y_AXIS] < yMinSteps) Printer::destinationSteps[Y_AXIS] = Printer::yMinSteps;
#endif
#if min_software_endstop_z == true
    if (destinationSteps[Z_AXIS] < zMinSteps) Printer::destinationSteps[Z_AXIS] = Printer::zMinSteps;
#endif

#if max_software_endstop_x == true
    if (destinationSteps[X_AXIS] > Printer::xMaxSteps) Printer::destinationSteps[X_AXIS] = Printer::xMaxSteps;
#endif
#if max_software_endstop_y == true
    if (destinationSteps[Y_AXIS] > Printer::yMaxSteps) Printer::destinationSteps[Y_AXIS] = Printer::yMaxSteps;
#endif
#if max_software_endstop_z == true
    if (destinationSteps[Z_AXIS] > Printer::zMaxSteps) Printer::destinationSteps[Z_AXIS] = Printer::zMaxSteps;
#endif
}
bool Printer::isPositionAllowed(float x,float y,float z) {
    if(isNoDestinationCheck())  return true;
    bool allowed = true;
#if DRIVE_SYSTEM == 3
    allowed &= (z >= 0) && (z <= zLength+0.05+ENDSTOP_Z_BACK_ON_HOME);
    allowed &= (x * x + y * y <= deltaMaxRadiusSquared);
#endif // DRIVE_SYSTEM
    if(!allowed) {
        Printer::updateCurrentPosition(true);
        Commands::printCurrentPosition();
    }
    return allowed;
}
void Printer::updateDerivedParameter()
{
#if DRIVE_SYSTEM==3
    travelMovesPerSecond = EEPROM::deltaSegmentsPerSecondMove();
    printMovesPerSecond = EEPROM::deltaSegmentsPerSecondPrint();
    axisStepsPerMM[X_AXIS] = axisStepsPerMM[Y_AXIS] = axisStepsPerMM[Z_AXIS];
    maxAccelerationMMPerSquareSecond[X_AXIS] = maxAccelerationMMPerSquareSecond[Y_AXIS] = maxAccelerationMMPerSquareSecond[Z_AXIS];
    homingFeedrate[X_AXIS] = homingFeedrate[Y_AXIS] = homingFeedrate[Z_AXIS];
    maxFeedrate[X_AXIS] = maxFeedrate[Y_AXIS] = maxFeedrate[Z_AXIS];
    maxTravelAccelerationMMPerSquareSecond[X_AXIS] = maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = maxTravelAccelerationMMPerSquareSecond[Z_AXIS];
    zMaxSteps = axisStepsPerMM[Z_AXIS]*(zLength - zMin);
    float radius0 = EEPROM::deltaHorizontalRadius();
    float radiusA = radius0 + EEPROM::deltaRadiusCorrectionA();
    float radiusB = radius0 + EEPROM::deltaRadiusCorrectionB();
    float radiusC = radius0 + EEPROM::deltaRadiusCorrectionC();
    deltaAPosXSteps = floor(radiusA * cos(EEPROM::deltaAlphaA() * M_PI/180.0) * axisStepsPerMM[Z_AXIS] + 0.5);
    deltaAPosYSteps = floor(radiusA * sin(EEPROM::deltaAlphaA() * M_PI/180.0) * axisStepsPerMM[Z_AXIS] + 0.5);
    deltaBPosXSteps = floor(radiusB * cos(EEPROM::deltaAlphaB() * M_PI/180.0) * axisStepsPerMM[Z_AXIS] + 0.5);
    deltaBPosYSteps = floor(radiusB * sin(EEPROM::deltaAlphaB() * M_PI/180.0) * axisStepsPerMM[Z_AXIS] + 0.5);
    deltaCPosXSteps = floor(radiusC * cos(EEPROM::deltaAlphaC() * M_PI/180.0) * axisStepsPerMM[Z_AXIS] + 0.5);
    deltaCPosYSteps = floor(radiusC * sin(EEPROM::deltaAlphaC() * M_PI/180.0) * axisStepsPerMM[Z_AXIS] + 0.5);
    deltaDiagonalStepsSquaredA.l = static_cast<int32_t>((EEPROM::deltaDiagonalCorrectionA() + EEPROM::deltaDiagonalRodLength())*axisStepsPerMM[Z_AXIS]);
    deltaDiagonalStepsSquaredB.l = static_cast<int32_t>((EEPROM::deltaDiagonalCorrectionB() + EEPROM::deltaDiagonalRodLength())*axisStepsPerMM[Z_AXIS]);
    deltaDiagonalStepsSquaredC.l = static_cast<int32_t>((EEPROM::deltaDiagonalCorrectionC() + EEPROM::deltaDiagonalRodLength())*axisStepsPerMM[Z_AXIS]);
    if(deltaDiagonalStepsSquaredA.l>46000 || 2 * EEPROM::deltaHorizontalRadius()*axisStepsPerMM[Z_AXIS]>46000)
    {
        setLargeMachine(true);
        deltaDiagonalStepsSquaredA.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredA.l));
        deltaDiagonalStepsSquaredB.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredB.l));
        deltaDiagonalStepsSquaredC.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredC.l));
    }
    else {
        deltaDiagonalStepsSquaredA.l = RMath::sqr(deltaDiagonalStepsSquaredA.l);
        deltaDiagonalStepsSquaredB.l = RMath::sqr(deltaDiagonalStepsSquaredB.l);
        deltaDiagonalStepsSquaredC.l = RMath::sqr(deltaDiagonalStepsSquaredC.l);
    }
    deltaMaxRadiusSquared = RMath::sqr(EEPROM::deltaMaxRadius());
    long cart[3], delta[3];
    cart[X_AXIS] = cart[Y_AXIS] = 0;
    cart[Z_AXIS] = zMaxSteps;
    transformCartesianStepsToDeltaSteps(cart, delta);
    maxDeltaPositionSteps = delta[0];
    xMaxSteps = yMaxSteps = zMaxSteps;
    xMinSteps = yMinSteps = zMinSteps = 0;
#elif DRIVE_SYSTEM==4
    deltaDiagonalStepsSquared = long(EEPROM::deltaDiagonalRodLength()*axisStepsPerMM[X_AXIS]);
    if(deltaDiagonalStepsSquared>46000)
    {
        setLargeMachine(true);
        deltaDiagonalStepsSquaredF = float(deltaDiagonalStepsSquared)*float(deltaDiagonalStepsSquared);
    }
    else
        deltaDiagonalStepsSquared = deltaDiagonalStepsSquared*deltaDiagonalStepsSquared;
    deltaBPosXSteps = long(EEPROM::deltaDiagonalRodLength()*axisStepsPerMM[X_AXIS]);
    xMaxSteps = (long)(axisStepsPerMM[X_AXIS]*(xMin+xLength));
    yMaxSteps = (long)(axisStepsPerMM[Y_AXIS]*yLength);
    zMaxSteps = (long)(axisStepsPerMM[Z_AXIS]*(zMin+zLength));
    xMinSteps = (long)(axisStepsPerMM[X_AXIS]*xMin);
    yMinSteps = 0;
    zMinSteps = (long)(axisStepsPerMM[Z_AXIS]*zMin);
#else
    xMaxSteps = (long)(axisStepsPerMM[X_AXIS]*(xMin+xLength));
    yMaxSteps = (long)(axisStepsPerMM[Y_AXIS]*(yMin+yLength));
    zMaxSteps = (long)(axisStepsPerMM[Z_AXIS]*(zMin+zLength));
    xMinSteps = (long)(axisStepsPerMM[X_AXIS]*xMin);
    yMinSteps = (long)(axisStepsPerMM[Y_AXIS]*yMin);
    zMinSteps = (long)(axisStepsPerMM[Z_AXIS]*zMin);
    // For which directions do we need backlash compensation
#if ENABLE_BACKLASH_COMPENSATION
    backlashDir &= 7;
    if(backlashX!=0) backlashDir |= 8;
    if(backlashY!=0) backlashDir |= 16;
    if(backlashZ!=0) backlashDir |= 32;
#endif
#endif
    for(uint8_t i=0; i<4; i++)
    {
        invAxisStepsPerMM[i] = 1.0f/axisStepsPerMM[i];
#ifdef RAMP_ACCELERATION
        /** Acceleration in steps/s^3 in printing mode.*/
        maxPrintAccelerationStepsPerSquareSecond[i] = maxAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
        /** Acceleration in steps/s^2 in movement mode.*/
        maxTravelAccelerationStepsPerSquareSecond[i] = maxTravelAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
#endif
    }
    float accel = RMath::max(maxAccelerationMMPerSquareSecond[X_AXIS],maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
    minimumSpeed = accel*sqrt(2.0f/(axisStepsPerMM[X_AXIS]*accel));
    accel = RMath::max(maxAccelerationMMPerSquareSecond[Z_AXIS],maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
    minimumZSpeed = accel*sqrt(2.0f/(axisStepsPerMM[Z_AXIS]*accel));
    Printer::updateAdvanceFlags();
}
/**
  \brief Stop heater and stepper motors. Disable power,if possible.
*/
void Printer::kill(uint8_t only_steppers)
{
    if(areAllSteppersDisabled() && only_steppers) return;
    if(Printer::isAllKilled()) return;
    setAllSteppersDiabled();
    disableXStepper();
    disableYStepper();
    disableZStepper();
    Extruder::disableAllExtruderMotors();
#if FAN_BOARD_PIN>-1
    pwm_pos[NUM_EXTRUDER+1] = 0;
#endif // FAN_BOARD_PIN
    if(!only_steppers)
    {
        for(uint8_t i=0; i<NUM_TEMPERATURE_LOOPS; i++)
            Extruder::setTemperatureForExtruder(0,i);
        Extruder::setHeatedBedTemperature(0);
        UI_STATUS_UPD(UI_TEXT_KILLED);
#if defined(PS_ON_PIN) && PS_ON_PIN>-1
        //pinMode(PS_ON_PIN,INPUT);
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
#endif
        Printer::setAllKilled(true);
    }
    else UI_STATUS_UPD(UI_TEXT_STEPPER_DISABLED);
}

void Printer::updateAdvanceFlags()
{
    Printer::setAdvanceActivated(false);
#if defined(USE_ADVANCE)
    for(uint8_t i=0; i<NUM_EXTRUDER; i++)
    {
        if(extruder[i].advanceL!=0)
        {
            Printer::setAdvanceActivated(true);
        }
#ifdef ENABLE_QUADRATIC_ADVANCE
        if(extruder[i].advanceK!=0) Printer::setAdvanceActivated(true);
#endif
    }
#endif
}

void Printer::moveTo(float x,float y,float z,float e,float f)
{
    if(x != IGNORE_COORDINATE)
        destinationSteps[X_AXIS] = (x + Printer::offsetX) * axisStepsPerMM[X_AXIS];
    if(y != IGNORE_COORDINATE)
        destinationSteps[Y_AXIS] = (y + Printer::offsetY) * axisStepsPerMM[Y_AXIS];
    if(z != IGNORE_COORDINATE)
        destinationSteps[Z_AXIS] = z * axisStepsPerMM[Z_AXIS];
    if(e != IGNORE_COORDINATE)
        destinationSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
    if(f != IGNORE_COORDINATE)
        feedrate = f;
#if NONLINEAR_SYSTEM
    PrintLine::queueDeltaMove(ALWAYS_CHECK_ENDSTOPS, true, false); // Disable software endstop or we get wrong distances when length < real length
#else
    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS,true);
#endif
    updateCurrentPosition(false);
}

void Printer::moveToReal(float x,float y,float z,float e,float f)
{
    if(x == IGNORE_COORDINATE)
        x = currentPosition[X_AXIS];
    if(y == IGNORE_COORDINATE)
        y = currentPosition[Y_AXIS];
    if(z == IGNORE_COORDINATE)
        z = currentPosition[Z_AXIS];
    currentPosition[X_AXIS] = x;
    currentPosition[Y_AXIS] = y;
    currentPosition[Z_AXIS] = z;
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
    if(isAutolevelActive())
        transformToPrinter(x + Printer::offsetX,y + Printer::offsetY,z,x,y,z);
    else
#endif // FEATURE_AUTOLEVEL
    {
        x += Printer::offsetX;
        y += Printer::offsetY;
    }
    if(x != IGNORE_COORDINATE)
        destinationSteps[X_AXIS] = floor(x * axisStepsPerMM[X_AXIS] + 0.5);
    if(y != IGNORE_COORDINATE)
        destinationSteps[Y_AXIS] = floor(y * axisStepsPerMM[Y_AXIS] + 0.5);
    if(z != IGNORE_COORDINATE)
        destinationSteps[Z_AXIS] = floor(z * axisStepsPerMM[Z_AXIS] + 0.5);
    if(e != IGNORE_COORDINATE)
        destinationSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
    if(f != IGNORE_COORDINATE)
        Printer::feedrate = f;

#if NONLINEAR_SYSTEM
    PrintLine::queueDeltaMove(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS,true);
#endif
}

void Printer::setOrigin(float xOff,float yOff,float zOff)
{
    coordinateOffset[X_AXIS] = xOff;
    coordinateOffset[Y_AXIS] = yOff;
    coordinateOffset[Z_AXIS] = zOff;
}

void Printer::updateCurrentPosition(bool copyLastCmd)
{
    currentPosition[X_AXIS] = (float)(currentPositionSteps[X_AXIS])*invAxisStepsPerMM[X_AXIS];
    currentPosition[Y_AXIS] = (float)(currentPositionSteps[Y_AXIS])*invAxisStepsPerMM[Y_AXIS];
    currentPosition[Z_AXIS] = (float)(currentPositionSteps[Z_AXIS])*invAxisStepsPerMM[Z_AXIS];
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
    if(isAutolevelActive())
        transformFromPrinter(currentPosition[X_AXIS],currentPosition[Y_AXIS],currentPosition[Z_AXIS],currentPosition[X_AXIS],currentPosition[Y_AXIS],currentPosition[Z_AXIS]);
#endif // FEATURE_AUTOLEVEL
    currentPosition[X_AXIS] -= Printer::offsetX;
    currentPosition[Y_AXIS] -= Printer::offsetY;
    if(copyLastCmd) {
        lastCmdPos[X_AXIS] = currentPosition[X_AXIS];
        lastCmdPos[Y_AXIS] = currentPosition[Y_AXIS];
        lastCmdPos[Z_AXIS] = currentPosition[Z_AXIS];
    }
    //Com::printArrayFLN(Com::tP,currentPosition,3,4);
}

/**
  \brief Sets the destination coordinates to values stored in com.

  For the computation of the destination, the following facts are considered:
  - Are units inches or mm.
  - Reltive or absolute positioning with special case only extruder relative.
  - Offset in x and y direction for multiple extruder support.
*/

uint8_t Printer::setDestinationStepsFromGCode(GCode *com)
{
    register long p;
    /*if(!PrintLine::hasLines()) // only print for the first line
    {
        UI_STATUS(UI_TEXT_PRINTING);
    }*/
    float x,y,z;
    if(!relativeCoordinateMode)
    {
        if(com->hasX()) lastCmdPos[X_AXIS] = currentPosition[X_AXIS] = convertToMM(com->X) - coordinateOffset[X_AXIS];
        if(com->hasY()) lastCmdPos[Y_AXIS] = currentPosition[Y_AXIS] = convertToMM(com->Y) - coordinateOffset[Y_AXIS];
        if(com->hasZ()) lastCmdPos[Z_AXIS] = currentPosition[Z_AXIS] = convertToMM(com->Z) - coordinateOffset[Z_AXIS];
    }
    else
    {
        if(com->hasX()) currentPosition[X_AXIS] = (lastCmdPos[X_AXIS] += convertToMM(com->X));
        if(com->hasY()) currentPosition[Y_AXIS] = (lastCmdPos[Y_AXIS] += convertToMM(com->Y));
        if(com->hasZ()) currentPosition[Z_AXIS] = (lastCmdPos[Z_AXIS] += convertToMM(com->Z));
    }
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE //&& DRIVE_SYSTEM!=3
    if(isAutolevelActive())
    {
        transformToPrinter(lastCmdPos[X_AXIS] + Printer::offsetX, lastCmdPos[Y_AXIS] + Printer::offsetY, lastCmdPos[Z_AXIS], x, y, z);
    }
    else
#endif // FEATURE_AUTOLEVEL
    {
        x = lastCmdPos[X_AXIS] + Printer::offsetX;
        y = lastCmdPos[Y_AXIS] + Printer::offsetY;
        z = lastCmdPos[Z_AXIS];
    }
    long xSteps = static_cast<long>(floor(x * axisStepsPerMM[X_AXIS] + 0.5));
    long ySteps = static_cast<long>(floor(y * axisStepsPerMM[Y_AXIS] + 0.5));
    long zSteps = static_cast<long>(floor(z * axisStepsPerMM[Z_AXIS] + 0.5));
    destinationSteps[X_AXIS] = xSteps;
    destinationSteps[Y_AXIS] = ySteps;
    destinationSteps[Z_AXIS] = zSteps;
    if(com->hasE() && !Printer::debugDryrun())
    {
        p = convertToMM(com->E * axisStepsPerMM[E_AXIS]);
        if(relativeCoordinateMode || relativeExtruderCoordinateMode)
        {
            if(
#if MIN_EXTRUDER_TEMP > 30
                Extruder::current->tempControl.currentTemperatureC<MIN_EXTRUDER_TEMP ||
#endif
                fabs(com->E)>EXTRUDE_MAXLENGTH)
                p = 0;
            destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS] + p;
        }
        else
        {
            if(
#if MIN_EXTRUDER_TEMP > 30
                Extruder::current->tempControl.currentTemperatureC<MIN_EXTRUDER_TEMP ||
#endif
                fabs(p - currentPositionSteps[E_AXIS]) > EXTRUDE_MAXLENGTH * axisStepsPerMM[E_AXIS])
                currentPositionSteps[E_AXIS] = p;
            destinationSteps[E_AXIS] = p;
        }
    }
    else Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS];
    if(com->hasF())
    {
        if(unitIsInches)
            feedrate = com->F * 0.0042333f * (float)feedrateMultiply;  // Factor is 25.5/60/100
        else
            feedrate = com->F * (float)feedrateMultiply * 0.00016666666f;
    }
    if(!Printer::isPositionAllowed(lastCmdPos[X_AXIS],lastCmdPos[Y_AXIS], lastCmdPos[Z_AXIS])) {
        currentPositionSteps[E_AXIS] = destinationSteps[E_AXIS];
        return false; // ignore move
    }
    return !com->hasNoXYZ() || (com->hasE() && destinationSteps[E_AXIS] != currentPositionSteps[E_AXIS]); // ignore unproductive moves
}

void Printer::setup()
{
    HAL::stopWatchdog();
#if FEATURE_CONTROLLER==5
    HAL::delayMilliseconds(100);
#endif // FEATURE_CONTROLLER
    //HAL::delayMilliseconds(500);  // add a delay at startup to give hardware time for initalization
    HAL::hwSetup();
#ifdef ANALYZER
// Channel->pin assignments
#if ANALYZER_CH0>=0
    SET_OUTPUT(ANALYZER_CH0);
#endif
#if ANALYZER_CH1>=0
    SET_OUTPUT(ANALYZER_CH1);
#endif
#if ANALYZER_CH2>=0
    SET_OUTPUT(ANALYZER_CH2);
#endif
#if ANALYZER_CH3>=0
    SET_OUTPUT(ANALYZER_CH3);
#endif
#if ANALYZER_CH4>=0
    SET_OUTPUT(ANALYZER_CH4);
#endif
#if ANALYZER_CH5>=0
    SET_OUTPUT(ANALYZER_CH5);
#endif
#if ANALYZER_CH6>=0
    SET_OUTPUT(ANALYZER_CH6);
#endif
#if ANALYZER_CH7>=0
    SET_OUTPUT(ANALYZER_CH7);
#endif
#endif

#if defined(ENABLE_POWER_ON_STARTUP) && PS_ON_PIN>-1
    SET_OUTPUT(PS_ON_PIN); //GND
    WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
#endif
#if SDSUPPORT
    //power to SD reader
#if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER,HIGH);
#endif
#if defined(SDCARDDETECT) && SDCARDDETECT>-1
    SET_INPUT(SDCARDDETECT);
    PULLUP(SDCARDDETECT,HIGH);
#endif
#endif

    //Initialize Step Pins
    SET_OUTPUT(X_STEP_PIN);
    SET_OUTPUT(Y_STEP_PIN);
    SET_OUTPUT(Z_STEP_PIN);

    //Initialize Dir Pins
#if X_DIR_PIN>-1
    SET_OUTPUT(X_DIR_PIN);
#endif
#if Y_DIR_PIN>-1
    SET_OUTPUT(Y_DIR_PIN);
#endif
#if Z_DIR_PIN>-1
    SET_OUTPUT(Z_DIR_PIN);
#endif

    //Steppers default to disabled.
#if X_ENABLE_PIN > -1
    SET_OUTPUT(X_ENABLE_PIN);
    WRITE(X_ENABLE_PIN,!X_ENABLE_ON);
#endif
#if Y_ENABLE_PIN > -1
    SET_OUTPUT(Y_ENABLE_PIN);
    WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON);
#endif
#if Z_ENABLE_PIN > -1
    SET_OUTPUT(Z_ENABLE_PIN);
    WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON);
#endif
#if FEATURE_TWO_XSTEPPER
    SET_OUTPUT(X2_STEP_PIN);
    SET_OUTPUT(X2_DIR_PIN);
#if X2_ENABLE_PIN > -1
    SET_OUTPUT(X2_ENABLE_PIN);
    WRITE(X2_ENABLE_PIN,!X_ENABLE_ON);
#endif
#endif

#if FEATURE_TWO_YSTEPPER
    SET_OUTPUT(Y2_STEP_PIN);
    SET_OUTPUT(Y2_DIR_PIN);
#if Y2_ENABLE_PIN > -1
    SET_OUTPUT(Y2_ENABLE_PIN);
    WRITE(Y2_ENABLE_PIN,!Y_ENABLE_ON);
#endif
#endif

#if FEATURE_TWO_ZSTEPPER
    SET_OUTPUT(Z2_STEP_PIN);
    SET_OUTPUT(Z2_DIR_PIN);
#if X2_ENABLE_PIN > -1
    SET_OUTPUT(Z2_ENABLE_PIN);
    WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON);
#endif
#endif

    //endstop pullups
#if MIN_HARDWARE_ENDSTOP_X
#if X_MIN_PIN>-1
    SET_INPUT(X_MIN_PIN);
#if ENDSTOP_PULLUP_X_MIN
    PULLUP(X_MIN_PIN,HIGH);
#endif
#else
#error You have defined hardware x min endstop without pin assignment. Set pin number for X_MIN_PIN
#endif
#endif
#if MIN_HARDWARE_ENDSTOP_Y
#if Y_MIN_PIN>-1
    SET_INPUT(Y_MIN_PIN);
#if ENDSTOP_PULLUP_Y_MIN
    PULLUP(Y_MIN_PIN,HIGH);
#endif
#else
#error You have defined hardware y min endstop without pin assignment. Set pin number for Y_MIN_PIN
#endif
#endif
#if MIN_HARDWARE_ENDSTOP_Z
#if Z_MIN_PIN>-1
    SET_INPUT(Z_MIN_PIN);
#if ENDSTOP_PULLUP_Z_MIN
    PULLUP(Z_MIN_PIN,HIGH);
#endif
#else
#error You have defined hardware z min endstop without pin assignment. Set pin number for Z_MIN_PIN
#endif
#endif
#if MAX_HARDWARE_ENDSTOP_X
#if X_MAX_PIN>-1
    SET_INPUT(X_MAX_PIN);
#if ENDSTOP_PULLUP_X_MAX
    PULLUP(X_MAX_PIN,HIGH);
#endif
#else
#error You have defined hardware x max endstop without pin assignment. Set pin number for X_MAX_PIN
#endif
#endif
#if MAX_HARDWARE_ENDSTOP_Y
#if Y_MAX_PIN>-1
    SET_INPUT(Y_MAX_PIN);
#if ENDSTOP_PULLUP_Y_MAX
    PULLUP(Y_MAX_PIN,HIGH);
#endif
#else
#error You have defined hardware y max endstop without pin assignment. Set pin number for Y_MAX_PIN
#endif
#endif
#if MAX_HARDWARE_ENDSTOP_Z
#if Z_MAX_PIN>-1
    SET_INPUT(Z_MAX_PIN);
#if ENDSTOP_PULLUP_Z_MAX
    PULLUP(Z_MAX_PIN,HIGH);
#endif
#else
#error You have defined hardware z max endstop without pin assignment. Set pin number for Z_MAX_PIN
#endif
#endif
#if FEATURE_Z_PROBE && Z_PROBE_PIN>-1
    SET_INPUT(Z_PROBE_PIN);
#if Z_PROBE_PULLUP
    PULLUP(Z_PROBE_PIN,HIGH);
#endif
#endif // FEATURE_FEATURE_Z_PROBE
#if FAN_PIN>-1
    SET_OUTPUT(FAN_PIN);
    WRITE(FAN_PIN,LOW);
#endif
#if FAN_BOARD_PIN>-1
    SET_OUTPUT(FAN_BOARD_PIN);
    WRITE(FAN_BOARD_PIN,LOW);
#endif
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN>-1
    SET_OUTPUT(EXT0_HEATER_PIN);
    WRITE(EXT0_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
    SET_OUTPUT(EXT1_HEATER_PIN);
    WRITE(EXT1_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
    SET_OUTPUT(EXT2_HEATER_PIN);
    WRITE(EXT2_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
    SET_OUTPUT(EXT3_HEATER_PIN);
    WRITE(EXT3_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
    SET_OUTPUT(EXT4_HEATER_PIN);
    WRITE(EXT4_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
    SET_OUTPUT(EXT5_HEATER_PIN);
    WRITE(EXT5_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT0_EXTRUDER_COOLER_PIN) && EXT0_EXTRUDER_COOLER_PIN>-1
    SET_OUTPUT(EXT0_EXTRUDER_COOLER_PIN);
    WRITE(EXT0_EXTRUDER_COOLER_PIN,LOW);
#endif
#if defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>1
    SET_OUTPUT(EXT1_EXTRUDER_COOLER_PIN);
    WRITE(EXT1_EXTRUDER_COOLER_PIN,LOW);
#endif
#if defined(EXT2_EXTRUDER_COOLER_PIN) && EXT2_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>2
    SET_OUTPUT(EXT2_EXTRUDER_COOLER_PIN);
    WRITE(EXT2_EXTRUDER_COOLER_PIN,LOW);
#endif
#if defined(EXT3_EXTRUDER_COOLER_PIN) && EXT3_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>3
    SET_OUTPUT(EXT3_EXTRUDER_COOLER_PIN);
    WRITE(EXT3_EXTRUDER_COOLER_PIN,LOW);
#endif
#if defined(EXT4_EXTRUDER_COOLER_PIN) && EXT4_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>4
    SET_OUTPUT(EXT4_EXTRUDER_COOLER_PIN);
    WRITE(EXT4_EXTRUDER_COOLER_PIN,LOW);
#endif
#if defined(EXT5_EXTRUDER_COOLER_PIN) && EXT5_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>5
    SET_OUTPUT(EXT5_EXTRUDER_COOLER_PIN);
    WRITE(EXT5_EXTRUDER_COOLER_PIN,LOW);
#endif
#if CASE_LIGHTS_PIN>=0
    SET_OUTPUT(CASE_LIGHTS_PIN);
    WRITE(CASE_LIGHTS_PIN, CASE_LIGHT_DEFAULT_ON);
#endif // CASE_LIGHTS_PIN
#ifdef XY_GANTRY
    Printer::motorX = 0;
    Printer::motorY = 0;
#endif

#if STEPPER_CURRENT_CONTROL!=CURRENT_CONTROL_MANUAL
    motorCurrentControlInit(); // Set current if it is firmware controlled
#endif
    microstepInit();
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
    resetTransformationMatrix(true);
#endif // FEATURE_AUTOLEVEL
    feedrate = 50; ///< Current feedrate in mm/s.
    feedrateMultiply = 100;
    extrudeMultiply = 100;
    lastCmdPos[X_AXIS] = lastCmdPos[Y_AXIS] = lastCmdPos[Z_AXIS] = 0;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    advanceExecuted = 0;
#endif
    advanceStepsSet = 0;
#endif
    for(uint8_t i=0; i<NUM_EXTRUDER+3; i++) pwm_pos[i]=0;
    currentPositionSteps[X_AXIS] = currentPositionSteps[Y_AXIS] = currentPositionSteps[Z_AXIS] = currentPositionSteps[E_AXIS] = 0;
    maxJerk = MAX_JERK;
#if DRIVE_SYSTEM!=3
    maxZJerk = MAX_ZJERK;
#endif
    offsetX = offsetY = 0;
    interval = 5000;
    stepsPerTimerCall = 1;
    msecondsPrinting = 0;
    filamentPrinted = 0;
    flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
    xLength = X_MAX_LENGTH;
    yLength = Y_MAX_LENGTH;
    zLength = Z_MAX_LENGTH;
    xMin = X_MIN_POS;
    yMin = Y_MIN_POS;
    zMin = Z_MIN_POS;
    wasLastHalfstepping = 0;
#if ENABLE_BACKLASH_COMPENSATION
    backlashX = X_BACKLASH;
    backlashY = Y_BACKLASH;
    backlashZ = Z_BACKLASH;
    backlashDir = 0;
#endif
#if defined(USE_ADVANCE)
    extruderStepsNeeded = 0;
#endif
    EEPROM::initBaudrate();
    HAL::serialSetBaudrate(baudrate);
    Com::printFLN(Com::tStart);
    UI_INITIALIZE;
    HAL::showStartReason();
    Extruder::initExtruder();
    EEPROM::init(); // Read settings from eeprom if wanted
    updateDerivedParameter();
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::setupTimer();
#if NONLINEAR_SYSTEM
    transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
#if DELTA_HOME_ON_POWER
    homeAxis(true,true,true);
#endif
    Commands::printCurrentPosition();
#endif // DRIVE_SYSTEM
    Extruder::selectExtruderById(0);
#if SDSUPPORT
    sd.initsd();
#endif
#if FEATURE_WATCHDOG
    HAL::startWatchdog();
#endif // FEATURE_WATCHDOG
}

void Printer::defaultLoopActions()
{

    Commands::checkForPeriodicalActions();  //check heater every n milliseconds

    UI_MEDIUM; // do check encoder
    millis_t curtime = HAL::timeInMilliseconds();
    if(PrintLine::hasLines())
        previousMillisCmd = curtime;
    else
    {
        curtime -= previousMillisCmd;
        if(maxInactiveTime!=0 && curtime >  maxInactiveTime ) Printer::kill(false);
        else Printer::setAllKilled(false); // prevent repeated kills
        if(stepperInactiveTime!=0 && curtime >  stepperInactiveTime )
            Printer::kill(true);
    }
#if defined(SDCARDDETECT) && SDCARDDETECT>-1 && defined(SDSUPPORT) && SDSUPPORT
    sd.automount();
#endif
    DEBUG_MEMORY;
}

#if FEATURE_MEMORY_POSITION
void Printer::MemoryPosition()
{
    updateCurrentPosition(false);
    realPosition(memoryX,memoryY,memoryZ);
    memoryE = currentPositionSteps[E_AXIS]*axisStepsPerMM[E_AXIS];
    memoryF = feedrate;
}

void Printer::GoToMemoryPosition(bool x,bool y,bool z,bool e,float feed)
{
    bool all = !(x || y || z);
    moveToReal((all || x ? (lastCmdPos[X_AXIS] = memoryX) : IGNORE_COORDINATE)
               ,(all || y ?(lastCmdPos[Y_AXIS] = memoryY) : IGNORE_COORDINATE)
               ,(all || z ? (lastCmdPos[Z_AXIS] = memoryZ) : IGNORE_COORDINATE)
               ,(e ? memoryE:IGNORE_COORDINATE),
               feed);
    feedrate = memoryF;
}
#endif


#if DRIVE_SYSTEM==3
void Printer::deltaMoveToTopEndstops(float feedrate)
{
    for (uint8_t i=0; i<3; i++)
        Printer::currentPositionSteps[i] = 0;
    transformCartesianStepsToDeltaSteps(currentPositionSteps, currentDeltaPositionSteps);
    PrintLine::moveRelativeDistanceInSteps(0,0,zMaxSteps*1.5,0,feedrate, true, true);
    offsetX = 0;
    offsetY = 0;
}
void Printer::homeXAxis()
{
    destinationSteps[X_AXIS] = 0;
    PrintLine::queueDeltaMove(true,false,false);
}
void Printer::homeYAxis()
{
    Printer::destinationSteps[Y_AXIS] = 0;
    PrintLine::queueDeltaMove(true,false,false);
}
void Printer::homeZAxis() // Delta z homing
{
    deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
    PrintLine::moveRelativeDistanceInSteps(0,0,2*axisStepsPerMM[Z_AXIS]*-ENDSTOP_Z_BACK_MOVE,0,Printer::homingFeedrate[Z_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
    deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR);
#if defined(ENDSTOP_Z_BACK_ON_HOME)
    if(ENDSTOP_Z_BACK_ON_HOME > 0)
        PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[Z_AXIS]*-ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR,0,homingFeedrate[Z_AXIS],true,false);
#endif

    long dx = -EEPROM::deltaTowerXOffsetSteps();
    long dy = -EEPROM::deltaTowerYOffsetSteps();
    long dz = -EEPROM::deltaTowerZOffsetSteps();
    long dm = RMath::min(dx,RMath::min(dy,dz));
    //Com::printFLN(Com::tTower1,dx);
    //Com::printFLN(Com::tTower2,dy);
    //Com::printFLN(Com::tTower3,dz);
    dx -= dm;
    dy -= dm;
    dz -= dm;
    currentPositionSteps[X_AXIS] = 0;
    currentPositionSteps[Y_AXIS] = 0;
    currentPositionSteps[Z_AXIS] = zMaxSteps;
    transformCartesianStepsToDeltaSteps(currentPositionSteps,currentDeltaPositionSteps);
    currentDeltaPositionSteps[X_AXIS] -= dx;
    currentDeltaPositionSteps[Y_AXIS] -= dy;
    currentDeltaPositionSteps[Z_AXIS] -= dz;
    PrintLine::moveRelativeDistanceInSteps(0,0,dm,0,homingFeedrate[Z_AXIS],true,false);
    currentPositionSteps[X_AXIS] = 0;
    currentPositionSteps[Y_AXIS] = 0;
    currentPositionSteps[Z_AXIS] = zMaxSteps; // Extruder is now exactly in the delta center
    coordinateOffset[X_AXIS] = 0;
    coordinateOffset[Y_AXIS] = 0;
    coordinateOffset[Z_AXIS] = 0;
    transformCartesianStepsToDeltaSteps(currentPositionSteps, currentDeltaPositionSteps);
    realDeltaPositionSteps[X_AXIS] = currentDeltaPositionSteps[X_AXIS];
    realDeltaPositionSteps[Y_AXIS] = currentDeltaPositionSteps[Y_AXIS];
    realDeltaPositionSteps[Z_AXIS] = currentDeltaPositionSteps[Z_AXIS];
    maxDeltaPositionSteps = currentDeltaPositionSteps[X_AXIS];
#if defined(ENDSTOP_Z_BACK_ON_HOME)
    if(ENDSTOP_Z_BACK_ON_HOME > 0)
        maxDeltaPositionSteps += axisStepsPerMM[Z_AXIS]*ENDSTOP_Z_BACK_ON_HOME;
#endif
    Extruder::selectExtruderById(Extruder::current->id);
}

void Printer::homeAxis(bool xaxis,bool yaxis,bool zaxis) // Delta homing code
{
    long steps;
    setHomed(true);
    bool homeallaxis = (xaxis && yaxis && zaxis) || (!xaxis && !yaxis && !zaxis);
    if (X_MAX_PIN > -1 && Y_MAX_PIN > -1 && Z_MAX_PIN > -1 && MAX_HARDWARE_ENDSTOP_X & MAX_HARDWARE_ENDSTOP_Y && MAX_HARDWARE_ENDSTOP_Z)
    {
        UI_STATUS_UPD(UI_TEXT_HOME_DELTA);
        // Homing Z axis means that you must home X and Y
        if (homeallaxis || zaxis)
        {
            homeZAxis();
        }
        else
        {
            if (xaxis) Printer::destinationSteps[X_AXIS] = 0;
            if (yaxis) Printer::destinationSteps[Y_AXIS] = 0;
            PrintLine::queueDeltaMove(true,false,false);
        }
        UI_CLEAR_STATUS
    }

    moveToReal(0,0,Printer::zMin+Printer::zLength,IGNORE_COORDINATE,homingFeedrate[Z_AXIS]); // Move to designed coordinates including translation
    updateCurrentPosition(true);
    #ifdef BEDCOMPENSATION
        Printer::currentPositionZgCode = Printer::currentPosition[Z_AXIS];
    #endif
    UI_CLEAR_STATUS
    Commands::printCurrentPosition();
}
#else
#if DRIVE_SYSTEM==4  // Tuga printer homing
void Printer::homeXAxis()
{
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_X && X_MIN_PIN > -1 && X_HOME_DIR==-1 && MIN_HARDWARE_ENDSTOP_Y && Y_MIN_PIN > -1 && Y_HOME_DIR==-1) ||
            (MAX_HARDWARE_ENDSTOP_X && X_MAX_PIN > -1 && X_HOME_DIR==1 && MAX_HARDWARE_ENDSTOP_Y && Y_MAX_PIN > -1 && Y_HOME_DIR==1))
    {
        long offX = 0,offY = 0;
#if NUM_EXTRUDER>1
        for(uint8_t i=0; i<NUM_EXTRUDER; i++)
        {
#if X_HOME_DIR < 0
            offX = RMath::max(offX,extruder[i].xOffset);
            offY = RMath::max(offY,extruder[i].yOffset);
#else
            offX = RMath::min(offX,extruder[i].xOffset);
            offY = RMath::min(offY,extruder[i].yOffset);
#endif
        }
        // Reposition extruder that way, that all extruders can be selected at home pos.
#endif
        UI_STATUS_UPD(UI_TEXT_HOME_X);
        steps = (Printer::xMaxSteps-Printer::xMinSteps) * X_HOME_DIR;
        currentPositionSteps[X_AXIS] = -steps;
        currentPositionSteps[Y_AXIS] = 0;
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentDeltaPositionSteps);
        PrintLine::moveRelativeDistanceInSteps(2*steps,0,0,0,homingFeedrate[X_AXIS],true,true);
        currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? xMinSteps-offX : xMaxSteps+offX;
        currentPositionSteps[Y_AXIS] = 0; //(Y_HOME_DIR == -1) ? yMinSteps-offY : yMaxSteps+offY;
        //PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[0]*-ENDSTOP_X_BACK_MOVE * X_HOME_DIR,axisStepsPerMM[1]*-ENDSTOP_X_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        // PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[0]*2*ENDSTOP_X_BACK_MOVE * X_HOME_DIR,axisStepsPerMM[1]*2*ENDSTOP_X_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS]*-ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS]*2*ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_X_BACK_ON_HOME)
        if(ENDSTOP_X_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS]*-ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS],true,false);
        // PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[0]*-ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR,axisStepsPerMM[1]*-ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR,0,0,homingFeedrate[0],true,false);
#endif
        currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? xMinSteps-offX : xMaxSteps+offX;
        currentPositionSteps[Y_AXIS] = 0; //(Y_HOME_DIR == -1) ? yMinSteps-offY : yMaxSteps+offY;
        coordinateOffset[X_AXIS] = 0;
        coordinateOffset[Y_AXIS] = 0;
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentDeltaPositionSteps);
#if NUM_EXTRUDER>1
        PrintLine::moveRelativeDistanceInSteps((Extruder::current->xOffset-offX) * X_HOME_DIR,(Extruder::current->yOffset-offY) * Y_HOME_DIR,0,0,homingFeedrate[X_AXIS],true,false);
#endif
    }
}
void Printer::homeYAxis()
{
    // Dummy function x and y homing must occur together
}
#else // cartesian printer
void Printer::homeXAxis()
{
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_X && X_MIN_PIN > -1 && X_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_X && X_MAX_PIN > -1 && X_HOME_DIR==1))
    {
        long offX = 0;
#if NUM_EXTRUDER>1
        for(uint8_t i=0; i<NUM_EXTRUDER; i++)
#if X_HOME_DIR < 0
            offX = RMath::max(offX,extruder[i].xOffset);
#else
            offX = RMath::min(offX,extruder[i].xOffset);
#endif
        // Reposition extruder that way, that all extruders can be selected at home pos.
#endif
        UI_STATUS_UPD(UI_TEXT_HOME_X);
        steps = (Printer::xMaxSteps-Printer::xMinSteps) * X_HOME_DIR;
        currentPositionSteps[X_AXIS] = -steps;
        PrintLine::moveRelativeDistanceInSteps(2*steps,0,0,0,homingFeedrate[X_AXIS],true,true);
        currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? xMinSteps-offX : xMaxSteps + offX;
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * 2 * ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_X_BACK_ON_HOME)
        if(ENDSTOP_X_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS],true,false);
#endif
        currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? xMinSteps-offX : xMaxSteps+offX;
#if NUM_EXTRUDER>1
        PrintLine::moveRelativeDistanceInSteps((Extruder::current->xOffset-offX) * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS],true,false);
#endif
    }
}
void Printer::homeYAxis()
{
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_Y && Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_Y && Y_MAX_PIN > -1 && Y_HOME_DIR==1))
    {
        long offY = 0;
#if NUM_EXTRUDER>1
        for(uint8_t i=0; i<NUM_EXTRUDER; i++)
#if Y_HOME_DIR<0
            offY = RMath::max(offY,extruder[i].yOffset);
#else
            offY = RMath::min(offY,extruder[i].yOffset);
#endif
        // Reposition extruder that way, that all extruders can be selected at home pos.
#endif
        UI_STATUS_UPD(UI_TEXT_HOME_Y);
        steps = (yMaxSteps-Printer::yMinSteps) * Y_HOME_DIR;
        currentPositionSteps[1] = -steps;
        PrintLine::moveRelativeDistanceInSteps(0,2*steps,0,0,homingFeedrate[1],true,true);
        currentPositionSteps[1] = (Y_HOME_DIR == -1) ? yMinSteps-offY : yMaxSteps+offY;
        PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[Y_AXIS]*-ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[Y_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[Y_AXIS]*2*ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[Y_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_Y_BACK_ON_HOME)
        if(ENDSTOP_Y_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[Y_AXIS]*-ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR,0,0,homingFeedrate[Y_AXIS],true,false);
#endif
        currentPositionSteps[Y_AXIS] = (Y_HOME_DIR == -1) ? yMinSteps-offY : yMaxSteps+offY;
#if NUM_EXTRUDER>1
        PrintLine::moveRelativeDistanceInSteps(0,(Extruder::current->yOffset-offY) * Y_HOME_DIR,0,0,homingFeedrate[Y_AXIS],true,false);
#endif
    }
}
#endif

void Printer::homeZAxis()
{
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_Z && Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_Z && Z_MAX_PIN > -1 && Z_HOME_DIR==1))
    {
        UI_STATUS_UPD(UI_TEXT_HOME_Z);
        steps = (zMaxSteps - zMinSteps) * Z_HOME_DIR;
        currentPositionSteps[2] = -steps;
        PrintLine::moveRelativeDistanceInSteps(0,0,2*steps,0,homingFeedrate[2],true,true);
        currentPositionSteps[2] = (Z_HOME_DIR == -1) ? zMinSteps : zMaxSteps;
        PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[Z_AXIS]*-ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homingFeedrate[Z_AXIS]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[Z_AXIS]*2*ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homingFeedrate[Z_AXIS]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_Z_BACK_ON_HOME)
        if(ENDSTOP_Z_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[Z_AXIS]*-ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR,0,homingFeedrate[Z_AXIS],true,false);
#endif
        currentPositionSteps[Z_AXIS] = (Z_HOME_DIR == -1) ? zMinSteps : zMaxSteps;
#if DRIVE_SYSTEM==4
        currentDeltaPositionSteps[Z_AXIS] = currentPositionSteps[Z_AXIS];
#endif
    }
}

void Printer::homeAxis(bool xaxis,bool yaxis,bool zaxis) // home non-delta printer
{
    float startX,startY,startZ;
    realPosition(startX,startY,startZ);
    setHomed(true);
#if !defined(HOMING_ORDER)
#define HOMING_ORDER HOME_ORDER_XYZ
#endif
#if HOMING_ORDER==HOME_ORDER_XYZ
    if(xaxis) homeXAxis();
    if(yaxis) homeYAxis();
    if(zaxis) homeZAxis();
#elif HOMING_ORDER==HOME_ORDER_XZY
    if(xaxis) homeXAxis();
    if(zaxis) homeZAxis();
    if(yaxis) homeYAxis();
#elif HOMING_ORDER==HOME_ORDER_YXZ
    if(yaxis) homeYAxis();
    if(xaxis) homeXAxis();
    if(zaxis) homeZAxis();
#elif HOMING_ORDER==HOME_ORDER_YZX
    if(yaxis) homeYAxis();
    if(zaxis) homeZAxis();
    if(xaxis) homeXAxis();
#elif HOMING_ORDER==HOME_ORDER_ZXY
    if(zaxis) homeZAxis();
    if(xaxis) homeXAxis();
    if(yaxis) homeYAxis();
#elif HOMING_ORDER==HOME_ORDER_ZYX
    if(zaxis) homeZAxis();
    if(yaxis) homeYAxis();
    if(xaxis) homeXAxis();
#endif
    if(xaxis)
    {
        if(X_HOME_DIR<0) startX = Printer::xMin;
        else startX = Printer::xMin+Printer::xLength;
    }
    if(yaxis)
    {
        if(Y_HOME_DIR<0) startY = Printer::yMin;
        else startY = Printer::yMin+Printer::yLength;
    }
    if(zaxis)
    {
        if(Z_HOME_DIR<0) startZ = Printer::zMin;
        else startZ = Printer::zMin+Printer::zLength;
    }
    updateCurrentPosition(true);
    moveToReal(startX,startY,startZ,IGNORE_COORDINATE,homingFeedrate[0]);
    UI_CLEAR_STATUS
    Commands::printCurrentPosition();
}
#endif  // Not delta printer

void Printer::zBabystep() {
    bool dir = zBabystepsMissing > 0;
    if(dir) zBabystepsMissing--; else zBabystepsMissing++;
#if DRIVE_SYSTEM == 3
        Printer::enableXStepper();
        Printer::enableYStepper();
#endif
        Printer::enableZStepper();
        Printer::unsetAllSteppersDisabled();
#if DRIVE_SYSTEM == 3
        bool xDir = Printer::getXDirection();
        bool yDir = Printer::getYDirection();
#endif
        bool zDir = Printer::getZDirection();
#if DRIVE_SYSTEM == 3
        Printer::setXDirection(dir);
        Printer::setYDirection(dir);
#endif
        Printer::setZDirection(dir);
#if DRIVE_SYSTEM == 3
        WRITE(X_STEP_PIN,HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN,HIGH);
#endif
        WRITE(Y_STEP_PIN,HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN,HIGH);
#endif
#endif
        WRITE(Z_STEP_PIN,HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN,HIGH);
#endif
        HAL::delayMicroseconds(STEPPER_HIGH_DELAY + 2);
        Printer::endXYZSteps();
#if DRIVE_SYSTEM == 3
        Printer::setXDirection(xDir);
        Printer::setYDirection(yDir);
#endif
        Printer::setZDirection(zDir);
        //HAL::delayMicroseconds(STEPPER_HIGH_DELAY + 1);
}


void Printer::setAutolevelActive(bool on)
{
#if FEATURE_AUTOLEVEL
    if(on == isAutolevelActive()) return;
    flag0 = (on ? flag0 | PRINTER_FLAG0_AUTOLEVEL_ACTIVE : flag0 & ~PRINTER_FLAG0_AUTOLEVEL_ACTIVE);
    if(on)
        Com::printInfoFLN(Com::tAutolevelEnabled);
    else
        Com::printInfoFLN(Com::tAutolevelDisabled);
    updateCurrentPosition(false);
#endif // FEATURE_AUTOLEVEL    if(isAutolevelActive()==on) return;
}
#if MAX_HARDWARE_ENDSTOP_Z
float Printer::runZMaxProbe()
{
#if NONLINEAR_SYSTEM
    long startZ = realDeltaPositionSteps[Z_AXIS] = currentDeltaPositionSteps[Z_AXIS]; // update real
#endif
    Commands::waitUntilEndOfAllMoves();
    long probeDepth = 2*(Printer::zMaxSteps-Printer::zMinSteps);
    stepsRemainingAtZHit = -1;
    setZProbingActive(true);
    PrintLine::moveRelativeDistanceInSteps(0,0,probeDepth,0,EEPROM::zProbeSpeed(),true,true);
    if(stepsRemainingAtZHit<0)
    {
        Com::printErrorFLN(Com::tZProbeFailed);
        return -1;
    }
    setZProbingActive(false);
    currentPositionSteps[Z_AXIS] -= stepsRemainingAtZHit;
#if NONLINEAR_SYSTEM
    probeDepth -= (realDeltaPositionSteps[Z_AXIS] - startZ);
#else
    probeDepth -= stepsRemainingAtZHit;
#endif
    float distance = (float)probeDepth*invAxisStepsPerMM[Z_AXIS];
    Com::printF(Com::tZProbeMax,distance);
    Com::printF(Com::tSpaceXColon,realXPosition());
    Com::printFLN(Com::tSpaceYColon,realYPosition());
    PrintLine::moveRelativeDistanceInSteps(0,0,-probeDepth,0,EEPROM::zProbeSpeed(),true,true);
    return distance;
}
#endif

#if FEATURE_Z_PROBE
float Printer::runZProbe(bool first,bool last,uint8_t repeat,bool runStartScript)
{
    float oldOffX =  Printer::offsetX;
    float oldOffY =  Printer::offsetY;
    if(first)
    {
        if(runStartScript)
            GCode::executeFString(Com::tZProbeStartScript);
        Printer::offsetX = -EEPROM::zProbeXOffset();
        Printer::offsetY = -EEPROM::zProbeYOffset();
        PrintLine::moveRelativeDistanceInSteps((Printer::offsetX - oldOffX) * Printer::axisStepsPerMM[X_AXIS],
                                               (Printer::offsetY - oldOffY) * Printer::axisStepsPerMM[Y_AXIS],0,0,EEPROM::zProbeXYSpeed(),true,ALWAYS_CHECK_ENDSTOPS);
    }
    Commands::waitUntilEndOfAllMoves();
    int32_t sum = 0,probeDepth,shortMove = (int32_t)((float)Z_PROBE_SWITCHING_DISTANCE*axisStepsPerMM[Z_AXIS]);
    int32_t lastCorrection = currentPositionSteps[Z_AXIS];
#if NONLINEAR_SYSTEM
    realDeltaPositionSteps[Z_AXIS] = currentDeltaPositionSteps[Z_AXIS]; // update real
#endif
    int32_t updateZ = 0;
    for(uint8_t r=0; r<repeat; r++)
    {
        probeDepth = 2 * (Printer::zMaxSteps - Printer::zMinSteps);
        stepsRemainingAtZHit = -1;
        int32_t offx = axisStepsPerMM[X_AXIS] * EEPROM::zProbeXOffset();
        int32_t offy = axisStepsPerMM[Y_AXIS] * EEPROM::zProbeYOffset();
        //PrintLine::moveRelativeDistanceInSteps(-offx,-offy,0,0,EEPROM::zProbeXYSpeed(),true,true);
        waitForZProbeStart();
        setZProbingActive(true);
        PrintLine::moveRelativeDistanceInSteps(0,0,-probeDepth,0,EEPROM::zProbeSpeed(),true,true);
        if(stepsRemainingAtZHit<0)
        {
            Com::printErrorFLN(Com::tZProbeFailed);
            return -1;
        }
        setZProbingActive(false);
#if NONLINEAR_SYSTEM
        stepsRemainingAtZHit = realDeltaPositionSteps[Z_AXIS] - currentDeltaPositionSteps[Z_AXIS];
#endif
#if DRIVE_SYSTEM == 3
        currentDeltaPositionSteps[X_AXIS] += stepsRemainingAtZHit;
        currentDeltaPositionSteps[Y_AXIS] += stepsRemainingAtZHit;
        currentDeltaPositionSteps[Z_AXIS] += stepsRemainingAtZHit;
#endif
        currentPositionSteps[Z_AXIS] += stepsRemainingAtZHit; // now current position is correct
        if(r == 0 && first) {// Modify start z position on first probe hit to speed the ZProbe process
            int32_t newLastCorrection = currentPositionSteps[Z_AXIS] + (int32_t)((float)EEPROM::zProbeBedDistance() * axisStepsPerMM[Z_AXIS]);
            if(newLastCorrection < lastCorrection) {
                    updateZ = lastCorrection - newLastCorrection;
                lastCorrection = newLastCorrection;
            }
        }
        sum += lastCorrection - currentPositionSteps[Z_AXIS];
        if(r + 1 < repeat)
            PrintLine::moveRelativeDistanceInSteps(0,0,shortMove,0,EEPROM::zProbeSpeed(),true,false);
    }
    float distance = (float)sum * invAxisStepsPerMM[Z_AXIS] / (float)repeat + EEPROM::zProbeHeight();
    //Com::printF(Com::tZProbe,distance);
    //Com::printF(Com::tSpaceXColon,realXPosition());
    //Com::printFLN(Com::tSpaceYColon,realYPosition());
    // Go back to start position
    PrintLine::moveRelativeDistanceInSteps(0,0,lastCorrection-currentPositionSteps[Z_AXIS],0,EEPROM::zProbeSpeed(),true,false);
    //PrintLine::moveRelativeDistanceInSteps(offx,offy,0,0,EEPROM::zProbeXYSpeed(),true,true);

    if(last)
    {
        oldOffX = Printer::offsetX;
        oldOffY = Printer::offsetY;
        GCode::executeFString(Com::tZProbeEndScript);
        if(Extruder::current)
        {
            Printer::offsetX = -Extruder::current->xOffset * Printer::invAxisStepsPerMM[X_AXIS];
            Printer::offsetY = -Extruder::current->yOffset * Printer::invAxisStepsPerMM[Y_AXIS];
        }
        PrintLine::moveRelativeDistanceInSteps((Printer::offsetX - oldOffX)*Printer::axisStepsPerMM[X_AXIS],
                                               (Printer::offsetY - oldOffY)*Printer::axisStepsPerMM[Y_AXIS],0,0,EEPROM::zProbeXYSpeed(),true,ALWAYS_CHECK_ENDSTOPS);
    }
    return distance;
}

void Printer::waitForZProbeStart()
{
#if Z_PROBE_WAIT_BEFORE_TEST
    if(isZProbeHit()) return;
#if UI_DISPLAY_TYPE!=0
    uid.setStatusP(Com::tHitZProbe);
    uid.refreshPage();
#endif
#ifdef DEBUG_PRINT
    debugWaitLoop = 3;
#endif
    while(!isZProbeHit())
    {
        defaultLoopActions();
    }
#ifdef DEBUG_PRINT
    debugWaitLoop = 4;
#endif
    HAL::delayMilliseconds(30);
    while(isZProbeHit())
    {
        defaultLoopActions();
    }
    HAL::delayMilliseconds(30);
    UI_CLEAR_STATUS;
#endif
}
#if FEATURE_AUTOLEVEL
void Printer::transformToPrinter(float x,float y,float z,float &transX,float &transY,float &transZ)
{
    transX = x*autolevelTransformation[0]+y*autolevelTransformation[3]+z*autolevelTransformation[6];
    transY = x*autolevelTransformation[1]+y*autolevelTransformation[4]+z*autolevelTransformation[7];
    transZ = x*autolevelTransformation[2]+y*autolevelTransformation[5]+z*autolevelTransformation[8];
}

void Printer::transformFromPrinter(float x,float y,float z,float &transX,float &transY,float &transZ)
{
    transX = x*autolevelTransformation[0]+y*autolevelTransformation[1]+z*autolevelTransformation[2];
    transY = x*autolevelTransformation[3]+y*autolevelTransformation[4]+z*autolevelTransformation[5];
    transZ = x*autolevelTransformation[6]+y*autolevelTransformation[7]+z*autolevelTransformation[8];
}

void Printer::resetTransformationMatrix(bool silent)
{
    autolevelTransformation[0] = autolevelTransformation[4] = autolevelTransformation[8] = 1;
    autolevelTransformation[1] = autolevelTransformation[2] = autolevelTransformation[3] =
                                     autolevelTransformation[5] = autolevelTransformation[6] = autolevelTransformation[7] = 0;
    if(!silent)
        Com::printInfoFLN(Com::tAutolevelReset);
}

void Printer::buildTransformationMatrix(float h1,float h2,float h3)
{
    float ax = EEPROM::zProbeX2()-EEPROM::zProbeX1();
    float ay = EEPROM::zProbeY2()-EEPROM::zProbeY1();
    float az = h1-h2;
    float bx = EEPROM::zProbeX3()-EEPROM::zProbeX1();
    float by = EEPROM::zProbeY3()-EEPROM::zProbeY1();
    float bz = h1-h3;
    // First z direction
    autolevelTransformation[6] = ay*bz-az*by;
    autolevelTransformation[7] = az*bx-ax*bz;
    autolevelTransformation[8] = ax*by-ay*bx;
    float len = sqrt(autolevelTransformation[6]*autolevelTransformation[6]+autolevelTransformation[7]*autolevelTransformation[7]+autolevelTransformation[8]*autolevelTransformation[8]);
    if(autolevelTransformation[8]<0) len = -len;
    autolevelTransformation[6] /= len;
    autolevelTransformation[7] /= len;
    autolevelTransformation[8] /= len;
    autolevelTransformation[3] = 0;
    autolevelTransformation[4] = autolevelTransformation[8];
    autolevelTransformation[5] = -autolevelTransformation[7];
    // cross(y,z)
    autolevelTransformation[0] = autolevelTransformation[4]*autolevelTransformation[8]-autolevelTransformation[5]*autolevelTransformation[7];
    autolevelTransformation[1] = autolevelTransformation[5]*autolevelTransformation[6]-autolevelTransformation[3]*autolevelTransformation[8];
    autolevelTransformation[2] = autolevelTransformation[3]*autolevelTransformation[7]-autolevelTransformation[4]*autolevelTransformation[6];
    len = sqrt(autolevelTransformation[0]*autolevelTransformation[0]+autolevelTransformation[2]*autolevelTransformation[2]);
    autolevelTransformation[0] /= len;
    autolevelTransformation[2] /= len;
    len = sqrt(autolevelTransformation[4]*autolevelTransformation[4]+autolevelTransformation[5]*autolevelTransformation[5]);
    autolevelTransformation[4] /= len;
    autolevelTransformation[5] /= len;
    Com::printArrayFLN(Com::tInfo,autolevelTransformation,9,5);
}
#endif

#endif


#ifdef BEDCOMPENSATION

#ifdef BEDCOMPENSATION_DEBUG

void sdebug(char * buf) {
    Com::print(buf);
}

void bdebug(char * buf, float val) {
    Com::print(buf);
    Com::print(" ");
    Com::printFloat(val,3);
    Com::print("\n");
}

#else

#define bdebug(x,y)

#endif


/**
 * heh.
 */
inline float sqr(float x)
{
  return x * x;
}

/**
 * Deallocates the bed mesh releasing the memory used.
 */
void Printer::freeBedMesh() {
    if (Printer::mesh) {
        free(Printer::mesh);
        Printer::mesh = 0;
    }
}

static char lpmeshX = 99, lpmeshY = 99;
static float lpmeshZ = 0.0;

/* 
 * Probes the bed at the given mesh coords but doesn't do any checking at all!
 */
float probeBedAtReal(char meshX, char meshY) {
    //Return the cached result to avoid repeatedly probing the same location unnecesarily.
    if (meshX == lpmeshX && meshY == lpmeshY) return lpmeshZ;
    float xProbe = meshX*Printer::meshSpacing + Printer::meshOffsetX + BEDCOMPENSATION_MARGIN;
    float yProbe = meshY*Printer::meshSpacing + Printer::meshOffsetY + BEDCOMPENSATION_MARGIN;
    
    Printer::moveToReal(xProbe,yProbe,Printer::bedCompensationProbeHeight,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    float pVal = Printer::runZProbe(false,false);
    Printer::updateCurrentPosition();
    lpmeshZ = Printer::currentPosition[Z_AXIS]-pVal;
    lpmeshX = meshX;
    lpmeshY = meshY;
    return lpmeshZ;
}

/**
 * Returns 1 if the mesh coords are probe-able.
 */
char isMeshLocationProbeable(char meshX, char meshY) {
    float xProbe = meshX*Printer::meshSpacing + Printer::meshOffsetX + BEDCOMPENSATION_MARGIN;
    float yProbe = meshY*Printer::meshSpacing + Printer::meshOffsetY + BEDCOMPENSATION_MARGIN;

    char ret = 1;

    #if DRIVE_SYSTEM == 3
        float xDist = xProbe-EEPROM::zProbeXOffset();
        float yDist = yProbe-EEPROM::zProbeYOffset();
        xDist = xDist*xDist + BEDCOMPENSATION_MARGIN*BEDCOMPENSATION_MARGIN;
        yDist = yDist*yDist + BEDCOMPENSATION_MARGIN*BEDCOMPENSATION_MARGIN;
        ret &= (xDist+yDist <= Printer::deltaMaxRadiusSquared);
    #endif // DRIVE_SYSTEM

    return ret;

}

/**
 * Probes the bed at the requested X, Y mesh coordinates.
 * If the requested location is impossible it will probe the closest point (that is a mesh point) that is possible.
 *
 * This couldn't really be much less efficient but I don't care. Efficiency is always tomorrow's problem, function is today's.
 */
float probeBedAt(char meshX, char meshY) {
    char closestX;
    char closestY;
    float closestDstSqr = 999999.9;
    for (char tY = 0; tY < Printer::meshWidth+1; tY++) {
        for (char tX = 0; tX < Printer::meshWidth+1; tX++) {
            if (isMeshLocationProbeable(tX,tY)) {
                float dstSqr = (meshX-tX)*(meshX-tX) + (meshY-tY)*(meshY-tY);
                if (dstSqr < closestDstSqr) {
                    closestDstSqr = dstSqr;
                    closestX = tX;
                    closestY = tY;
                }
            }
        }
    }

    return probeBedAtReal(closestX,closestY);

}

struct meshTriangle mkTriangle(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3) {
    //v1 = [x3 - x1, y3 - y1, z3 - z1]
    float v1[3] = {x3 - x1, y3 - y1, z3 - z1};

    //v2 = [x2 - x1, y2 - y1, z2 - z1]
    float v2[3] = {x2 - x1, y2 - y1, z2 - z1};

    //cp = [v1[1] * v2[2] - v1[2] * v2[1],
    //      v1[2] * v2[0] - v1[0] * v2[2],
     //     v1[0] * v2[1] - v1[1] * v2[0]]

    float cp[3] = {v1[1] * v2[2] - v1[2] * v2[1],
                   v1[2] * v2[0] - v1[0] * v2[2],
                   v1[0] * v2[1] - v1[1] * v2[0]};

    //got the normal, now normalize it:
    float magnitude = sqrt(cp[0]*cp[0]
                          +cp[1]*cp[1]
                          +cp[2]*cp[2]);

    cp[0] = cp[0]/magnitude;
    cp[1] = cp[1]/magnitude;
    cp[2] = cp[2]/magnitude;
    //the normal is now normalized.
    float a = cp[0], b = cp[1], c = cp[2];

    float d = a * x1 + b * y1 + c * z1;

    struct meshTriangle tri;

    tri.A = a;
    tri.B = b;
    tri.C = c;
    tri.D = d;

    return tri;
}

/**
 * builds a bed mesh returning the smallest Z seen.
 * This will update maxProbedZ

 * Note: The probe is deployed and retracted during this phase to avoid error introduced by deployment unfairly biasing the first probe sequence.
 * (during testing, automatic z height corrections didn't seem to have enough effect (for the retry probing), 
   however after rehoming and trying again it worked fine. This hints that the deployment is affecting z height measurement somehow, as this is the only thing that wasn't being performed for the retry probing.)
 */
float buildBedMesh0() {

    float minSeenZ = 99999.9;
    Printer::maxProbedZ = -99999.9;

    float previousRow[Printer::meshWidth+1];
    float currentRow[Printer::meshWidth+1];

    unsigned int onTriangle = 0;

	Printer::bedBadnessScore = 0.0;
	int probeCount = 0;

    //deploy probe
    Printer::moveToReal(0,0,Printer::bedCompensationProbeHeight,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    Printer::runZProbe(true,false);

    for (char tY = 0; tY < Printer::meshWidth+1; tY++) {

        //Capture the current row:
        for (char tX = 0; tX < Printer::meshWidth+1; tX++) {

            //Make sure we don't appear fully dead:
            GCode::readFromSerial();
            Commands::checkForPeriodicalActions();

            previousRow[tX] = currentRow[tX];
            float probed = currentRow[tX] = probeBedAt(tX,tY);

			Printer::bedBadnessScore += sqr(probed);
			probeCount++;

            minSeenZ = fmin(minSeenZ,probed);
            Printer::maxProbedZ = fmax(Printer::maxProbedZ, probed);
        }

        Com::printArrayFLN(Com::tProbeRow,currentRow,Printer::meshWidth+1,3);

        //Build triangles for this row:
        if (tY>0) { //Don't build triangles if we've only probed one row. There's not enough probe points yet.
            for (char tX = 1; tX < Printer::meshWidth+1; tX++) {
                float posX = tX*Printer::meshSpacing+Printer::meshOffsetX;
                float posY = tY*Printer::meshSpacing+Printer::meshOffsetY;
                float posXm1 = (tX-1)*Printer::meshSpacing+Printer::meshOffsetX;
                float posYm1 = (tY-1)*Printer::meshSpacing+Printer::meshOffsetY;


                //triangle A: (pX-1,pY),(pX-1,pY-1),(pX,pY-1)
                Printer::mesh[onTriangle++] = mkTriangle(posXm1, posY, currentRow[tX-1],
                                                         posXm1, posYm1, previousRow[tX-1],
                                                         posX, posYm1, previousRow[tX]);

                //Add triangle B: (pX-1,pY),(pX,pY),(pX,pY-1)
                Printer::mesh[onTriangle++] = mkTriangle(posXm1, posY, currentRow[tX-1],
                                                         posX, posY, currentRow[tX],
                                                         posX, posYm1, previousRow[tX]);
            }
        }
    }

    //Retract probe:
    Printer::moveToReal(0,0,Printer::bedCompensationProbeHeight,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
    Printer::runZProbe(false,true);
	
    Printer::bedBadnessScore *= 100;
	Printer::bedBadnessScore /= probeCount;

    return minSeenZ;
}

/**
 * Generates a print surface mesh. Returns 0 for success, otherwise failure (probably memory allocation failure.)
 */
char Printer::buildBedMesh() {
    //1: clear old mesh if it exists
    Printer::freeBedMesh();

    int meshBufferSize = sizeof(struct meshTriangle)*Printer::meshWidth*Printer::meshWidth*2;
    Com::printFLN(Com::tMeshSize, meshBufferSize);

    Printer::mesh = (struct meshTriangle*)malloc(meshBufferSize);

    if (!Printer::mesh) {
        //Report mesh failed (error code 1)
        Com::printFLN(Com::tMeshFailedRAM, 1);
        return 1;
    }

    // Com::printNumber(sizeof(meshTriangle));
    // Com::println();
    // Com::printNumber(Printer::meshWidth);
    // Com::println();
    // Com::printNumber((int)meshBufferSize);
    // Com::println();
    // Com::printNumber((int)mesh);
    // Com::println();

    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();

    //1st attempt at building the mesh:
    UI_STATUS_UPD(UI_TEXT_BED_PROBE);
    float minSeen = buildBedMesh0();
    if (minSeen<0 || minSeen>BEDCOMPENSATION_ACCEPTABLE_ZERO_DEVIATION) {
        //The mesh was too far offset so attempt again:
        UI_STATUS_UPD(UI_TEXT_BED_PROBE_RETRY);
        Com::printFLN(Com::tMeshOffsetTooLarge, minSeen);


        //Update zLength of the printer to account for the lowest probed point.
        Printer::updateCurrentPosition();
        Printer::zLength -= minSeen;
        //Over compensate by a tiny amount: (always positive, to try to ensure we don't get a negative outcome)
        Printer::zLength += BEDCOMPENSATION_ZPROBE_ERROR/2.0;

        Printer::updateDerivedParameter();
        Printer::homeAxis(true,true,true);
        Printer::updateCurrentPosition();

/*
        //Make sure we don't appear fully dead:
        GCode::readFromSerial();
        Commands::checkForPeriodicalActions();
*/


        float minSeen = buildBedMesh0();
        if (minSeen<0 || minSeen>BEDCOMPENSATION_ACCEPTABLE_ZERO_DEVIATION) {
            Com::printFLN(Com::tMeshFailedOffset);
            //no need to hang onto useless data:
            Printer::freeBedMesh();
            return 2;
        }
    }

    Com::printFLN(Com::tMesh);
    for (int i = 0; i<(Printer::meshWidth*Printer::meshWidth)*2; i++) {
        Com::printF(Com::tMeshTriangle, i);
        Com::printF(Com::tMeshA, Printer::mesh[i].A);
        Com::printF(Com::tMeshB, Printer::mesh[i].B);
        Com::printF(Com::tMeshC, Printer::mesh[i].C);
        Com::printFLN(Com::tMeshD, Printer::mesh[i].D);
    }

    UI_CLEAR_STATUS

    //Success!
    return 0;
}

/**
 * Determines the index into the mesh array that will give us the triangle relating to the X and Y coords given.
 */
int getTriangleIndex(float x, float y) {
    //Calculate X and Y after offset and unitisation:
    float ouX = (x-Printer::meshOffsetX)/Printer::meshSpacing;
    float ouY = (y-Printer::meshOffsetY)/Printer::meshSpacing;
    
    //Determine if we're in an A or B triangle:
    char isB = (fmod(ouX,1)+fmod(ouY,1) > 1 ? 1 : 0);

    //Find the square coords we're in:
    int sqX = (int)ouX;
    int sqY = (int)ouY;

    return sqX*2 + sqY*2*Printer::meshWidth + isB;
}

/**
 * Gets the Z offset of the bed at (x,y)
 * Expensive. Don't piss around.
 */
float getBedZat(float x, float y) {
    //Which mesh triangle does this point reside in?
    int tIdx = getTriangleIndex(x,y);

    //4) Calculate Z from the plane equation of this triangle
    struct meshTriangle plane = Printer::mesh[tIdx];

    //woop maths.
    return (plane.D - plane.A * x - plane.B * y) / plane.C;
}

/**
 * Looks up what the Z should be at given a target Z and the bed Z.
 */
float mappedZprecomp(float bedZ, float z) {
    if (Printer::correctedByZ>BEDCOMPENSATION_NEVERCORRECT_HEIGHT) {
        //Same distortion at all Z points.
        return bedZ+z;
    } else if (z > Printer::correctedByZ) {

        //Target Z is above the threshold so everything's good already!
        return z;

    } else {

        //In the distortion zone, and distortion correction is enabled:
        return bedZ * (1-z/Printer::correctedByZ) + z;

    }
}

/**
 * Looks up what the Z should be at an X and Y position and taking account of the allowable distortion.
 */
float mappedZ(float x, float y, float z) {
    mappedZprecomp(getBedZat(x,y), z);
}

/**
 * Actually does a G0/G1 command.
 * lifted straight from Commands.cpp
 */
void commitMoveGCode(GCode *com) {
    if(com->hasS())
                Printer::setNoDestinationCheck(com->S!=0);
            if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
#if NONLINEAR_SYSTEM
                PrintLine::queueDeltaMove(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
                PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS,true);
#endif
}

/*
 * We do not apply the target E multiplier to the line immediately but instead each move does the average e mult of the start and end points.
 */
float previousPointEMult = 1.0;


/**
 * Given a move GCode this will mangle it to have the corrected Z height and E-value.
 * target[XYZ] must always be absolute.
 * targetE must always be relative.
 */
void mangleMove(GCode *com, float targetX, float targetY, float targetZ, float targetE) {

    //First up, put the X and Y back into the GCode:
    if (Printer::relativeCoordinateMode) {
        com->setX(targetX-Printer::currentPosition[X_AXIS]);
        com->setY(targetY-Printer::currentPosition[Y_AXIS]);
    } else {
        //Absolute mode:
        com->setX(targetX);
        com->setY(targetY);
    }

    //mangle the Z coord
    float zHere = getBedZat(targetX, targetY);
    float moveZ = mappedZprecomp(zHere, targetZ);

    if (Printer::relativeCoordinateMode) {
        com->setZ(moveZ-Printer::currentPositionZgCode);
    } else {
        com->setZ(moveZ);
    }
    

    if (com->hasE()) {
        //This is an extrusion move not a travel so E needs recalculation.
        float moveEMult;

        if (Printer::correctedByZ>BEDCOMPENSATION_NEVERCORRECT_HEIGHT) {
            moveEMult = 1.0;
        } else {
            float gotoEMult = (Printer::correctedByZ - zHere) / Printer::correctedByZ;
            moveEMult = (gotoEMult+previousPointEMult)/2;
            previousPointEMult = gotoEMult;
        }

        if (Printer::relativeExtruderCoordinateMode) {
            com->E = targetE * moveEMult;
        } else {
            com->E = targetE * moveEMult + Printer::Eposition;
			Printer::Eposition = com->E;
        }

        
    }

    #ifdef BEDCOMPENSATION_DEBUG
    sdebug("post-mangle: ");
    com->printCommand();
    #endif
    commitMoveGCode(com);
    Printer::updateCurrentPosition();
    Printer::currentPositionZgCode = targetZ;
}

/**
 * Sets the Eposition to be the value that the GCode asked for rather than the mangled version we might have used.
 * (we may have extruded more or less than the gcode requested due to modded layer heights.)
 * 
 * This must be done only AFTER all submoves have been issued.
 */
void fixupE(GCode *com, float originalGcodeE) {
    if (com->hasE() && !Printer::relativeExtruderCoordinateMode) {
        Printer::currentPositionSteps[E_AXIS] = Printer::convertToMM(originalGcodeE)*Printer::axisStepsPerMM[E_AXIS];
        Printer::Eposition = originalGcodeE;
        //As we've 'reset' our E meter to the value expected the next absolute move E GCode can carry on from where it expected.
    }
}

/**
 * Does a G0/G1 command taking into account the bed compensation mesh.
 * It does this by re-writing the code and then performing the action as it would have happened.
 */
void Printer::doMoveCommand(GCode *com) {

    #ifdef BEDCOMPENSATION_DEBUG
    sdebug("Current position: ");
    Commands::printCurrentPosition();
    sdebug("GCode move: ");
    com->printCommand();

    #endif

    //convinience definitions
    //Start coordinate (absolute)
    #define x1 (currentPosition[X_AXIS])
    #define y1 (currentPosition[Y_AXIS])
    #define z1 (Printer::currentPositionZgCode)
    #define e1 (Printer::Eposition)

    //End absolute coordinate:
    float x2, y2, z2, e2; /* e2 is relative, others absolute! */
    

    //In any case we need to know the exact target Z height at the end of the move:
    if (com->hasZ()) {
        z2 = com->Z;
        if (Printer::relativeCoordinateMode) z2 += z1;
    } else {
        z2 = z1;
    }
    
    //1) SUPER fast path (are Z above our distortion zone?)
    /**
     * Superfastpath: If our move starts and finishes above the correction line then we can just dispatch immediately.
                      OR if the move is E only. (like a retract/restart)
     */
    if ((z1 > Printer::correctedByZ and z2 > Printer::correctedByZ) or com->hasNoXYZ()) {
        //JFDI:
        commitMoveGCode(com);
        Printer::updateCurrentPosition();
        if (com->hasZ()) Printer::currentPositionZgCode = com->Z;
        fixupE(com, com->E);
        return;
    }

    //2) Do some preliminaries needed for the fastpath and slowpath.
    if (com->hasX()) {
        x2 = com->X;
        if (Printer::relativeCoordinateMode) x2 += x1;
    } else {
        x2 = x1;
    }
    if (com->hasY()) {
        y2 = com->Y;
        if (Printer::relativeCoordinateMode) y2 += y1;
    } else {
        y2 = y1;
    }

    float gCodeE = 0;
    if (com->hasE()) {
        //This is used to 'catch up' the printer's E position after the move.

        //e2 is always relative!
        gCodeE = e2 = com->E;

        if (!Printer::relativeExtruderCoordinateMode) {
            e2 -= e1;
        }
    } else {
        e2 = e1;
    }

    //Yes yes I know this can be done faster if we're in relative mode and I still don't care.
    float totalDistance = sqrt(sqr(x2-x1)+sqr(y2-y1));

    //3) fast path (is the move shorter than our short-move threshold?)
    /*
     * Fastpath: Is the move shorter than our fastpath limit?
     */
    if (totalDistance<BEDCOMPENSATION_FASTPATH_MAXLENGTH) {
        //Fastpath onlypath... at the moment.
        mangleMove(com, x2, y2, z2, e2);
    } else {
        //4) slowpath (split move into smaller moves.)

        //Establish some values that only need to be calculated once per full line:

        //The M and C of the move's y=mx+c line equation.
        float overallGradient = (y2-y1)/(x2-x1);
        float moveC = 0.0;
        if (fabs(overallGradient)<INFINITY) {
            //If the gradient is non-infinite then calculate the Y-intercept.
            moveC = y1-overallGradient*x1;
        }

        bdebug("Planned line m:",overallGradient);
        bdebug("Planned line c:",moveC);
        bdebug("totalDistance:",totalDistance);

        char goesRight = x2>x1;
        char goesUp = y2>y1;

        float totalComplete = 0.0;

        #define totalRelativeE e2

        float totalRelativeZ = z2-z1;

        float zStart = z1;

        //make sure we catch runnaway loops:
        //FIXME: remove or set test VERY high when working. 
        char segments = 0;

        // The loop condition is embedded within:
        /**
         * The purpose of this loop is to split the current requested line into sublines.
         * Lines are split whenever the line crosses a mesh edge. (considering only X and Y coordinates)
         *
         * This is determined by calculating the distance to each possible mesh edge that could be crossed next and chosing the closest, then going there.
         * If the target point was closest, we go there instead and then finish.
         */
        while (1) {
            if (segments++>100) {
                sdebug("Too many segments in line. HALTING FOR DEBUG.\n");
                Printer::kill(0);
                while (1);
            }

            float positionInBoxX = fmod(x1, Printer::meshSpacing);
            if (positionInBoxX < 0) positionInBoxX += Printer::meshSpacing;
            float positionInBoxY = fmod(y1, Printer::meshSpacing);
            if (positionInBoxY < 0) positionInBoxY += Printer::meshSpacing;

            float inboxX = x1 - positionInBoxX;
            float inboxY = y1 - positionInBoxY;

            float distanceToX, distanceToY;
            float nextCrossX, nextCrossY;
            if (goesRight) {
                distanceToX = Printer::meshSpacing - positionInBoxX;
                nextCrossX = inboxX + Printer::meshSpacing;
            } else {
                distanceToX = -positionInBoxX;
                nextCrossX = inboxX;
            }

            if (goesUp) {
                distanceToY = Printer::meshSpacing - positionInBoxY;
                nextCrossY = inboxY + Printer::meshSpacing;
            } else {
                distanceToY = -positionInBoxY;
                nextCrossY = inboxY;
            }

            //1) Where does it next cross a virtical edge? (edges parallel to the Y axis)
            //    the Y position of this crossing
            float nextCrossX_y;
            //    the distance in the Y axis to thie crossing
            float nextCrossX_y_dst;
            //the square of the distance to this crossing point.
            float nextCrossXdstS;

            nextCrossX_y = overallGradient * nextCrossX + moveC;
            nextCrossX_y_dst = nextCrossX_y-y1;
            nextCrossXdstS = sqr(distanceToX) + sqr(nextCrossX_y_dst);
            if (nextCrossXdstS<=0.01) nextCrossXdstS = INFINITY;
            //bdebug("X-crossing is away: ",sqrt(nextCrossXdstS));
            //bdebug("X x: ",nextCrossX);
            //bdebug("X y: ",nextCrossX_y);

            //2) Horizontal edges (parallel to X axis)
            float nextCrossY_x, nextCrossY_x_dst, nextCrossYdstS;
            if (fabs(overallGradient) < INFINITY) {
                //normal gradient, so calculate X:
                nextCrossY_x = (nextCrossY-moveC)/overallGradient;
            } else {
                //infinite gradient means that the X coord is unchanged.
                nextCrossY_x = x1;
            }
            nextCrossY_x_dst = nextCrossY_x - x1;
            nextCrossYdstS = sqr(distanceToY) + sqr(nextCrossY_x_dst);
            if (nextCrossYdstS<=0.01) nextCrossYdstS = INFINITY;
            //bdebug("Y-crossing is away: ",sqrt(nextCrossYdstS));
            //bdebug("Y x: ",nextCrossY_x);
            //bdebug("Y y: ",nextCrossY);

            //3) Diagonal edges (See relevel.py (the reference for this whole idea), the reasoning behind this code is 'fun'...)
            float nextCrossD_x, nextCrossD_y, ncd_dx, ncd_dy, diagYintercept, nextCrossDdstS;
            char steep, goesPositive;
            if (overallGradient != -1) {
                steep = abs(overallGradient)>1;

                if (steep) {
                    goesPositive = goesUp;
                } else {
                    goesPositive = goesRight;
                }

                diagYintercept = inboxY+inboxX;
                char inB = (positionInBoxX + positionInBoxY) > Printer::meshSpacing;

                if (goesPositive) {
                    diagYintercept = diagYintercept + Printer::meshSpacing;
                }

                if (inB) {
                    diagYintercept = diagYintercept + Printer::meshSpacing;
                }

                nextCrossD_x = (diagYintercept-moveC) / (overallGradient + 1);
                nextCrossD_y = -1 * nextCrossD_x + diagYintercept;
                ncd_dx = (nextCrossD_x - x1);
                ncd_dy = (nextCrossD_y - y1);

                if (((ncd_dx<0) == goesRight) || ((ncd_dy<0) == goesUp)) {
                    nextCrossDdstS = INFINITY;
                } else {
                    nextCrossDdstS = sqr(ncd_dx) + sqr(ncd_dy);
                }

                if (nextCrossDdstS<=0.01) nextCrossDdstS = INFINITY;
                //bdebug("D-crossing is away: ",sqrt(nextCrossDdstS));
                //bdebug("D x: ",nextCrossD_x);
                //bdebug("D y: ",nextCrossD_y);

            } else {
                nextCrossDdstS = INFINITY;
            }

            float targetDstS = sqr(x2-x1)+sqr(y2-y1);
            //bdebug("target is away: ",sqrt(targetDstS));

            //ouch...
            float closest = fmin(nextCrossXdstS, fmin(nextCrossYdstS, fmin(nextCrossDdstS, targetDstS)));

            char done = 0;
            float moveToX, moveToY;
            if (closest == targetDstS) {
                moveToX = x2;
                moveToY = y2;
                done = 1;
                //sdebug("move to target\n");
            } else if (closest == nextCrossXdstS) {
                moveToX = nextCrossX;
                moveToY = nextCrossX_y;
                //sdebug("move to X\n");
            } else if (closest == nextCrossYdstS) {
                moveToX = nextCrossY_x;
                moveToY = nextCrossY;
                //sdebug("move to Y\n");
            } else if (closest == nextCrossDdstS) {
                moveToX = nextCrossD_x;
                moveToY = nextCrossD_y;
                //sdebug("move to D\n");
            } else {
                //none matched...? Goto target.
                moveToX = x2;
                moveToY = y2;
                done = 1;
                //sdebug("No targets were closest!? Going to target.\n");
            }

            bdebug("x->",moveToX);
            bdebug("y->",moveToY);


            totalComplete += sqrt(closest);
            float fractionComplete = totalComplete / totalDistance;

            //eMove is relative!
            float eMove = totalRelativeE * fractionComplete;
            bdebug("eMove->",moveToY);
            //zMove is absolute.
            float zMove = zStart + totalRelativeZ * fractionComplete;

            mangleMove(com, moveToX, moveToY, zMove, eMove);

            if (done) {
                break;
            }
        }
    }

    //Now 'correct' the E offset to show we've moved as much as the original unmangled GCode intended: (if we're in absolute E moves)
    fixupE(com, gCodeE);
}

#endif


