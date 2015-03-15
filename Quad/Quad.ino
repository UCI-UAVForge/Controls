// Copyright 2015 Jason Watkins
// This file is part of the UCI UAVForge Quad-copter Controls System.
//
// UCI UAVForge Quad-copter Controls System is free software : you can
// redistribute it and / or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// UCI UAVForge Quad-copter Controls System is distributed in the hope that it
// will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// UCI UAVForge Quad-copter Controls System. If not, see
// <http://www.gnu.org/licenses/>.

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>

// Application dependencies
// TODO: We probably don't need a lot of these dependencies, and removing them
//       may speed up compilation. Unfortunately, they are highly
//       interconnected, making it difficult to prune the list.
#include <GCS.h>
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_SerialManager.h>   // Serial manager library
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Baro_Glitch.h>     // Baro glitch protection library
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_NavEKF.h>
#include <AP_Mission.h>         // Mission command library
#include <AP_Rally.h>           // Rally point library
#include <AC_PID.h>             // PID library
#include <AC_HELI_PID.h>        // Heli specific Rate PID library
#include <AC_P.h>               // P library
#include <AC_AttitudeControl.h> // Attitude control library
#include <AC_AttitudeControl_Heli.h> // Attitude control library for traditional helicopter
#include <AC_PosControl.h>      // Position control library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>     		// ArduCopter waypoint navigation library
#include <AC_Circle.h>          // circle navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence.h>           // Arducopter Fence library
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library
#include <AP_Frsky_Telem.h>
#include <AP_LandingGear.h>     // Landing Gear library
#include <AP_Terrain.h>

#include <PID.h>


// Utility functions. Used primarily to keep control outputs within range.
float Map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Wrap180(float value)
{
    if (value < -180)
    {
        return value + 360;
    }
    if (value > 180)
    {
        return value - 360;
    }
    return value;
}

float Constrain(float value, float lower, float upper)
{
    if (value > upper)
    {
        return upper;
    }
    if (value < lower)
    {
        return lower;
    }
    return value;
}

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

//////////////////////////////////
//// INSTRUMENTS DECLARATIONS ////
//////////////////////////////////
AP_InertialSensor ins;
AP_Baro barometer;
AP_GPS gps;

// TODO: Compiler doesn't recognize HAL_COMPASS_PX4 flag,
//       but it compiles fine like this. Why?
//#if CONFIG_COMPASS == HAL_COMPASS_PX4
AP_Compass_PX4 compass;
//#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
//AP_Compass_VRBRAIN compass;
//#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
//AP_Compass_HMC5843 compass;
//#elif CONFIG_COMPASS == HAL_COMPASS_HIL
//AP_Compass_HIL compass;
//#elif CONFIG_COMPASS == HAL_COMPASS_AK8963
//AP_Compass_AK8963_MPU9250 compass;
//#else
//#error Unrecognized CONFIG_COMPASS setting
//#endif

// TODO: Analyze whether there is ever a situation where we can't use NavEKF.
//       Seems like NavEKF falls back to DCM anyway if there's a problem, so
//       we probably don't need to bother with preprocessor here.
//#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs = AP_AHRS_NavEKF(ins, barometer, gps);
//#else
//AP_AHRS_DCM ahrs = AP_AHRS_DCM(ins, barometer, gps);;
//#endif
//////////////////////////////////////
//// END INSTRUMENTS DECLARATIONS ////
//////////////////////////////////////

////////////////////////////
//// COMMS DECLARATIONS ////
////////////////////////////
uint8_t readBuffer[18];
uint16_t outputFlags;
size_t bytesInBuffer;

int Read();
void SendPing();
void SendStartInputs(uint16_t flags);
void SendScalar16(uint8_t type, uint16_t value);
void SendScalar32(uint8_t type, uint32_t value);
void SendScalarF(uint8_t type, float value);
void SendVector2(uint8_t type, Vector2i value);
void SendVector2(uint8_t type, Vector2ui value);
void SendVector2(uint8_t type, Vector2<int32_t> value);
void SendVector2(uint8_t type, Vector2<uint32_t> value);
void SendVector2(uint8_t type, Vector2f value);
void SendVector3(uint8_t type, Vector3i value);
void SendVector3(uint8_t type, Vector3ui value);
void SendVector3(uint8_t type, Vector3<int32_t> value);
void SendVector3(uint8_t type, Vector3<uint32_t> value);
void SendVector3(uint8_t type, Vector3f value);
void SendVector4(uint8_t type, int16_t w, int16_t x, int16_t y, int16_t z);
void SendVector4(uint8_t type, uint16_t w, uint16_t x, uint16_t y, uint16_t z);
uint16_t GetOutputFlags();
void HandlePacket();
uint8_t GetPayloadLength(int id);
bool ValidateChecksum(size_t len);
void SetChecksum(uint8_t* buffer, size_t len);
void Send(uint8_t* buffer, size_t len);
void ShiftBytes();

/////////////////////////
//// RC DECLARATIONS ////
/////////////////////////
uint16_t channels[8] = { 0 };
const uint16_t THR_MIN = 1100;
const uint16_t THR_MAX = 2000;
const uint16_t YAW_MIN = 1100;
const uint16_t YAW_MAX = 1900;
const uint16_t PIT_MIN = 1100;
const uint16_t PIT_MAX = 1900;
const uint16_t ROL_MIN = 1100;
const uint16_t ROL_MAX = 1900;
const uint16_t GAIN_MIN = 968;
const uint16_t GAIN_MAX = 2074;
/////////////////////////////
//// END RC DECLARATIONS ////
/////////////////////////////

////////////////////////////
//// MOTOR DECLARATIONS ////
////////////////////////////
const int FL = 2;
const int FR = 0;
const int BL = 1;
const int BR = 3;
////////////////////////////////
//// END MOTOR DECLARATIONS ////
////////////////////////////////

/////////////////////////////////////
//// RRC CONTROLLER DECLARATIONS ////
/////////////////////////////////////
PID rrcPitchPID;
PID rrcRollPID;
PID rrcYawPID;
/////////////////////////////////////////
//// END RRC CONTROLLER DECLARATIONS ////
/////////////////////////////////////////

//////////////////////////////////////////
//// ATTITUDE CONTROLLER DECLARATIONS ////
//////////////////////////////////////////
PID pitchPID;
PID rollPID;
PID yawPID;
//////////////////////////////////////////////
//// END ATTITUDE CONTROLLER DECLARATIONS ////
//////////////////////////////////////////////

void setup()
{
    ////////////////////
    //// COMMS INIT ////
    ////////////////////
    // Used to receive control input from the Arduino
    hal.uartD->begin(115200);
    // Used to output telemetry info
    hal.console->begin(115200);
    ////////////////////////
    //// END COMMS INIT ////
    ////////////////////////

    /////////////////////////
    //// INSTRUMENT INIT ////
    /////////////////////////
    // COLD_START assumes the vehicle is stationary on a level surface.
    ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
    compass.init();
    barometer.init();
    barometer.calibrate(); // Sets the vehicle's current altitude to 0.
    ahrs.init();
    ahrs.set_compass(&compass);
    /////////////////////////////
    //// END INSTRUMENT INIT ////
    /////////////////////////////

    // Enable motor outputs. Motors still won't turn until the arming switch
    // is activated.
    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);

    //////////////////
    //// RRC INIT ////
    //////////////////
    rrcPitchPID.kP(0.87);
    rrcPitchPID.kI(0.1);
    rrcPitchPID.imax(10);
    rrcPitchPID.kD(0.07);

    rrcRollPID.kP(0.87);
    rrcRollPID.kI(0.1);
    rrcRollPID.imax(10);
    rrcRollPID.kD(0.03);

    rrcYawPID.kP(2.0);
    // Disabled integral on yaw because it was causing the quad to over-correct
    //rrcYawPID.kI(0.1);
    //rrcYawPID.imax(10);
    //////////////////////
    //// END RRC INIT ////
    //////////////////////

    ///////////////////////
    //// ATTITUDE INIT ////
    ///////////////////////
    pitchPID.kP(4.5);

    rollPID.kP(4.5);

    yawPID.kP(1);
    ///////////////////////////
    //// END ATTITUDE INIT ////
    ///////////////////////////
}

void loop()
{
    static float yawHoldAngle = 0;

    // Used to filter barometer readings.
    static int lastTime;
    int now = hal.scheduler->micros();
    int dt = now - lastTime;

    // Track iterations and only send telemtry every 50 iterations.
    static uint8_t iterCounter;
    ++iterCounter;
    iterCounter %= 50;

    // Read comms input
    int result = Read();
    SendScalar32(0xFF, (uint32_t)result);
    // For testing purposes, just output everything all the time.
    //uint16_t flags = GetOutputFlags();
    uint16_t flags = 0xFFFF;

    ////////////////////////////
    //// INSTRUMENTS UPDATE ////
    ////////////////////////////
    // Wait until new orientation data (normally 5ms max)
    ins.wait_for_sample();
    ahrs.update();
    Vector3f gyro = ins.get_gyro();
    gyro = Vector3f(ToDeg(gyro.x), ToDeg(gyro.y), ToDeg(gyro.z));
    Vector3f attitude = Vector3f(ToDeg(ahrs.roll), ToDeg(ahrs.pitch), ToDeg(ahrs.yaw));

    barometer.update();
    // Filtered altitude data
    //static float lastAlt;
    //float alt = barometer.get_altitude();
    //float RC = 1 / (2 * PI*4);
    //alt = lastAlt +
    //    ((dt / (RC + dt)) *
    //    (alt - lastAlt));
    //Vector3f position = Vector3f(0, 0, alt);
    Location l;
    ahrs.get_position(l);
    Vector3f position(l.lat, l.lng, l.alt);
    ////////////////////////////////
    //// END INSTRUMENTS UPDATE ////
    ////////////////////////////////

    ///////////////////
    //// RC UPDATE ////
    ///////////////////
    uint16_t throttle = channels[2];
    Vector2f attitude2D = Vector2f(
        Map((float)channels[0], ROL_MIN, ROL_MAX, -45, 45),
        Map((float)channels[1], PIT_MIN, PIT_MAX, -45, 45));
    float rcYaw = Map((float)channels[3], YAW_MIN, YAW_MAX, -180, 180);
    ///////////////////////
    //// END RC UPDATE ////
    ///////////////////////

    //////////////////////
    //// CONTROL LOOP ////
    //////////////////////
    Vector3i outputs;
    Vector3f rateTargets;
    uint16_t fl = 0;
    uint16_t bl = 0;
    uint16_t fr = 0;
    uint16_t br = 0;
    Vector3f attError;
    // Do the magic
    if (throttle > THR_MIN + 100) // Throttle raised, turn on stablisation.
    {
        // Input targets
        Vector3f attitudeInputs = Vector3f(attitude2D.x, attitude2D.y, yawHoldAngle);

        // Stablise PIDS
        attError = attitudeInputs - attitude;
        rateTargets = Vector3f(
            Constrain(rollPID.get_pid(attError.x, 1), -250, 250),
            Constrain(pitchPID.get_pid(attError.y, 1), -250, 250),
            Constrain(yawPID.get_pid(Wrap180(attError.z), 1), -250, 250));
        if (abs(rcYaw) > 5) // Override yaw rate if there is a stick input.
        {
            rateTargets.z = rcYaw;
            yawHoldAngle = attitude.z;
        }

        // rate PIDS
        // TODO: Tune rate limits. At 500, a roll doublet was enough to
        //       destabalize and crash the quad. 250 is probably too low, but
        //       I'm leaving it there in the hope that it will make the quad
        //       nearly bulletproof.
        Vector3f rrcError = rateTargets - gyro;
        outputs = Vector3i(
            Constrain(rrcRollPID.get_pid(rrcError.x, 1), -250, 250),
            Constrain(rrcPitchPID.get_pid(rrcError.y, 1), -250, 250),
            Constrain(rrcYawPID.get_pid(rrcError.z, 1), -250, 250));

        // mix pid outputs and send to the motors.
        fl = throttle + outputs.x + outputs.y + outputs.z;
        bl = throttle + outputs.x - outputs.y - outputs.z;
        fr = throttle - outputs.x + outputs.y - outputs.z;
        br = throttle - outputs.x - outputs.y + outputs.z;
        hal.rcout->write(FL, fl);
        hal.rcout->write(BL, bl);
        hal.rcout->write(FR, fr);
        hal.rcout->write(BR, br);
    }
    else
    {
        hal.rcout->write(FL, 1000);
        hal.rcout->write(BL, 1000);
        hal.rcout->write(FR, 1000);
        hal.rcout->write(BR, 1000);

        // reset PID integrals whilst on the ground
        pitchPID.reset_I();
        rollPID.reset_I();

        rrcPitchPID.reset_I();
        rrcRollPID.reset_I();
        rrcYawPID.reset_I();
    }
    //////////////////////////
    //// END CONTROL LOOP ////
    //////////////////////////


    if (flags & 0x0001)
    {
        SendScalar32(0x01, now);
    }
    switch (iterCounter)
    {
    case 0:
        // This was originally the clock, but I find it's better to send
        // the clock on every iteration.
        break;
    case 1:
        if (flags & 0x0002)
        {
            SendVector4(0x01, channels[2], channels[0], channels[1], channels[3]);
        }
        break;
    case 3:
        if (flags & 0x0008)
        {
            SendVector4(0x03, fl, bl, fr, br);
        }
        break;
    case 5:
        if (flags & 0x0010)
        {
            SendVector3(0x01, gyro);
        }
        break;
    case 6:
        if (flags & 0x0020)
        {
            SendVector3(0x02, attitude);
        }
        break;
    case 7:
        //if (flags & 0x0040)
        //{
        //    SendScalarF(0x02, heading);
        //}
        break;
    case 10:
        if (flags & 0x0200)
        {
            SendVector3(0x05, position);
        }
        break;
    case 12:
        if (flags & 0x0800)
        {
            SendVector3(0x01, outputs);
        }
        break;
    case 13:
        if (flags & 0x1000)
        {
            SendVector3(0x08, rateTargets);
        }
        break;
        //case 20:
        //    SendScalar16(0x01, rc.GetGain());
        //    break;
    default:
        break;
    }
    lastTime = now;
}

void ShiftBytes()
{
    for (size_t i = 0; i < bytesInBuffer - 1; ++i)
    {
        readBuffer[i] = readBuffer[i + 1];
    }
    --bytesInBuffer;
}

int Read()
{
    static int counter = 0;
    int nextByte;
    if (bytesInBuffer)
    {
        nextByte = readBuffer[0];
    }
    else
    {
        nextByte = hal.uartD->read();
        if (nextByte != 0x9E)
        {
            return nextByte;
        }
        ++bytesInBuffer;
        readBuffer[0] = (uint8_t)nextByte;
    }

    nextByte = hal.uartD->read();
    for (int i = bytesInBuffer; nextByte != -1 && i < 12; ++i)
    {
        readBuffer[i] = (uint8_t)nextByte;
        ++bytesInBuffer;
        if ((uint8_t)bytesInBuffer == 12)
        {
            break;
        }
        nextByte = hal.uartD->read();
    }
    if (!ValidateChecksum(12))
    {
        ShiftBytes();
        return -4;
    }
    if ((uint8_t)bytesInBuffer < (12))
    {
        return -6;
    }

    HandlePacket();
    bytesInBuffer = 0;
    return ++counter;
}

void HandlePacket()
{
    int pt = (int)(readBuffer[0] & 0x0F);
    switch (pt)
    {
    case 1:
        break;
    case 2:
        break;
    case 3:
        outputFlags = *((uint16_t*)(readBuffer + 1));
        break;
    case 4:
        break;
    case 8:
        break;
    case 9:
        break;
    case 10:
        break;
    case 11:
        break;
    case 12:
        break;
    case 13:
        break;
    case 14:
        switch (*((uint8_t*)(readBuffer + 1)))
        {
        case 1:
            channels[2] = *((uint16_t*)(readBuffer + 2));
            channels[0] = *((uint16_t*)(readBuffer + 4));
            channels[1] = *((uint16_t*)(readBuffer + 6));
            channels[3] = *((uint16_t*)(readBuffer + 8));
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void SendPing()
{
    uint8_t buffer[3];
    buffer[0] = (uint8_t)1;
    SetChecksum(buffer, 1);
    Send(buffer, 3);

}

void SendStartInputs(uint16_t flags)
{
    uint8_t buffer[5];
    buffer[0] = (uint8_t)4 | 0x20;
    *((uint16_t*)(buffer + 1)) = flags;
    SetChecksum(buffer, 3);
    Send(buffer, 5);

}

void SendScalar16(uint8_t type, uint16_t value)
{
    uint8_t buffer[6];
    buffer[0] = (uint8_t)8 | 0x30;
    *((uint8_t*)(buffer + 1)) = type;
    *((uint16_t*)(buffer + 2)) = value;
    SetChecksum(buffer, 4);
    Send(buffer, 6);
}

void SendScalar32(uint8_t type, uint32_t value)
{
    uint8_t buffer[8];
    buffer[0] = (uint8_t)9 | 0x50;
    buffer[1] = type;
    *((uint32_t*)(buffer + 2)) = value;
    SetChecksum(buffer, 6);
    Send(buffer, 8);
}

void SendScalarF(uint8_t type, float value)
{
    SendScalar32(type, *((uint32_t*)&value));
}

void SendVector2(uint8_t type, Vector2i value)
{
    uint8_t buffer[8];
    buffer[0] = (uint8_t)10 | 0x50;
    *((uint8_t*)(buffer + 1)) = type;
    *((Vector2i*)(buffer + 2)) = value;
    SetChecksum(buffer, 8);
    Send(buffer, 8);
}

void SendVector2(uint8_t type, Vector2ui value)
{
    SendVector2(type, *((Vector2i*)&value));
}

void SendVector2(uint8_t type, Vector2<int32_t> value)
{
    uint8_t buffer[12];
    buffer[0] = (uint8_t)11 | 0x90;
    *((uint8_t*)(buffer + 1)) = type;
    *((Vector2<int32_t>*)(buffer + 2)) = value;
    SetChecksum(buffer, 10);
    Send(buffer, 12);
}

void SendVector2(uint8_t type, Vector2<uint32_t> value)
{
    SendVector2(type, *((Vector2<int32_t>*)&value));
}

void SendVector2(uint8_t type, Vector2f value)
{
    SendVector2(type, *((Vector2<int32_t>*)&value));
}

void SendVector3(uint8_t type, Vector3i value)
{
    uint8_t buffer[10];
    buffer[0] = (uint8_t)12 | 0x70;
    *((uint8_t*)(buffer + 1)) = type;
    *((Vector3i*)(buffer + 2)) = value;
    SetChecksum(buffer, 8);
    Send(buffer, 10);
}

void SendVector3(uint8_t type, Vector3ui value)
{
    SendVector3(type, *((Vector3i*)&value));
}

void SendVector3(uint8_t type, Vector3<int32_t> value)
{
    uint8_t buffer[16];
    buffer[0] = (uint8_t)13 | 0xD0;
    *((uint8_t*)(buffer + 1)) = type;
    *((Vector3<int32_t>*)(buffer + 2)) = value;
    SetChecksum(buffer, 14);
    Send(buffer, 16);
}

void SendVector3(uint8_t type, Vector3<uint32_t> value)
{
    SendVector3(type, *((Vector3<int32_t>*)&value));
}

void SendVector3(uint8_t type, Vector3f value)
{
    SendVector3(type, *((Vector3<int32_t>*)&value));
}

void SendVector4(uint8_t type, int16_t w, int16_t x, int16_t y, int16_t z)
{
    uint8_t buffer[12];
    buffer[0] = (uint8_t)14 | 0x90;
    *((uint8_t*)(buffer + 1)) = type;
    *((int16_t*)(buffer + 2)) = w;
    *((int16_t*)(buffer + 4)) = x;
    *((int16_t*)(buffer + 6)) = y;
    *((int16_t*)(buffer + 8)) = z;
    SetChecksum(buffer, 10);
    Send(buffer, 12);
}

void SendVector4(uint8_t type, uint16_t w, uint16_t x, uint16_t y, uint16_t z)
{
    uint8_t buffer[12];
    buffer[0] = (uint8_t)14 | 0x90;
    buffer[1] = type;
    *((uint16_t*)(buffer + 2)) = w;
    *((uint16_t*)(buffer + 4)) = x;
    *((uint16_t*)(buffer + 6)) = y;
    *((uint16_t*)(buffer + 8)) = z;
    SetChecksum(buffer, 10);
    Send(buffer, 12);
}

uint16_t GetOutputFlags()
{
    return outputFlags;
}

// Maps a packet type identifier to a packet length
// Returns: The total number of bytes in a packet.
uint8_t GetPayloadLength(int id)
{
    switch (id)
    {
    case 1:
    case 2:
        return 0;
    case 3:
    case 4:
        return 2;
    case 8:
        return 3;
    case 9:
    case 10:
        return 5;
    case 11:
    case 14:
        return 9;
    case 12:
        return 7;
    case 13:
        return 13;
    default:
        return (uint8_t)-1;
    }
}

// Appends a Fletcher-16 checksum to the specified packet. See http://en.wikipedia.org/wiki/Fletcher%27s_checksum
// for more information. The checksum is appended in check-byte format. i.e. recalculating the checksum over the
// entire packet will yield 0 for both sums.
void SetChecksum(uint8_t* data, size_t bytes)
{
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    uint8_t* ptr = data;
    size_t len = bytes;
    while (len--)
    {
        sum1 = (sum1 + *ptr++) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    uint8_t check1 = 255 - ((sum1 + sum2) % 255);
    uint8_t check2 = 255 - ((sum1 + check1) % 255);
    data[bytes] = check1;
    data[bytes + 1] = check2;
}

// Checks that the Fletcher-16 checksum of a packet is 0.
bool ValidateChecksum(size_t bytes)
{
    uint8_t* data = readBuffer;
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    for (size_t i = 0; i < bytes; i++)
    {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return sum1 == 0 && sum2 == 0;
}

void Send(uint8_t* buffer, size_t len)
{
    hal.console->write(buffer, len);
}

AP_HAL_MAIN();