#include "Instruments.h"

namespace Quad
{
    Instruments::Instruments(const AP_HAL::HAL& hal)
        : hal(hal)
    {
        // Turn off Barometer to avoid bus collisions
        hal.gpio->pinMode(40, GPIO_OUTPUT);
        hal.gpio->write(40, 1);

        // Turn on MPU6050 - quad must be kept still as gyros will calibrate
        this->ins.init(AP_InertialSensor::COLD_START,
            AP_InertialSensor::RATE_100HZ,
            NULL);

        // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
        hal.scheduler->suspend_timer_procs();  // stop bus collisions
        ins.dmp_init();
        hal.scheduler->resume_timer_procs();
    }

    void Instruments::Update()
    {
        // Wait until new orientation data (normally 5ms max)
        while (ins.num_samples_available() == 0);
        ins.update();
    }

    Vector3f Instruments::GetGyro()
    {
        Update();
        return ins.get_gyro();
    }


    Vector3f Instruments::GetOrientation()
    {
        Update();
        float roll;
        float pitch;
        float yaw;
        ins.quaternion.to_euler(&roll, &pitch, &yaw);
        roll = ToDeg(roll);
        pitch = ToDeg(pitch);
        yaw = ToDeg(yaw);
        return Vector3f(roll, pitch, yaw);
    }
}
