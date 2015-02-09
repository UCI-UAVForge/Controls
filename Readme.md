# UCI UAVForge Controls
Firmware for the UAVForge vehicle autopilots. For the time being, "Quad"
is the only available project.


## Platforms
### Quad
Firmware for the UAVForge quad copter's Ardupilot APM 2.5 autopilot. Current
feature set is limited to attitude and rotation rate PID control laws. Future
features include velocity PID control and a communications bus for to
facilitate communication with other microcontrollers on the UAV.

#### Telemetry Visualizer
When compiled with the `TELEM` proprocessor flag defined, the quad firmware
will output telemetry data over the `hal.console` serial port when the quad
is throttled up. The ArdupilotTelemetryVisualizer folder contains a simple
WinForms application to read that data.

Note that the current version is quite fragile. For the telemetry visualizer
work correctly, the quad should be powered up and allowed to boot fully before
running the telemetry visualizer. (The quad is finished booting when the red
LED turns on).

### Fixed Wing
Future plans include developing an autopilot for fixed wing aircraft.

### Tilt Rotor
The primary purpose of this project. We are currently developing the a
tilt-rotor UAV airframe. Once the design is finalized, we will begin work
on the autopilot.


## Contact
If you use our code, or if you're intersted in this project, we want to hear
from you! Drop us a line at jason-watkins@outlook.com or chrisprijic@gmail.com.


## Licensing
All UCI UAVForge Controls code is licensed under version 3 of the
[GPL](http://www.gnu.org/copyleft/gpl.html). 

You can redistribute this software and / or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.