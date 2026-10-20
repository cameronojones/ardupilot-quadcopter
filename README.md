# ArduPilot Based Quadcopter

A six-person engineering project at Durham University combining quadcopter design, assembly, calibration and autonomous flight testing.

My main contribution focused on Lua mission development and validation using ArduPilot and Mission Planner SITL, alongside practical assembly and integration.

[Read the project report](./ArduPilot_Quadcopter_Report.pdf)

[View the autonomous mission script](./Scripts/closed_loop.lua)

## Project Overview

The team developed an ArduPilot-based quadcopter incorporating a flight controller, radio receiver, positioning sensors, telemetry and an FPV camera.

The project combined mechanical and electrical integration with flight-controller configuration, autonomous mission development and evaluation of potential industrial inspection applications.

## My Contribution

- Developed and validated a Lua mission for take-off, a closed square route, return to the launch position and landing.
- Troubleshot missing Cygwin dependencies and OneDrive-related SITL directory conflicts.
- Corrected firmware-dependent Lua altitude handling.
- Implemented switch activation, abort behaviour, mission timeouts and release of scripted control following an external flight-mode change.
- Evaluated return-position error across six desktop SITL trials.
- Participated in practical assembly, calibration, troubleshooting and flight testing.

The wider airframe design, camera integration, computer-vision work and business case were team activities. Detailed contributions are identified in the report.

## Autonomous Mission

The script commands ArduPilot's existing position controller through a state machine.

| Setting | Value |
|---|---|
| Commanded altitude | 3 m above home |
| Square dimensions | 5 m × 5 m |
| Nominal horizontal route length | 20 m |
| Waypoint acceptance radius | 0.8 m |
| Script update interval | 200 ms |
| Automatic arming | Disabled by default |

The vehicle is armed manually. A LOW-to-HIGH transition on the configured Scripting1 switch starts the mission.

Moving the switch LOW during the active mission requests LAND. An external change from GUIDED mode causes the script to stop issuing mission commands.

## Validation Results

The project report records six desktop SITL trials with final horizontal return errors of:

0.61, 0.57, 0.64, 0.59, 0.63 and 0.55 m.

- Mean return error: 0.598 m
- Sample standard deviation: 0.035 m
- All six results were within the configured 0.8 m acceptance radius

These measurements describe horizontal error when the final waypoint was accepted, immediately before LAND was requested. They do not measure touchdown accuracy or maximum deviation along the route.

A separate illustrative laptop simulation and a physical flight are documented in the report. The physical flight demonstrated wind disturbance, subsequent route recovery and landing; equivalent quantitative physical-flight accuracy was not established.

## Repository Contents

- `ArduPilot_Quadcopter_Report.pdf` — complete project report
- `main.tex` — LaTeX report source
- `Figures/` — report images and diagrams
- `Scripts/closed_loop.lua` — autonomous square-route mission

## Limitations and Further Development

Waypoint acceptance currently depends on horizontal distance, without a dwell-time or low-velocity requirement.

Further improvements include explicit validation of all four generated waypoints, confirmation of LAND mode requests, and repeated physical-flight measurements using exported position logs.

Lua API compatibility depends on the ArduCopter firmware version. The script is included as academic project source, with validation evidence described in the report.

## Project Context

Durham University undergraduate engineering practical project, June 2026.
