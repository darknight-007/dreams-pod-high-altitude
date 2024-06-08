# High-Altitude Balloon Payload with Attitude Control and Predictive Scheduling for Aerobiological Sampling and Environmental Monitoring

**Jnaneshwar Das, School of Earth and Space Exploration**

**Other contributors:**  
*EarthPod sensor probe* – Cole Bauer, Katrina Davis, Darwin Mick, Desmond Hanaan, Elizabeth Trembath-Reichert  
*Balloon payload considerations* – Scott Clemens, Anyell Mata, Tom Sharp

## Introduction

We introduce a lightweight, low SWaP (< 1.5 kg) high-altitude balloon payload for aerobiological sampling. The innovation incorporates the latest drone flight control technologies and custom environmental sensing hardware. A ground-tested prototype for the payload has been designed.

## Description

The main body, inspired by the CubeSat design format and quadrotor drone layout, consists of multiple levels interconnected with load-bearing aluminum spacers. 

- **Level 6**: Houses a Pixhawk flight controller running the PX4 flight stack, and a GPS and compass module.
- **Level 5**: Accommodates an aerobiological sampling payload capable of collecting air samples and storing them in filtrates, triggered by onboard compute for adaptive sampling.
- **Level 4**: Features the EarthPod sensor probe, which conducts various atmospheric measurements, including relative humidity, temperature, wind speed and direction, and light intensity/flux, and is capable of decision-making using an onboard Arduino Feather.
- **Level 3**: Contains two reaction wheels for stabilizing yaw angle (heading) to point in a desired direction, such as imaging specific ground features with a pitch-gimbaled multi-spectral camera and spectrometer imaging payload with onboard compute.
- **Level 2**: Houses two 4S lithium-polymer batteries to power the system's various components.
- **Level 1 (Bottom)**: Contains the imaging system and yaw stabilization of the tower and pitch stabilization of the gimbal mechanism managed by the Pixhawk flight controller on level 6. Also houses a radio telemetry link and antenna, connected to the flight controller on level 6. The payload is suspended using a load-bearing 360-degree swivel.

![image](https://github.com/darknight-007/dreams-pod-high-altitude/assets/3958994/82b1199c-2163-4a96-8eea-ca734cfbffe7)

![image](https://github.com/darknight-007/dreams-pod-high-altitude/assets/3958994/3326ffeb-d4bd-4b31-a082-8ec8b4db836c)

![image](https://github.com/darknight-007/dreams-pod-high-altitude/assets/3958994/ee785b45-6aeb-4c6c-bdc6-8917d78ce8d2)

In some manifestations, solar panels can be added to one or more sides of the payload tower, and the yaw stabilization system can ensure the panels are on average directed at the sun. Yaw stabilization can also enable imaging of the sun with a top-mounted camera, or tracking features such as solar eclipse shadows. The payload can also be used as a ground-based sensor probe, in a sensor network format, especially where long-term autonomy and attitude stabilization are desired. All decision-making happens onboard the payload, with computations split between flight controller (guidance, navigation, and control), EarthPod sensor probe (atmospheric and aerobiological sampling), and Intel Atom or ARM compute to carry out image processing tasks. A bank of lithium polymer batteries power the system and can be recharged with actively oriented solar panels that can be reoriented by the yaw stabilization system.

![image](https://github.com/darknight-007/dreams-pod-high-altitude/assets/3958994/ffbbd9b4-0b26-4eef-856f-b4dff0201f8b)

## Demo Video

Demo video, sun-tracking during partial solar eclipse in Phoenix:  
[![image](https://github.com/darknight-007/dreams-pod-high-altitude/assets/3958994/f91c6c9b-8ab2-41e5-bf43-cf0af7ba206e)](https://github.com/darknight-007/dreams-pod-high-altitude/assets/3958994/a45d4cd9-dda4-4b37-beda-d7d8ff1691a0){:target="_blank"}

Video of sun-tracking experiment at ground level during April 8 partial solar eclipse in Arizona:  
[Watch on YouTube](https://www.youtube.com/shorts/dlAszII4H6c){:target="_blank"}

## Tested Environment

Tested using ROS2 Humble, on Ubuntu 22.04 running on Hardkernel Odroid M1, required ROS packages, mavros, mavros-extras.

## Funding Acknowledgements

NASA Flight Opportunities Tech Flights (Lunar Lander ExoCam), NSF CNS 152161, Behar endowment funds, Earth Innovation Hub Corp. (501c3).

