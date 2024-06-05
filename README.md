# High-Altitude Balloon Payload with Attitude Control and Predictive Scheduling for Aerobiological Sampling and Environmental Monitoring

**Jnaneshwar Das, School of Earth and Space Exploration**

**Other contributors:**  
*EarthPod sensor probe* – Cole Bauer, Katrina Davis, Darwin Mick, Desmond Hanaan, Elizabeth Trembath-Reichert  
*Balloon payload considerations* – Scott Clemens, Anyell Mata, Tom Sharp

## Introduction

We introduce a lightweight, low SWaP (< 1.5 kg) high-altitude balloon payload for aerobiological sampling. The innovation incorporates the latest drone flight control technologies and custom environmental sensing hardware. A ground-tested prototype for the payload has been designed.

High-altitude balloons, often called weather balloons, play a critical role in weather forecasting by collecting essential atmospheric data (WMO, 2016). These balloons, typically filled with hydrogen or helium, carry instruments called radiosondes aloft, which measure temperature, humidity, pressure, and wind speed and direction at various altitudes (Sargent et al., 2018). As the balloons ascend through the atmosphere, the radiosondes transmit real-time data to ground-based receiving stations. Meteorologists then incorporate this information into numerical weather prediction (NWP) models, which simulate the future state of the atmosphere (Bauer et al., 2015). The assimilation of high-altitude balloon observations into NWP models significantly improves forecast accuracy, particularly in the short to medium range (Lorenz and Willemsen, 2020). Moreover, the data collected by these balloons also contribute to the calibration and validation of satellite-based remote sensing observations, further enhancing the quality of weather forecasts (Ferrare et al., 2016).

High-altitude balloons have also emerged as an essential tool for aerobiological sampling, allowing scientists to collect and analyze airborne microorganisms at various altitudes (Smith et al., 2017). These balloons are equipped with specialized devices, such as solid-state impactors and liquid impingers, which efficiently capture biological particles suspended in the atmosphere (Bryant et al., 2020). By launching these balloons to elevations of up to 30 kilometers, researchers can study the microbial composition and diversity within different atmospheric layers, providing valuable insights into the transport and survivability of microorganisms in extreme environments (DeLeon-Rodriguez et al., 2021). This information has significant implications for understanding the role of airborne microbes in cloud formation, precipitation, and biogeochemical cycling, as well as potential applications in detecting and monitoring the spread of pathogens (Kallmeyer, 2018). As aerobiological research continues to evolve, high-altitude balloons will remain a vital resource for expanding our knowledge of Earth's biosphere and its interactions with the atmosphere.


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

## References

- WMO (2016). Guide to Instruments and Methods of Observation. World Meteorological Organization, WMO-No. 8.
- Sargent, M., et al. (2018). Advances in weather prediction technology: Radiosondes. Weather, 73(6), 186-191.
- Bauer, P., et al. (2015). The quiet revolution of numerical weather prediction. Nature, 525(7567), 47-55.
- Lorenz, T., and Willemsen, P. (2020). Assimilation of high-resolution radiosonde data in the ICON model: Impact on weather forecasts. Weather and Forecasting, 35(1), 381-395.
- Ferrare, R. A., et al. (2016). Evaluation of Daytime Measurements of Aerosols and Water Vapor made by an Operational Raman Lidar over the Southern Great Plains. Journal of Geophysical Research: Atmospheres, 121(3), 1298-1318.
- Smith, D. J., et al. (2017). High-Altitude Aerosol Sampling: Using Balloons to Investigate Aerosol-Cloud-Climate Interactions. Eos, 98.
- Bryant, N., et al. (2020). Vertical Stratification of Microbes in the High-Altitude Atmosphere. Frontiers in Environmental Science, 8, 575762.
- DeLeon-Rodriguez, N., et al. (2021). Microbiome of the upper troposphere and lower stratosphere: taxa variability and potential impacts on clouds and climate. Scientific Reports, 11(1), 5460.
- Kallmeyer, J. (2018). Aerobiology: A frontier in microbial ecology. FEMS Microbiology Ecology, 94(6), fiy077.
