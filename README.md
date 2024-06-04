Tested using ROS2 Humble, on Ubuntu 22.04 running on Hardkernel Odroid M1, required ROS packages, mavros, mavros-extras 

Demo video, sun-tracking during partial solar eclipse in Phoenix
https://github.com/darknight-007/dreams-pod-high-altitude/assets/3958994/a45d4cd9-dda4-4b37-beda-d7d8ff1691a0

**High-altitude balloon payload with attitude control and predictive scheduling, for aerobiological sampling and environmental monitoring 
**
_Jnaneshwar Das, School of Earth and Space Exploration 
_
**Other contributors: 
**EarthPod sensor probe – Cole bauer, Katrina davis, Darwin Mick, Desmond Hanaan, Elizabeth Trembath-Reichert
**Balloon payload considerations – Scott Clemens, Anyell Mata, Tom Sharp

High-altitude balloons, often called weather balloons, play a critical role in weather forecasting by collecting essential atmospheric data (WMO, 2016). These balloons, typically filled with hydrogen or helium, carry instruments called radiosondes aloft, which measure temperature, humidity, pressure, and wind speed and direction at various altitudes (Sargent et al., 2018). As the balloons ascend through the atmosphere, the radiosondes transmit real-time data to ground-based receiving stations. Meteorologists then incorporate this information into numerical weather prediction (NWP) models, which simulate the future state of the atmosphere (Bauer et al., 2015). The assimilation of high-altitude balloon observations into NWP models significantly improves forecast accuracy, particularly in the short to medium range (Lorenz and Willemsen, 2020). Moreover, the data collected by these balloons also contribute to the calibration and validation of satellite-based remote sensing observations, further enhancing the quality of weather forecasts (Ferrare et al., 2016). High-altitude balloons have also emerged as an essential tool for aerobiological sampling, allowing scientists to collect and analyze airborne microorganisms at various altitudes (Smith et al., 2017). These balloons are equipped with specialized devices, such as solid-state impactors and liquid impingers, which efficiently capture biological particles suspended in the atmosphere (Bryant et al., 2020). By launching these balloons to elevations of up to 30 kilometers, researchers can study the microbial composition and diversity within different atmospheric layers, providing valuable insights into the transport and survivability of microorganisms in extreme environments (DeLeon-Rodriguez et al., 2021). This information has significant implications for understanding the role of airborne microbes in cloud formation, precipitation, and biogeochemical cycling, as well as potential applications in detecting and monitoring the spread of pathogens (Kallmeyer, 2018). As aerobiological research continues to evolve, high-altitude balloons will remain a vital resource for expanding our knowledge of Earth's biosphere and its interactions with the atmosphere.

We introduce a lightweight, low SWaP (< 1.5 kg) high-altitude balloon payload for aerobiological sampling. The innovation incorporates the latest drone flight control technologies and custom environmental sensing hardware. A ground-tested prototype for the payload has been designed.

Description: The main body, inspired by the CubeSat design format and quadrotor drone layout, consists of multiple levels interconnected with load-bearing aluminum spacers. The top level (6) houses a Pixhawk flight controller running the PX4 flight stack, and a GPS and compass module. The subsequent level (5) can accommodate an aerobiological sampling payload capable of collecting air samples and storing them in filtrates, triggered by onboard compute for adaptive sampling. The fourth level features the EarthPod sensor probe, which conducts various atmospheric measurements, including relative humidity, temperature, wind speed and direction, and light intensity/flux, and is capable of decision-making using an onboard Arduino Feather. The third level contains two reaction wheels for stabilizing yaw angle (heading) to point in a desired direction, such as imaging specific ground features with a pitch-gimbaled multi-spectral camera and spectrometer imaging payload with onboard compute. The second level houses two 4S lithium-polymer batteries to power the system's various components. The imaging system is situated at the first level (or bottom) of the payload 'tower', with both yaw stabilization of the tower and pitch stabilization of the gimbal mechanism managed by the Pixhawk flight controller on level 6. Level 1 also houses a radio telemetry link and antenna, connected to the flight controller on level 6. The payload is suspended using load-bearing 360 degree swivel. 

Figure 1: System architecture depicting the primary components of the payload.


Figure 2: Front view of the payload 'tower', displaying each of the seven levels and their functions.




Figure 3: 3D rendering of the payload (left and center), and a manifestation where multiple payloads with potentially complementary functions can be linked together (right).  

In some manifestations, solar panels can be added to one or more sides of the payload tower, and the yaw stabilization system can ensure the panels are on average, directed at the sun. 

Yaw stabilization can also enable imaging of the sun with a top mounted camera, or tracking features such as solar eclipse shadows. 

The payload can also be used as a ground based sensor probe, in a sensor network format, especially where long term autonomy and attitude stabilization is desired. 

All decision-making happens onboard the payload, with computations split between flight controller (guidance, navigation, and control), EarthPod sensor probe (atmospheric and aerobiological sampling), and Intel Atom or ARM compute to carry out image processing tasks. A bank of lithium polymer batteries power the system, and can be recharged with actively oriented solar panels that can be reoriented by the yaw stabilization system. 






Figure 4: Campus field test of prototype system for yaw stabilization

Video of sun-tracking experient at ground level during April 8 partial solar eclipse in Arizona: https://www.youtube.com/shorts/dlAszII4H6c

Funding acknowledgements: 
NASA Flight Opportunities Tech Flights (Lunar Lander ExoCam), NSF CNS 152161, Behar endowment funds. 


References 

WMO (2016). Guide to Instruments and Methods of Observation. World Meteorological Organization, WMO-No. 8.
Sargent, M., et al. (2018). Advances in weather prediction technology: Radiosondes. Weather, 73(6), 186-191.
Bauer, P., et al. (2015). The quiet revolution of numerical weather prediction. Nature, 525(7567), 47-55.
Lorenz, T., and Willemsen, P. (2020). Assimilation of high-resolution radiosonde data in the ICON model: Impact on weather forecasts. Weather and Forecasting, 35(1), 381-395.
Ferrare, R. A., et al. (2016). Evaluation of Daytime Measurements of Aerosols and Water Vapor made by an Operational Raman Lidar over the Southern Great Plains. Journal of Geophysical Research: Atmospheres, 121(3), 1298-1318.
Smith, D. J., et al. (2017). High-Altitude Aerosol Sampling: Using Balloons to Investigate Aerosol-Cloud-Climate Interactions. Eos, 98.
Bryant, N., et al. (2020). Vertical Stratification of Microbes in the High-Altitude Atmosphere. Frontiers in Environmental Science, 8, 575762.
DeLeon-Rodriguez, N., et al. (2021). Microbiome of the upper troposphere and lower stratosphere: taxa variability and potential impacts on clouds and climate. Scientific Reports, 11(1), 5460.
Kallmeyer, J. (2018). Aerobiology: A frontier in microbial ecology. FEMS Microbiology Ecology, 94(6), fiy077.
