from datetime import datetime, timezone
from pyorbital import astronomy
from pyorbital import planets
import time

# south tempe
latitude = 33.37557039913588
longitude = -111.91379195037831

while True:
    current_time = datetime.utcnow()
    current_time_local = current_time.replace(tzinfo=timezone.utc).astimezone(tz=None)
    sun_zenith_angle = astronomy.sun_zenith_angle(current_time, longitude, latitude)
    sun_altitude, sun_azimuth = astronomy.get_alt_az(current_time, longitude, latitude)
    moon = planets.Moon(current_time)

    rasc, decl, alt, azi = moon.topocentric_position(longitude, latitude)
    print(alt, azi)
    print("zenith angle, azimuth, sun_alt", sun_zenith_angle, sun_azimuth, sun_altitude)
    time.sleep(1)

