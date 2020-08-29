SpectralESP

This project uses the ESP32 and micropython to drive the AS7265x sensor assembly.

Derived from pySpectralTriad and AS7265X_sparkfun_micropython

Updated to use Python 3.8 on the workstation and Micropython 1.12 on the sensor driver ESP32.  Removed serial port support in favor of scraping data from ESP32 webserver.

Modify the boot.py to include your SSID and WPA2-PASSWORD.
Currently the IP address is hard-coded in the python client script.

Possible further development -

1. pass the sensor IP address from cli

2. Enhance sensor options usability

3. Add OLED display support to indicated IP address obtained.
