# cw4_nmea2000

This is a ROS node, deigned for [Project11](https://github.com/CCOMJHC/project11) which translates NMEA messages to ROS diagnostics.
NMEA messages are read separately from a [Maretron USB100](https://www.maretron.com/products/usb100.php) and published to ```/ben/sensors/nmea_sentence```
This package then parses, labels and publishes them to ```/diagnostics``` as key value pairs inside of a diagnostic array.
