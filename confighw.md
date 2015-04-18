# Sensors #
## IMU ##
connect you IMU board to I2C bus on Arduino MEGA
## GPS ##
connect you GPS receiver to Serial2 port of Arduino GPS\_TX->RX2
## Sonar ##
connect you [DYP-ME007](dypme007.md) sonar:
| **Sonar** | **Arduino**      |
|:----------|:-----------------|
| VCC     | +5v or ESC BEC |
| GND     | GND            |
| Trig    | D9             |
| Echo    | D10            |

# RC #
you can define you RC set in /libraries/APM\_RC/APM\_RC.cpp, but default is MultiWii style:
| **RC**           | **Arduino** |
|:-----------------|:------------|
| CH1 (Roll)     | A9        |
| CH2 (Pitch)    | A10       |
| CH3 (Throttle) | A8        |
| CH4 (Yaw)      | A11       |
| CH5 (AUX1)     | A12       |

# Motor mappings #
![http://megapirateng.googlecode.com/svn/images/FRAMES_hexa_octa_y6.gif](http://megapirateng.googlecode.com/svn/images/FRAMES_hexa_octa_y6.gif)