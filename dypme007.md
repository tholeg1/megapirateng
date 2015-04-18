# Introduction #

DYP-ME007(v1 or v2) Ultrasonic Wave Detector Ranging Module Distance Sensor

# Details #
  * Probe Main Technical Parameters
  * Center resonant frequency: **40kHz ± 2kHz**
  * Static capacitance: **3300P ± 300P**
  * Resonant impedance: **120Ω ± 20Ω**
  * Frequency bandwidth (-3dB): **Δf-3dB ≥ 2kHz**
  * Operation voltage: **300 ~ 500VP-P**
  * Limit voltage ≤ **1000VP-P**
  * Transmitting beam angle: **6O degrees**
  * Operation temperature: **-40 ~ +80 ℃**
  * Protection class: **IP65**
# Module Performance #
  * Voltage: **DC5V**;
  * Static current: Less than **2mA**;
  * Output signal: Electric frequency signal, high level **5V**, low level **0V**;
  * Sensor angle: **Not more than 15 degrees**;
  * Detection distance: **2cm-500cm** _(Actually up to 2.5m)_
  * High precision: **Up to 0.3cm**
# How it's work #
![http://megapirateng.googlecode.com/svn/images/me007_diagram.gif](http://megapirateng.googlecode.com/svn/images/me007_diagram.gif)
  1. Send 12us pulse to pin Trig, to start measurement
  1. The ultra sonic module will automatically send eight 40khz square waves, automatically detects whether there is a reflect signal
  1. When there is an reflect signal back, the Echo pin will output a high level, the duration of the high-level signal is the time (Techo) from untral sonic launch to return.
**As a result, the Measured distance = (Techo `*` (Sound speed (340M/S))) / 2;**

_(Suggested measure period should >60ms to avoid the wrong echo signal)_
# Board photo #
![http://megapirateng.googlecode.com/svn/images/me007v1.jpg](http://megapirateng.googlecode.com/svn/images/me007v1.jpg)