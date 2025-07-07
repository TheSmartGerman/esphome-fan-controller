# ESPHome Solar Inverter Fan Controller (Deye SUN 12K)

**Based on this Projekt:**  https://github.com/patrickcollins12/esphome-fan-controller

This project describes how to build a quiet thermostat controlled fan for cooling your Solarinverter.

The software is ESPHome and Home Assistant. The hardware is an ESP32 with three 12v 140mm Computer Fan (4-Pin PWM) and a Temperature Sensor (DS18B20) and Small OLED Display.

<img src="images/hut_font.jpg" width=300>
<img src="images/hut_back.jpg" width=300>
<img src="images/pcb_w_disp_working.jpg" width=300>

## Partlist
 - 3x 140mm 4-PIN PWM PC FANs
 - 3x 140mm FAN Grill
 - 1x ESP32 (ESP32-C3 Super Mini)
 - 1x 0,96" OLED Display
 - 1x 12V -> 3,3V voltage Regulator
 - 1x DS18B20 OneWire Temperatur Sensor
 - 1x [3D Printed Cooling HUT](/3d_print/Deye12kTopCoolingV3.zip)

## Motivation
- Keep the Inverter more cool -> extend Lifetime
- Prevent extern / internal FAns to stay off -> reduce noise level of the Inverter (depends also on FW of the inverter)


## Features
The main features are:

- the **fan dynamically adjusts** it's speed based on the temperature sensor using a Process Control mechanism called PID
- **adjustable target temperature**. I currently target 35°C. 
- uses ESP32's Wifi to connect to Home Assistant for control and reporting
- the ESP32 is standalone and so the cooling function will continue to operate without Wifi. Doesn't need HomeAssistant or Wifi to operate. Wifi is only needed for setup, manual control and reporting.
- **no screen** is needed on the device itself, all management is done via Home Assistant. Display Data is just for information.
- System uses 3 Fans (up to you to use more or less).
- it is easily extendable
- **manual speed control** over ride if you don't want to use PID Control
- **no coding is needed**. Just some configuration in YAML files. 

## Parts

- **DHT11** or **DS18B20** - Any Temperature sensor ESPHome can handle will be fine. I had / prefere the OneWire type. But with only one temp. sensor any sensor is fine. If you like to have more than one. OneWire will be easier to extend. I'm using the one on a board with 3-pins. <br>Preferd: <img src="images/DS18B20.jpg" width="100"> Works also:<img src="images/dht-11.png" width="100">
<br>

- **12v PWM 4-pin Computer Fan** - I'm using 3 x [140mm Corsair fans](https://www.corsair.com/us/en/Categories/Products/Fans/Magnetic-Levitation-Fans/ml-config/p/CO-9050039-WW). Any 12v PWM-controllable fan should work. Cost $8-$15 USD. I recommend getting high quality fans if you care about noise and need to move a lot of air<br><img src="images/corsair-fan.png" width="100">. 

- **12v Power Adapter** - 1A or 2A should be fine depending on your fan's current draw. Cost $7 <br><img src="images/12v%20power%20supply.jpeg" width="100"> 

- **12v DC Female Jack** - with wire outlets. You can normally buy these with the Power Adapter<br><img src="images/12v%20jack.jpg" width="100"> or <img src="images/12v%20jack%202.png" width="100"> 

- **LM2596** or **Mini360** - to convert 12v down to 3.3v. Cost $1.50 each (normally in packs of 6-10)<br><img src="images/mini360.jpg" width="100"> or <img src="images/LM2596.png" width="100"> 

- **ESP32-C3** or **ESP32 Dev Kit**. You can use any ESP32. But this one is quit small and cheap<br><img src="images/esp32c3mini.jpg" width="100"> or <img src="images/nodemcu-esp32.png" width="100"> 

## Choosing a Good Fan
You need a 4-pin fan which has PWM. 3-pin fans aren't acceptable, they are just on/off with tachometer sensor.

As you'll see below, our fans are being powered by the PWM pin. Our expectation is that the fans stop spinning at 0% power. Some people have reported that some fans don't stop running at 0% power (or worse that they stop completely at 100% power which is weird). 

It appears that Corsair and Noctua fans behave as expected so you might want to stick with them.

However, if your fan does behave this way, you can use a MOSFET to turn it off. There are ([instructions here for how to do this](https://github.com/patrickcollins12/esphome-fan-controller/issues/17#issuecomment-1557136383)). Please post your progress to that issue.

## Wiring Diagram
### Basic Breadboard Version

***This Wiring is from the original project and only left for reference or own breadboar versions***

<img src="images/12v%20fan%20controller%20w%20tach.png">

Some important notes:
- connect the fan PWM pin to a PWM GPIO
- turn the knob on the buck converter with a screwdriver to make it output exactly 3.3v. You'll need a multimeter to measure that output.
- ensure the 12v and 3.3v grounds are connected together.
- the Blue line is the tachometer ("Tach") input pin. It is optional to connect this. You can connect this to a PWM input pin of your choice (GPIO25 in the example config) and it will send 1-2 pulses per full turn (depending on the fan). You can use this to monitor actual RPM of the fan and detect a fan defect, blocked rotor, etc. You will need one PWM input for each tach on each fan.
- you could easily skip the Buck converter and use two separate power sources 3.3v and 12v. 
- the fritzing diagram shows a 4-pin DHT-11, when in fact I have the simpler 3-pin version as shown in the parts list. The 4-pin version might need a pullup resistor, haven't tried it.

### Common Wiring Error - not joining grounds 

NOTE: if you don't join your 3.3v and 12v ground wires together your fan will keep spinning. At least 5 different builds have reported this issue. 

### Extended Custom PCB Version
This projects focus is now on a own custom PCB to have a small footprint and more options.

***Improvements***
- PWM is driven by a externel transistor (MosFET)
- PulsCounter Signal have some protective circuits to ESP input pins
- optional PullUp / PullDown resistors
- optional input voltage measurement
- optional Display, just plug an play
- Dedicated 3-Pin Molex style connectors for FANs (4th FAN possible)
- Extention pin header for e.g. UART (optional)

Early prototype & final PCB working
<br>
<img src="images/Prototype.jpg" width=200>
<img src="images/pcb_w_disp_working.jpg" width=200>

Some pictures only PCB with and without display

<img src="images/pcb_wo_disp.jpg" width=200>
<img src="images/pcb_w_disp_black.jpg" width=200>

On request I can provide SMD assembled PCB or completed kit (shipping in EU). Contact: crazyhazle@gmx.de

<img src="images/pcb_full_kit.jpg" width=300>

## Installing the software onto the ESP32

### Get this repo
Clone this github repository.
From the command line and then cd into the directory

```
git clone https://github.com/patrickcollins12/esphome-fan-controller.git
cd esphome-fan-controller
```

### Review the YAML and read the ESPHome docs.

Review the YAML file. Change the device name from ``console-fan`` to whatever seems appropriate. You might want to change the yaml filename as well.

**Ensure the pins are set correctly** for the PWM Fan (ledc) and the DS18B10 / DHT-11

All GPIOs are define by subtitutions:

```yaml
substitutions:
  name: "esp32-c3-supermini"
  friendly_name: "Inverter Fan Controller"
  # GPIO - Pin definition
  gpio_vin: GPIO0   # GPIO0 - VIN
  # gpio 2 strapped pin - boot mode
  gpio_fan_seeed_1: GPIO1 
  gpio_fan_speed_2: GPIO3
  gpio_fan_speed_3: GPIO4
  # ? GIPO5 - Only usable when WiFi not configured on the device ?
  # I2C - Display
  gpio_i2c_sda: GPIO6
  gpio_i2c_scl: GPIO7
  gpio_led_status: GPIO8  # Built In LED GPIO8
  # Strapped Pin GPI09 Bootmode, but internal pullup
  gpio_temp: GPIO9 # onewire
  gpio_pwm_fan: GPIO10 # PWM OUtput

  # UART - Not used yet
  # possible for RS485 Modbus ?
  gpio_uart_rx: GPIO20 # or alternative OneWire (JP2)
  gpio_uart_tx: GPIO21 # or RPM4 (JP1)

  # one Wire - Adresses
  # dallas_address: "0x0d031455f7fxxxxxx" - only one Senosr so need to adress them
```

Review the instructions for 
- [the ESPHome Climate Thermostat](https://esphome.io/components/climate/index.html)
-  [ESPHome PID Climate Thermostat](https://esphome.io/components/climate/pid.html)
- [DS18B20 sensor](https://esphome.io/components/sensor/dallas_temp.html) / [DHT-11 sensor](https://esphome.io/components/sensor/dht.html)



### Setup your temperature sensor

Set the correct pin for your temp sensor. Note that the DHT11 sensor is setup to use an exponential moving average. Without this filter the PID controller reacts to every minor sensor movement. If you have a faster sensor like the BME260 you might need to tweak this filter.

For OneWire:

```yaml
  - platform: dallas_temp
    #address: ${dallas_address} - only needed if there is more than one sensor
    name: "Temperature"
    id: temp_sensor_onewire
    accuracy_decimals: 3

    # If you don't smooth the temperature readings 
    # the PID controller over reacts to small changes.
    filters:
      - exponential_moving_average:  
          alpha: 0.1
          send_every: 1
```
For DHT:

```yaml
  # GET TEMP/HUMIDITY FROM DHT11
  - platform: dht
    pin: ${gpio_temp}
    temperature:
      name: "Temperature"
      id: console_fan_temperature
      accuracy_decimals: 3

      # If you don't smooth the temperature readings 
      # the PID controller over reacts to small changes.
      filters:
         - exponential_moving_average:  
             alpha: 0.1
             send_every: 1

```

(Some people [take an average of two temperature sensors](https://github.com/patrickcollins12/esphome-fan-controller/issues/5).)

### Setup your PWM fan

Make sure you connect your fan to a PWM capable GPIO. All ESP32 pins that can act as outputs can be used as PWM pins but GPIOs 34-39 can’t generate PWM.

Also note that some fans stop spinning below a certain power  e.g. 13%, so set that as the minimum.  Also a max power can be applied to the fans to make them less noisy (e.g. 80%). You might want to remove this minimum or maximum. 

```yaml
  - platform: ledc
    id: console_heat_speed
    pin: ${gpio_pwm_fan}

    # 25KHz is standard PWM PC fan frequency, minimises buzzing
    frequency: "25000 Hz"

    min_power: 13%
    max_power: 80%

```

### Setup your wifi details

``mv secrets-sample.yaml secrets.yaml``

Edit your wifi credentials in secrets.yaml. The .gitignore will prevent you accidentally uploading your wifi credentials to github.

I defined some common packages, feel free to use them.

```yaml
packages:
  wifi: !include
    file: common/wifi.yaml
    vars:
      varFriendlyName: ${friendly_name}
      varName: ${name}

  debug: !include common/debug32.yaml
  common: !include 
    file: common/common.yaml
    vars: 
        varFriendlyName: ${friendly_name}
```

### Install ESPHome

[Install ESPHome](https://esphome.io/guides/installing_esphome.html) according to the instructions on the ESPHome website.

Most people use the ESPHome that runs inside Home Assistant. You can use that too. The most simple way is to use [Web - ESPHome](https://web.esphome.io/)

### Install to ESP32

Connect your ESP32 via USB to your computer, then upload the Firmware to the ESP32.

At this time if you've set the pins right, the sensor should be spitting out values and the PID can control the fan. The basic example will show Input Voltage, FAN1 - FAN3 RPM and Current Temp.

Success! 

```
% esphome logs console-fan.yaml
INFO Reading configuration console-fan.yaml...
INFO Starting log output from console-fan.local using esphome API
INFO Successfully connected to console-fan.local
...
[22:54:09][C][mdns:085]:   Hostname: console-fan
[22:54:09][C][homeassistant.text_sensor:023]: Homeassistant Text Sensor 'ha_kp'
[22:54:09][C][homeassistant.text_sensor:024]:   Entity ID: 'input_text.kp'
[22:54:10][C][homeassistant.text_sensor:023]: Homeassistant Text Sensor 'ha_ki'
[22:54:10][C][homeassistant.text_sensor:024]:   Entity ID: 'input_text.ki'
[22:54:10][C][homeassistant.text_sensor:023]: Homeassistant Text Sensor 'ha_kd'
[22:54:10][C][homeassistant.text_sensor:024]:   Entity ID: 'input_text.kd'
[22:54:10][D][dht:048]: Got Temperature=30.0°C Humidity=38.0%
[22:54:10][D][sensor:113]: 'Humidity': Sending state 38.00000 % with 0 decimals of accuracy
[22:54:11][D][dht:048]: Got Temperature=30.0°C Humidity=38.0%
[22:54:11][D][sensor:113]: 'Humidity': Sending state 38.00000 % with 0 decimals of accuracy
[22:54:12][D][dht:048]: Got Temperature=30.0°C Humidity=38.0%
[22:54:12][D][sensor:113]: 'Humidity': Sending state 38.00000 % with 0 decimals of accuracy
```

## Setup Home Assistant

If the above steps worked correctly, the device will be auto-discovered by Home Assistant. You will need to add the device.

Multiple sensors and switches are exposed by the ESPHome software.

You also need to setup the dashboard. I'll explain those two steps below.

## Setting up the Home Assistant Dashboard

***Important***: This steps are from the orginal repo. May some adjustems have to be done.

![dashboard](images/ha.jpg)

Here is my full dashboard in Home Assistant.

For this full dashboard configuration, checkout ```lovelace-dashboard.yaml```

Let's go through this page section-by-section.

### The Primary Controls

<img src="images/primary-controls.jpg" width=400>
 
```yaml
type: entities
entities:
  - entity: fan.manual_fan_speed
  - entity: sensor.fan_speed_pwm_voltage
```

- You can turn on  `manual fan speed` and use this fan control to adjust the speed manually. 

- The ``Fan Speed (PWM Voltage)`` is the % of voltage being sent out via PWM to the fan controller. At 100% it will be sending 12v, at 50% it will be sending 6v.

If `manual fan speed` is off, this next card stack will conditionally display.

```yaml
type: conditional
conditions:
  - condition: state
    entity: fan.manual_fan_speed
    state_not: 'on'
card:
  type: vertical-stack
  cards:
    - type: entities
      title: Thermostat Fan (PID)
      entities:
        - entity: climate.console_fan_thermostat
        - entity: sensor.openweathermap_temperature
          name: open weather
        - entity: sensor.contact_sensor_1_device_temperature
          name: room temperature
    - type: vertical-stack
      cards:
        - type: glance
          entities:
            - entity: sensor.console_fan_is_in_deadband
              name: in_deadband?
            - entity: sensor.console_fan_error_value
              name: error
              icon: mdi:equal
        - type: glance
          show_icon: false
          entities:
            - entity: sensor.console_fan_output_value
              name: output
            - entity: sensor.console_fan_p_term
              name: p_term
            - entity: sensor.console_fan_i_term
              name: i_term
            - entity: sensor.console_fan_d_term
              name: d_term
```

- The `Console Fan Thermostat` is a controllable thermostat, by clicking it you can alter the target temperature and turn the fan on/off. These changes will be persisted to flash on the ESP32.

- The `Open Weather` and `Room` temperatures are from other sensors in my house for reference.

- The ``kp, ki and kd`` inputs are exposed from the device. Your ESP32 will be automatically receiving changes to these values to control the behavior of the PID controller. While you could tune these from the config.yaml it requires a compile, upload and reboot cycle each time. This is inconvenient and best to tweak in real-time. We want to expose these 3 parameters to a Home Assistant dashboard.

### The Graphs

Add the fan speed and the thermostat to two separate graphs. I've also added my room temperature from a separate device for comparison.

<img src="images/graphs.jpg" width=400>

```yaml
type: vertical-stack
title: 3 hr
cards:
  - type: history-graph
    entities:
      - entity: sensor.fan_speed_pwm_voltage
    hours_to_show: 3
    refresh_interval: 0
  - type: history-graph
    entities:
      - entity: climate.console_fan_thermostat
        name: ' '
      - entity: sensor.contact_sensor_1_temperature
        name: room
    hours_to_show: 3
    refresh_interval: 0
```

### Helpful Details - More Sensors and Switches

<img src="images/details.jpg" width=400>


This dashboard YAML exposes various sensors and switches from the ESP32.

```yaml
type: entities
entities:
  - entity: sensor.console_fan_ip_address
  - entity: sensor.console_fan_wifi_strength
  - entity: sensor.console_fan_uptime
  - entity: sensor.humidity
    name: console fan humidity
  - entity: switch.console_fan_autotune
  - entity: switch.console_fan_esp32_restart
```

- ``console_fan_autotune`` is a button which starts the [PID tuning](https://esphome.io/components/climate/pid.html#autotuning) process. I ended up abandoning this approach and manually tuning the PID.

- ``console_fan_esp32_restart`` restarts the ESP32 remotely.

### PID Parameters - setting the PID parameters from the frontend

This dashboard allows you to configure the PID parameters. See the next section for how to set these parameters.
This dashboard will conditionally disappear if `manual fan speed` control is `on`.

<img src="images/pid-controls.jpg" width=400>

```yaml
type: conditional
conditions:
  - condition: state
    entity: fan.manual_fan_speed
    state_not: 'on'
card:
  type: vertical-stack
  cards:
    - type: entities
      entities:
        - entity: number.kp
        - entity: number.ki
        - entity: number.kd
        - entity: button.pid_climate_autotune
      title: PID Controls Setup
    - type: entities
      entities:
        - entity: number.deadband_threshold_low
          name: Threshold Low
        - entity: number.deadband_threshold_high
          name: Threshold High
        - entity: number.deadband_ki_multiplier
          name: ki multiplier
      title: Deadband Parameters
```

## Tuning your fan - by configuring the PID Parameters

The thermostat is controlled using a standard Process Control system called a PID. 

In our system the goal of the PID control is to set the fan voltage (speed) to bring the temperature measured by the sensor to a target temperature (30degC).

Search the internet and you'll find many resources for setting up a PID Controller. There are a lot of resources for fast response systems like cruise control systems but not many for dealing with slow response cooling systems like this one.

The [ESPHome PID Climate](https://esphome.io/components/climate/pid.html) system that we're using here has some resources to explain how to tune the parameters. However, its autotune system didn't spit out useful results for me and I will explain how to manually get the right parameters. Your system will be different to mine and so your parameters will need to be slightly different. For instance, your cabinet will be a different size, I have two fans, you may only have one, etc.

There are only two parameters you will need to adjust: the kp and ki parameters. The kd parameter is not useful for us.

In my system the goal of tuning was to minimise aggressive changes to the fan (my wife complained she could hear the fan turning on and off). I don't mind the system drifting up to 2degC away from the target temporarily while it slowly reacts. A 7% drift (2degC/30degC) on some control systems could blow up a Nuclear plant or cause a Cruise Control system to crash into another car. But in our system a 7% short temperature drift is a fine tradeoff for quiet fans.

### Setting the kp (gain) parameter - how aggressively to cool?

KP is the main Gain. How aggressively will the fan respond to a small change in temperarature? Do you want the fan to go to 100% power in response to a 0.1degC change in temperature? In general my goal was to have the fan at 50% in response to a 1degC change in temperature, thus could normally stave off any further temperature rises, but if it the temperature did keep rising the fan will then climb to 100% power.

Using lower gain (0.1) (my preferred setting):
 - takes longer to cool down
 - is more likely to under react to fast temperature changes.
 - 2deg sharp spikes can occur before the system reacts
 - It can takes up to 10 minutes to fully close a 0.5degC gap.
 - but, it is much less likely to oscillate around the target temperature and cause the fan to turn on and off constantly.

Using higher gain (1.0):
 - responds quickly to changes in temperature
 - but, is more likely to oscillate and make the fan swing from 0% to 100% and back again as it tries to control the temperature. This means you can hear the fan trying to adjust.

### Setting the ki parameter - how long to adjust an offset?

``ki: 0.0009``

The ki parameter adjusts for temperature offset. Try setting ki to 0. Set your system initially with kp=0.1, ki=0 and kd=0. You'll find that the system operates with a constant delta/offset to the target temperature. The ki parameter adjusts for this.

1/ki is the seconds it should attempt to correct an offset. So 0.03 will adjust in 30seconds. 0.0009 will close a small temperature delta in 20 minutes. See a good description here https://blog.opticontrols.com/archives/344

Higher numbers like 0.03 will respond much quicker, but it also will cause a lot of noise and oscillation in the fan speed.

### Setting the kd parameter - predicting a change

The kd (D in PID) is meant to pre-react and backoff early. Small parameters can help overshoot but does create some fan noise and oscillation. The interwebs says that most (70%) of process controllers don't use the D and just a PI controller.

### Setting the deadband parameters - minimising changes once inside the zone

In my first contribution to ESPHome I added [Deadband to PID Climate](https://esphome.io/components/climate/pid.html#deadband-setup). Follow the instructions there to ensure that your fans stop oscillating once it reachs the correct target temperature.

### Thanks
Special thanks to: https://github.com/patrickcollins12 

