# Board - PikoBLX

The PikoBLX is specially designed for acro and FPV micro and mini racing copters. The board provides 
various features in a small form factor, which makes it attractive for sub-130 size racers. It offers 20mm 
mounting hole spacing and a outer dimension of 27x27mm.   

The board is based on a STM32F3 processor whith 256KBytes flash, 3 hardware serial ports and floating point unit.
Main features:

* builtin voltage regulator for up to 5S operation, providing 5V and up to2A
* builtin power distibution board
* builtin LC filter for VTX
* JST connector for spektrum satellites
* SBUS software inverter
* hexa copter support
* buzzer and WS2812 LED
* MPU6000 on SPI bus
* battery monitoring
* reverse current protection
* race timing transponder


Due to its small form factor only serial receiver types are supported.   

Further information can be found [here](http://furiousfpv.com/product_info.php?products_id=41).

##Receiver Configurations

###Spektrum RX
The onboard JST connector is connected to a 3.3V voltage regulator and UART3.

###SBUS
Solder the jumper to SBUS and connect your receiver to the PPM pin on the left side.

###PPM
Solder the jumper to PPM and use the PPM pin on the left side.

##Pin Mapping

###PWM Output
| Pin Label | Description              |
| --------- | ------------------------ |
| PA4       | PWM1                     |
| PA6       | PWM2                     |
| PB0       | PWM3                     |
| PB1       | PWM4                     |
| PA1       | PWM5                     |
| PA2       | PWM6                     |


###MPU6000
| Pin Label | Description              |
| --------- | ------------------------ |
| PA15      | MPU_INT                  |
| PB12      | MPU_CS                   |
| PB15      | SPI2_MOSI                |
| PB14      | SPI2_MISO                |
| PB13      | SPI2_SCK                 |

###UARTS
| Pin Label | Description              |
| --------- | ------------------------ |
| PB7       | UART1_RX                 |
| PB6       | UART1_TX                 |
| PB4       | UART2_RX                 |
| PB3       | UART2_TX                 |
| PB11      | UART3_RX                 |

###MISC
| Pin Label | Description              |
| --------- | ------------------------ |
| PA7       | PPM                      |
| PA5       | ADC VBAT                 |
| PB9       | LED0                     |
| PB5       | LED1                     |
| PB11      | Spektrum Bind            |
| PA8       | Transponder              |


##Pin Layout

```
           TOP VIEW
+---------------------------------+
|        +------+                 |
|        |Boot  |                 |
+-----+  +------+        ^  +-----+
|9 |10|     +-------+    |  | USB |
+-----+     |       |    |  |     |
|7 |8 |     |STM32F3|    +  +-----+
+-----+     |       |             |
|5 |6 |     +-------+       +-----+
+-----+                     |13|14|
|3 |4 |                     +-----+
+-----+                     |11|12|
|1 |2 | +-------+ +-------+ +-----+
+-----+ |BAT|GND| |11 |12 |       |
|       +-------+ +-------+       |
|                                 |
+---------------------------------+
```

On the board the ESC ports are labled on the silk screen layer. ESC1,ESC2,...,ESC6 pad provides the PWM singal, the pads BAT++ and BAT--
left and right of it are for using the integrated power distribution board. Those pads can also be left open if an external 
power distribution board is used.   
The diagram above shows the position of the boot pins. It may be necessary to short those to activate the bootloader.
BAT and GND are for connecting a flight battery directly using the internal switching regulator. Up to 5S Lipo is possible.   

| Pin Number | Description              |
| ---------- | ------------------------ |
| 1          | GND                      |
| 2          | GND                      |
| 3          | +5V                      |
| 4          | +5V                      |
| 5          | PPM,SBUS,...             |
| 6          | RX2 --> UART2            |
| 7          | RX1 --> UART1            |
| 8          | TX2 --> UART2            |
| 9          | TX1 --> UART1            |
| 10         | LED Strip                |
| 11         | Buzzer +                 |
| 12         | Transponder +            |
| 13         | Buzzer -                 |
| 14         | Transponder -            |




