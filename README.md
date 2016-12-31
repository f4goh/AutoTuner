# Arduino HF AutoTuner #
F4GOH Anthony f4goh@orange.fr <br>

https://hamprojects.wordpress.com/2016/12/31/hf-automatic-tuner/

December 2016

Use this library freely with Arduino 1.6.5

## firmware version ##
- boite_de_couplageV08 (Arduino ATU firmware)
- boitier_de_commandeV02 (Arduino remote CTRL firmware)

## Installation ##
use liquid cristal i2c library 

## Usage notes ##


ATU depend of your I2C expander

MCP23008 expander :

```c++
#define mcpLRAdr 0x20    //adresse i2c mcp23008
#define mcpCRAdr 0x21
```

PCF8574 expander :

PCF8574  Address range is 0x20-0x27

PCF8574A Address range is 0x38-0x3F
```c++
#define pcfLRAdr 0x38    //address i2c PCF8574AT
#define pcfCRAdr 0x20    //address i2c PCF8574T
```
uncomment if you have an LCD display connected on ATU

```c++
//#define AFFLCD
```
uncomment for debug (in this case RS485 protocol is corrupted with debug function)

```c++
 //#define DEBUG_CMD
//#define DEBUG_SWR
//#define DEBUG_DBM
```

searchDelay 20 :without LCD connected direclty on ATU <br>

searchDelay 1 :with LCD connected direclty on ATU



```c++
#define searchDelay 20
//#define searchDelay 1
```


