# sensor/BNO055 - Absolute Orientation Sensor Board

 Instructions from Adafruit:
 https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor

#### I2C Connection to NANO328p  
```
    NANO328p | BNO055
    
    SDA(A4/27)----SDA 
    SCL(A5/28)----SCL
    
    5v----------5v
            (nc)3v 
    Gnd--------Gnd
```

#### Install required libraries with `arduino-cli`

```
arduino-cli lib install "Adafruit BNO055"
```

#### Compile and Upload with `arduino-cli` for Nano328p

  Check for a connected board.
```
  arduino-cli board list
````
  Compile the script.
```  
  arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 BNO055.ino
```
  Upload the script to the board.
```
  arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328 BNO055.ino
```

#### Listen to serial connection with `minicom`

```
  minicom USB0
```