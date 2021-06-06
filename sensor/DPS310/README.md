# sensor/DPS310

 Instructions from Adafruit:
 https://learn.adafruit.com/adafruit-dps310-precision-barometric-pressure-sensor/arduino

#### SPI Connection to MKR1010wifi  
```
    MKR1010 | DPS310
    
    GPIO(6)-----CS (set this pin in software)
    MOSI(8)----SD1
    SCK(9)-----SCK
    MISO(10)---SD0 
    
    5v----------5v
            (nc)3v 
    Gnd--------Gnd
```

#### Install required libraries with `arduino-cli`

```
arduino-cli lib install "Adafruit DPS310" 

arduino-cli lib install "Adafruit BusIO"

arduino-cli lib install "Adafruit Unified Sensor"
```

#### Compile and Upload with `arduino-cli` for MKR1010wifi

  Check for a connected board.
```
  arduino-cli board list
````
  Compile the script.
```  
  arduino-cli compile --fqbn arduino:samd:mkrwifi1010 DPS310.ino 
```
  Upload the script to the board.
```
  arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:samd:mkrwifi1010 DPS310.ino
```

#### Listen to serial connection with `minicom`

```
  minicom ACM0
```