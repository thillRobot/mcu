import android, time

droid = android.Android() 

# turn bluetooth on 
droid.toggleBluetoothState(True)
droid.bluetoothConnect('00001101-0000-1000-8000-00805F9B34FB')
connID=droid.bluetoothGetConnectedDeviceName()
print(connID)

time.sleep(1)

#droid.bluetoothWrite('!050@000#000')
        droid.bluetoothWrite ('B')
droid.exit()
