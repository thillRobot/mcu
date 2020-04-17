'''
@copyright: Hariharan Srinath, 2012
@license: This work is licensed under a Creative Commons Attribution 3.0 Unported License. http://creativecommons.org/licenses/by/3.0/
'''

xmldata = """<?xml version="1.0" encoding="utf-8"?>
<LinearLayout
    android:layout_width="fill_parent"
    android:layout_height="fill_parent"
    android:background="#ff314859"
    android:orientation="vertical"
    android:divider="show_divider_beginning"
    xmlns:android="http://schemas.android.com/apk/res/android">
    <TextView
            android:layout_width="fill_parent"
            android:layout_height="0px"
            android:textSize="16dp"
            android:text="FullScreenWrapper2 Demo"
            android:textColor="#ffffffff"
            android:layout_weight="20"
            android:gravity="center"/>
    <TextView
            android:layout_width="fill_parent"
            android:layout_height="0px"
            android:background="#ff000000"
            android:id="@+id/txt_colorbox" 
            android:text="WOOP!"
            android:textSize="20dp"
            android:layout_weight="60"
            android:gravity="center"/>
    <LinearLayout
        android:layout_width="fill_parent"
        android:layout_height="0px"
        android:orientation="horizontal"
        android:layout_weight="20">
        <Button
            android:layout_width="fill_parent"
            android:layout_height="fill_parent"
            android:background="#ff66a3d2"
            android:text = "Random Color"
            android:layout_weight="1"
            android:id="@+id/but_change" 
            android:textSize="14dp"
            android:gravity="center"/>
        <Button
            android:layout_width="fill_parent"
            android:layout_height="fill_parent"
            android:background="#ff0000"
            android:text = "RED"
            android:layout_weight="1"
            android:id="@+id/but_R" 
            android:textSize="12dp"
            android:gravity="center"/>    
        <Button
            android:layout_width="fill_parent"
            android:layout_height="fill_parent"
            android:background="#008000"
            android:text = "GREEN"
            android:layout_weight="1"
            android:id="@+id/but_G" 
            android:textSize="12dp"
            android:gravity="center"/>
        <Button
            android:layout_width="fill_parent"
            android:layout_height="fill_parent"
            android:background="#0000ff"
            android:text = "BLUE"
            android:layout_weight="1"
            android:id="@+id/but_B" 
            android:textSize="12dp"
            android:gravity="center"/>
        <Button
            android:layout_width="fill_parent"
            android:layout_height="fill_parent"
            android:background="#f125567b"
            android:layout_weight="1"
            android:text = "LED"
            android:textSize="12dp"
            android:id="@+id/but_sel" 
            android:gravity="center"/>        
        <Button
            android:layout_width="fill_parent"
            android:layout_height="fill_parent"
            android:background="#f125567b"
            android:layout_weight="1"
            android:text = "Exit"
            android:textSize="12dp"
            android:id="@+id/but_exit" 
            android:gravity="center"/>    

    </LinearLayout>
</LinearLayout>"""

import android, random, time, math
from fullscreenwrapper2 import *

class DemoLayout(Layout):
    def __init__(self):
        super(DemoLayout,self).__init__(xmldata,"FullScreenWrapper Demo")
        self.LEDselect=0
        
    def on_show(self):
        self.add_event(key_EventHandler(handler_function=self.close_app))
        self.views.but_change.add_event(click_EventHandler(self.views.but_change, self.change_color))
        
        self.views.but_R.add_event(click_EventHandler(self.views.but_R, self.send_R))
        self.views.but_G.add_event(click_EventHandler(self.views.but_G, self.send_G))
        self.views.but_B.add_event(click_EventHandler(self.views.but_B, self.send_B))
        
        self.views.but_sel.add_event(click_EventHandler(self.views.but_sel, self.select))
        
        self.views.but_exit.add_event(click_EventHandler(self.views.but_exit, self.close_app))
        
    def on_close(self):
        pass
    
    def close_app(self,view,event):
        FullScreenWrapper2App.exit_FullScreenWrapper2App()

    def change_color(self,view, event):
    
        i1 = random.randint(0,255)
        hexrep = hex(i1)[2:]
        if(len(hexrep)==1):
            hexrep = '0'+hexrep   
        h1=hexrep 
        
        i2 = random.randint(0,255)
        hexrep = hex(i2)[2:]
        if(len(hexrep)==1):
            hexrep = '0'+hexrep   
        h2=hexrep 
        
        i3 = random.randint(0,255)
        hexrep = hex(i3)[2:]
        if(len(hexrep)==1):
            hexrep = '0'+hexrep   
        h3=hexrep 
        
        colorvalue = "#ff"+h1+h2+h3
        colorcommand ="!"+str(i1).rjust(3,'0')+"@"+str(i2).rjust(3,'0')+"#"+str(i3).rjust(3,'0')+"$"+str(self.LEDselect)
        
        droid.bluetoothWrite(colorcommand)
        self.views.txt_colorbox.background=colorvalue
        self.views.txt_colorbox.text=str(colorcommand)
        self.views.txt_colorbox.gravity = "left"

    def select(self,view,event):
        self.LEDselect=self.LEDselect+1
        if self.LEDselect ==3:
            self.LEDselect =0
        self.views.txt_colorbox.text="LED "+str(self.LEDselect)    
        
    def send_R(self,view, event):    
        s="!200@000#000$"+str(self.LEDselect)
        self.views.txt_colorbox.text=s 
        droid.bluetoothWrite(s)
        
    def send_G(self,view, event):    
        s="!000@200#000$"+str(self.LEDselect)
        self.views.txt_colorbox.text=s 
        droid.bluetoothWrite(s)
        
    def send_B(self,view, event):    
        s="!000@000#200$"+str(self.LEDselect)
        self.views.txt_colorbox.text=s 
        droid.bluetoothWrite(s)
    
    
    def get_rand_hex_byte(self):
        j = random.randint(0,255)
        hexrep = hex(j)[2:]
        if(len(hexrep)==1):
            hexrep = '0'+hexrep   
        return hexrep 
        
       
if __name__ == '__main__':
    droid = android.Android()
    
    # turn on phone sensors
    droid.startSensingTimed(1, 250)
    time.sleep(1)
    
    # turn bluetooth on 
    droid.toggleBluetoothState(True)
    droid.bluetoothConnect('00001101-0000-1000-8000-00805F9B34FB')
    connID=droid.bluetoothGetConnectedDeviceName()
    #print(connID)
    
    random.seed()
    FullScreenWrapper2App.initialize(droid)
    FullScreenWrapper2App.show_layout(DemoLayout())
    FullScreenWrapper2App.eventloop()
    