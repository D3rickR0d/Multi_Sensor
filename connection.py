import csv
import serial
from serial.tools import list_ports
import time
// arduino serial port
arduino_port = "/dev/ttyACM0" #serialport arduino
baud = 115200
// creates a csv file meant to be written in.
f = open("data.csv","w",newline='')
f.truncate()

serialCom = serial.Serial(arduino_port,baud)

#reset arduino
serialCom.setDTR(False)
time.sleep(1)
serialCom.flushInput()
serialCom.setDTR(True)

// starts the info at line 0
line=0
//continues until told to stop
while(True):

// test function will try to make communication before the actual code begins

    try:
        #readlines
        s_bytes = serialCom.readline()
        #decode binary
        decoded_bytes = s_bytes.decode("utf-8").strip('\r\n')
        

        
        // line 0 is is where the sensor begins its begin statement
        if line == 1:
	// meant to split up the values separated by "/"
            Values = decoded_bytes.split("/")
        elif line > 1:
            Values = [x for x in decoded_bytes.split()]
        print(Values)
        
        writer = csv.writer(f,delimiter=",")
        writer.writerow(Values)
        
        
    except:
        print("ERROR, line not recorded")
    line+=1    
         
f.close()