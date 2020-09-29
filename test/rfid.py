
# coding: utf-8

# In[1]:


import numpy as np
import serial
import time
import threading
import queue
import argparse
import socket


# In[2]:
rfid_start = 'R;'
mag_start = 'Y;'
end_packet = ';E'



q = queue.Queue()


# In[7]:


def rfid_send():
    rfid_count = 0
    while not stop_event.wait(0):
        ser_rfid.write(bytes([170, 221, 0, 3, 1, 12, 13]))  # aa dd 00 03 01 0c 0d
        time.sleep(0.1)
        # print(ser_rfid.read())
        data = recv()
        """
        while data != 'aadd0009010c0000430020006e' and data != "aadd0009010c0000410010005c":
            rfid_count += 1
            if rfid_count == 20:
                return data
            ser_rfid.write(bytes([170, 221, 0, 3, 1, 12, 13]))
            time.sleep(0.1)
            data = recv()
        """
        #ser_rfid.write(bytes([170, 221, 0, 4, 1, 3, 10, 8]))
        time.sleep(0.1)
        _ = recv()
        if data == 'aadd0009010c0000430020006e':
            a = 1
        elif data == "aadd0009010c0000410010005c":
            a = 2
        else:
            a = 0
        q.queue.clear()
        q.put(a)
    print("stop")
    #return data


# In[8]:


def recv():
    while True:
        data = ser_rfid.read_all().hex()
        if data == b'':
            continue
        else:
            break
    return data


# In[ ]:


parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="com port", default = "COM6")
parser.add_argument("-ip", "--ip", help="up", default = "192.168.0.1")
parser.add_argument("-rp", "--rfid_port", help="rfid port", default = "/dev/ttyUSB0")
args = parser.parse_args()


# In[6]:


COM_PORT = args.port
BAUD_RATES = 115200    
ser = serial.Serial(COM_PORT, BAUD_RATES)  

COM_PORT_rfid = args.rfid_port # 請自行修改序列埠名稱
BAUD_RATES_rfid = 38400
ser_rfid = serial.Serial(COM_PORT_rfid, BAUD_RATES_rfid)


# In[10]:

stop_event = threading.Event()
t = threading.Thread(target = rfid_send)


# In[11]:


t.start()
hostname = args.ip
port = 9930
addr = (hostname,port)
clientsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# In[ ]:

mag = 0
try:
    while True:
        if q.get() == 1:
            mag = 0
            clientsock.sendto(bytes(rfid_start + "bye" + end_packet, encoding='utf8'), addr)
            print("bye")
            ser.close()
            stop_event.set()
            t.join()
            ser_rfid.close()
            break

        if q.get() == 2:
            mag = 1
            clientsock.sendto(bytes(rfid_start + "start" + end_packet, encoding='utf8'), addr)
        while ser.in_waiting and mag == 1:
            data_raw = ser.read()     
            int_raw = int.from_bytes(data_raw, byteorder='big')

            num = int_raw//64
            id = int_raw%64
          
            if(id == 57 and num == 1):

                data_raw1 = ser.read()
                data_raw2 = ser.read()
   
                int_raw1 = int.from_bytes(data_raw1, byteorder='big')
                int_raw2 = int.from_bytes(data_raw2, byteorder='big')

                P_N = (int_raw1 & 64) >> 6
                if(P_N == 1):offset = int_raw1 & 63
                else: offset = -(int_raw1 & 63)

                width = (((int_raw2&192)>>2)+((int_raw2&63)-41))*2
               
           
                print("num : {} --------- offset : {} --------- width : {} ".format(num,offset,width), end ="\r")
                clientsock.sendto(bytes(mag_start + str(offset) + end_packet, encoding='utf8'), addr)

           
            elif(id == 57 and num != 0):

                data_raw1 = ser.read()
                int_raw1 = int.from_bytes(data_raw1, byteorder='big')
                P_N = (int_raw1 & 64) >> 6
                if(P_N == 1):offset = int_raw1 & 63
                else: offset = -(int_raw1 & 63)
                print("num : {} --------- offset : {} ".format(num,offset), end ="\r")
           
            else :
                print("num : 0 --------- offset : Nan --------- width : Nan ", end = "\r")
       
       #input()

except KeyboardInterrupt:
    clientsock.sendto(bytes(rfid_start + "bye" + end_packet, encoding='utf8'), addr)
    print("bye")
    ser.close()
    stop_event.set()
    t.join()
    ser_rfid.close()
    


