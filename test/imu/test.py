import struct

# if input is string, per @robyschek will fail on python 3
data=b'\x64\xd8\x64\x3f'  
print(struct.unpack('<f', data))   #little endian
print(struct.unpack('>f', data))   # big endian

#your input  
list1=[0x64, 0xD8, 0x6E, 0x3F]
print(list1)
# aa=str(bytearray(list1))  # edit: this conversion wasn't needed
aa= bytearray(list1) 
print(struct.unpack('<f', aa))