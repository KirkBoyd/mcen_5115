import serial

ser = serial.Serial('COM3',9600) #Windows
#ser = serial.Serial('\dev\ttyUSB*',9600) #Unix
while True:
	packet = ser.readline().decode("utf-8").replace("\n", "")
	print(packet)
