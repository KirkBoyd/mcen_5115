import numpy as np
import serial

ser = serial.Serial('COM3',9600) #Windows
#ser = serial.Serial('\dev\ttyUSB*',9600) #Unix

START_MARKER = '<'
END_MARKER = '>'
COMMAND_SEP = '|'
VALUE_SEP = ','

buffLen = 6         #length of the expected message chunks (number of characters between two commas) (16-bit int has 5 digits + sign)
cmdBuffLen = 10     #length of the expected command string
arrayOfIntsLen = 16 #number of ints to receive
buffer = np.zeros(buffLen + 1) #add one for terminating null character
cmdBuffer = np.zeros(cmdBuffLen + 1)
bufferIndex = 0

arrayOfInts = np.zeros(arrayOfIntsLen)
arrayOfIntsIndex = 0

receiving = False   #set to true when start marker is received, set to false when end marker is received
commandReceived = False    #set to true when command separator is received (or if command buffer is full)



def parse():
    global buffer
    global cmdBuffer
    global bufferIndex
    global arrayOfInts
    global arrayOfIntsIndex
    global receiving
    global commandReceived
    
    if (ser.in_waiting > 0): # If there's at least one byte to read
        serialByte = ser.read().decode("utf-8") # Read it
        if (isWhiteSpace(serialByte)):
            return; # Ignore whitespace

        if (serialByte == START_MARKER): #Start marker received: reset indices and flags
            receiving = True
            commandReceived = False
            bufferIndex = 0
            arrayOfIntsIndex = 0
            return

        if (receiving): #If the start marker has been received
            if (not commandReceived): #If the command hasn't been received yet
                if (serialByte == COMMAND_SEP or serialByte == END_MARKER): #If the command separator is received
                    cmdBuffer[bufferIndex] = '\0' #Terminate the string in the buffer
                    if (cmdBuffer == "RAW"): #Check if the received string is "RAW"
                        print("RAW:")
                    else:
                        print("Unknown command (")
                        print(cmdBuffer)
                    if (serialByte == END_MARKER): #If the end marker is received
                        print("Message finished: (No data)")
                        receiving = False #Stop receivinng
                    else:
                        bufferIndex = 0 #Reset the index of the buffer to overwrite it with the numbers we're about to receive
                        commandReceived = True
                elif (bufferIndex < cmdBuffLen): #If the received byte is not the command separator or the end marker and the command buffer is not full
                    cmdBuffer[bufferIndex +1] = serialByte #Write the new data into the buffer
                else: #If the command buffer is full
                    print("Error: command buffer full, command is truncated")
            elif (serialByte == VALUE_SEP or serialByte == END_MARKER): #If the value separator or the end marker is received
                if (bufferIndex == 0): #If the buffer is still empty
                    print("(Empty input)")
                else:                           #If there's data in the buffer and the value separator or end marker is received
                    buffer[bufferIndex] = '\0'  #Terminate the string
                    parseInt(buffer)            #Parse the input
                    bufferIndex = 0             #Reset the index of the buffer to overwrite it with the next number
                if (serialByte == END_MARKER): #If the end marker is received
                    print("Message finished:")
                    printArrayOfInts()
                    receiving = False  #Stop receivinng
            elif (bufferIndex < buffLen): #If the received byte is not a special character and the buffer is not full yet
                buffer[bufferIndex + 1] = serialByte #Write the new data into the buffer
            else: #If the buffer is full
                print("Error: buffer is full, data is truncated")
            return #Optional (check for next byte before executing the loop, may prevent the RX buffer from overflowing)

def isWhiteSpace(character):
    if (character == ' '):
        return True
    if (character == '\r'):
        return True
    if (character == '\n'):
        return True
    return False
    
def parseInt(input):
    global arrayOfInts
    print("\tInput:\t")
    if (arrayOfIntsIndex >= arrayOfIntsLen):
        print("Error: array of ints is full")
        return
    value = int(input)
    arrayOfInts[arrayOfIntsIndex + 1] = value

def printArrayOfInts():
    for i in range(arrayOfIntsIndex):
        print(arrayOfInts[i])

while True:
    parse()