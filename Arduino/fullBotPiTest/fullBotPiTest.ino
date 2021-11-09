#define START_MARKER '<'
#define END_MARKER '>'
#define COMMAND_SEP '|'
#define VALUE_SEP '-'
String input;

#define aIn1_f 2//teensy pin 39
#define aIn2_f 4//teensy pin 38
#define pwmA_f 3//teensy pin 14 // speed for front left motor
#define bIn1_f 5//teensy pin 40
#define bIn2_f 7//teensy pin 41
#define pwmB_f 6//teensy pin 37 //speed for front right motor
#define aIn1_b 8//teensy pin 15
#define aIn2_b 9//teensy pin 16
#define pwmA_b 10//teensy pin 36 //speed for back left motor
#define bIn1_b 13//teensy pin 17
#define bIn2_b 12//teensy pin 22
#define pwmB_b 11//teensy pin 33 //speed for back right motor

void setup() {
  pinMode(aIn1_f, OUTPUT);
  pinMode(aIn2_f, OUTPUT);
  pinMode(pwmA_f, OUTPUT);
  pinMode(bIn1_f, OUTPUT);
  pinMode(bIn2_f, OUTPUT);
  pinMode(pwmB_f, OUTPUT);
  pinMode(aIn1_b, OUTPUT);
  pinMode(aIn2_b, OUTPUT);
  pinMode(pwmA_b, OUTPUT);
  pinMode(bIn1_b, OUTPUT);
  pinMode(bIn2_b, OUTPUT);
  pinMode(pwmB_b, OUTPUT);
  Serial.begin(9600);
}

const size_t buffLen = 6;     // length of the expected message chunks (number of characters between two commas) (16-bit int has 5 digits + sign)
char buffer[buffLen + 1];     // add one for terminating null character
const size_t cmdBuffLen = 10; // length of the expected command string
char cmdBuffer[cmdBuffLen + 1];

uint8_t bufferIndex = 0;

const size_t arrayOfIntsLen = 12; // number of ints to receive
int arrayOfInts[arrayOfIntsLen]= {0};
uint8_t arrayOfIntsIndex = 0;

bool receiving = false;       // set to true when start marker is received, set to false when end marker is received
bool commandReceived = false; // set to true when command separator is received (or if command buffer is full)



void loop() {
  if (Serial.available() > 0)
    {                                    // If there's at least one byte to read
        char serialByte = Serial.read(); // Read it
        if (isWhiteSpace(serialByte))
            return; // Ignore whitespace

        if (serialByte == START_MARKER)
        { // Start marker received: reset indices and flags
            receiving = true;
            commandReceived = false;
            bufferIndex = 0;
            arrayOfIntsIndex = 0;
            return;
        }
        if (receiving){ // If the start marker has been received
            if (!commandReceived){ // If the command hasn't been received yet
                if (serialByte == COMMAND_SEP || serialByte == END_MARKER){// If the command separator is received
                    cmdBuffer[bufferIndex] = '\0'; // Terminate the string in the buffer
                    if (strcmp(cmdBuffer, "MOT") == 0){ // Check if the received string is "MOT"
                    }
                    if (strcmp(cmdBuffer, "STOP") == 0){ // Check if the received string is "MO0"
                      Serial.println("STOP");
                      for(int i =0; i < arrayOfIntsLen; i++){
                        arrayOfInts[i] = 0;
                      }
                    }
                    else
                    {
                        bufferIndex = 0; // Reset the index of the buffer to overwrite it with the numbers we're about to receive
                        commandReceived = true;
                    }
                }
                else if (bufferIndex < cmdBuffLen)
                {                                          // If the received byte is not the command separator or the end marker and the command buffer is not full
                    cmdBuffer[bufferIndex++] = serialByte; // Write the new data into the buffer
                }
                else
                { // If the command buffer is full
                }
            }
            else if (serialByte == VALUE_SEP || serialByte == END_MARKER){ // If the value separator or the end marker is received
                if (bufferIndex == 0)
                { // If the buffer is still empty
                }
                else
                {                               // If there's data in the buffer and the value separator or end marker is received
                    buffer[bufferIndex] = '\0'; // Terminate the string
                    parseInt(buffer);           // Parse the input
                    bufferIndex = 0;            // Reset the index of the buffer to overwrite it with the next number
                }
                if (serialByte == END_MARKER)
                { // If the end marker is received
                    receiving = false; // Stop receivinng
                }
            }
            else if (bufferIndex < buffLen)
            {                                       // If the received byte is not a special character and the buffer is not full yet
                buffer[bufferIndex++] = serialByte; // Write the new data into the buffer
            }
            else
            { // If the buffer is full
            }
            return; // Optional (check for next byte before executing the loop, may prevent the RX buffer from overflowing)
        }           // end if (receiving)
    }               // end if (Serial.available() > 0)


    for(int i =0; i<4; i++){
      moveMotor(i,arrayOfInts[i+4]==1,arrayOfInts[i+8]==1,arrayOfInts[i]);
    }
}//end void loop

bool isWhiteSpace(char character)
{
    if (character == ' ')
        return true;
    if (character == '\r')
        return true;
    if (character == '\n')
        return true;
    return false;
}

void parseInt(char *input)
{
    if (arrayOfIntsIndex >= arrayOfIntsLen)
    {
        return;
    }
    int value = atoi(input);
    arrayOfInts[arrayOfIntsIndex++] = value;
}
