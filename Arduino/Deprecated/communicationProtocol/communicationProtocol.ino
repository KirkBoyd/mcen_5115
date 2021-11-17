char test[] = "<ROB-1555-50|OPP-50-300>";
#define START_MARKER '<'
#define END_MARKER '>'
#define COMMAND_SEP '|'
#define VALUE_SEP '-'

void setup() {
Serial.begin(9600);              //Starting serial communication
pinMode(LED_BUILTIN, OUTPUT);
}

void testLED(){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
//void loop() {
//  Serial.println(test);   // send the data
//  delay(10);                  // give the loop some break
//}

/*
    This sketch reads data from the Serial input stream of the format "<COMMAND|Value1-Value2-Value3-...-ValueN>"
    and parses it into the COMMAND and an array of N values.
    Whitespace is ignored.
*/




const size_t buffLen = 6;     // length of the expected message chunks (number of characters between two commas) (16-bit int has 5 digits + sign)
char buffer[buffLen + 1];     // add one for terminating null character
const size_t cmdBuffLen = 10; // length of the expected command string
char cmdBuffer[cmdBuffLen + 1];

uint8_t bufferIndex = 0;

const size_t arrayOfIntsLen = 12; // number of ints to receive
int arrayOfInts[arrayOfIntsLen];
uint8_t arrayOfIntsIndex = 0;

bool receiving = false;       // set to true when start marker is received, set to false when end marker is received
bool commandReceived = false; // set to true when command separator is received (or if command buffer is full)
//<MOT|255-216-160-122-0-1-0-1-1-0-1-0> // Packet Format


void loop()
{
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
                    Serial.println(arrayOfInts[0]);
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
} // end of loop

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
