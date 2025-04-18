#include "radio.h"

Adafruit_MCP23X17 mcp;

// Set this to 1 for using mcp GPIO expander, or 0 for straight to arduino
#define USE_MCP 1

// Functions For MCP or Arduino 
#if USE_MCP
  #define digitalWriteFunc(pin, state) mcp.digitalWrite(pin, state)
  #define pinModeFunc(pin, mode) mcp.pinMode(pin, mode)
#else
  #define digitalWriteFunc(pin, state) digitalWrite(pin, state)
  #define pinModeFunc(pin, mode) pinMode(pin, mode)

#endif

uint8_t DATA_LENGTH = 0x10;

//This function execute the initialzing sequence of the tranceiver
bool Radio_Init(RadioState state){

  SPI.begin();  //Remeber to actually begin SPI (Painful to debug Trust)

  if (USE_MCP) {
    if(!mcp.begin_I2C()) {
      //Serial.println(F("Failed to find MCP23X17 chip"));
      return false;
    }
  }
  
  pinModeFunc(SSradio, OUTPUT);
  pinModeFunc(SDN, OUTPUT);
  pinModeFunc(nIRQ, INPUT);

  mcp.digitalWrite(SSradio, HIGH);
  digitalWriteFunc(SDN, HIGH);

  
  //forces the transceiver to reset
  radioShutdown();
  radioPowerUp();
  
  //Configures the transceiver
  int cfgResult = radioConfig();

  if (cfgResult == 0){ // if the configuration was successful
    uint8_t cmd[]= {GET_INT_STATUS, 0xFB, 0x7F, 0x7F}; //Cmd: Clear all interrupt flag
    if (radioCommand(cmd, sizeof(cmd)) != true){
      return false; //Failed to clear the interrupt flag
    }
  }

  if (state == RADIO_RECEIVE){
    uint8_t channel= 0x00;
    uint8_t RxCmd[] = {START_RX, channel, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08};
    if (radioCommand(RxCmd,sizeof(RxCmd)) != true){
      return false; //Failed to enter RX mode
    }
    
  }
    
  return true;
   
}


//Function To Receive Radio Data, Make sure to free data after use
uint8_t* Radio_Receive(int* size) {
  if (checkForNewPacket()) {
      int sizeOfPacketReceived = checkRxFIFOsize();
      
      Serial.print(F("Packet Size: "));
      Serial.println(sizeOfPacketReceived);

      if (sizeOfPacketReceived == 0) {
          *size = 0;
          return NULL;  // Return NULL on error
      } else {
          // Allocate memory for received data
          uint8_t* rx = (uint8_t*)malloc(sizeOfPacketReceived * sizeof(uint8_t));
          if (rx == NULL) {
              *size = 0;
              return NULL;  // Memory allocation failed
          }
          getReceivedPacket(rx, sizeOfPacketReceived);
          *size = sizeOfPacketReceived;
          return rx;  // Caller must free(rx) after use
      }
  }
  *size = 0;
  return NULL;
}

//Function To Transmit Radio Data, Make sure to free data after use
bool Radio_Transmit(uint8_t* data, uint8_t dataSize) {
  bool ok = false;

  if (data == nullptr || dataSize == 0) {
      return ok; // Invalid input
  }

  // Create FIFO command with WRITE_TX_FIFO + data
  uint8_t FIFOCmd[dataSize + 1];
  FIFOCmd[0] = WRITE_TX_FIFO;
  memcpy(FIFOCmd + 1, data, dataSize); // Copy data into FIFO command

  digitalWriteFunc(SSradio, LOW);

  //Serial.print(F("Size of FIFOCmd: "));
  //Serial.println(sizeof(FIFOCmd));

  // Write to the FIFO buffer 
  delay(1);
  for (uint8_t i = 0; i < sizeof(FIFOCmd); i++) {
      SPI.transfer(FIFOCmd[i]);
  }
  digitalWriteFunc(SSradio, HIGH);

  // Start TX mode
  uint8_t channel = 0x00;
  uint8_t txCmd[] = {START_TX, channel, 0x00, 0x00, 0x00};
  if (radioCommand(txCmd, sizeof(txCmd))) {
    ok = true;
  }

  // Clear the FIFO buffer after transmission
  //TODO: Poll Register instead of delay?
  delay(1000); // Ensure time for transmission
  uint8_t infoClear[] = {FIFO_INFO, 0x02};
  uint8_t siz[2];
  radioCommand(infoClear, sizeof(infoClear), siz, sizeof(siz));

  return ok;
}

//This function configures the transceiver
int radioConfig(){
  
  int byteNumber = 0;
  int cmdNumber =1;
  
  while(cfg[byteNumber] != 0x00){ // while the length of the command is different of zero (while there is any command left) 
    
    uint8_t cmdLength = cfg[byteNumber]; //the first byte is the length of the command
    uint8_t cmd[cmdLength]; //creates a table to save the current command
    byteNumber++;
    
    memcpy(cmd, &cfg[byteNumber], cmdLength); //copies the command into the array previously created
    
    if (radioCommand(cmd, cmdLength)){//sends command to the radio and checks if the result is true
      if (cmdNumber ==1){
        uint8_t cmd[]= {GET_INT_STATUS, 0x00, 0x00, 0x00}; //Cmd: Clear all interrupt flag
        radioCommand(cmd, sizeof(cmd));
      }
      byteNumber += cmdLength; //moves to the next command
      cmdNumber++; 
    }
    else return cmdNumber; // Returns which configuration command was not exacuted properly
  }
  
  uint8_t answer[1];
  uint8_t command[]= {SET_PROPERTY, 0x00, 0x01, 0x02, 0x08}; // Sets the low battery threshold to 1.9V
  if (radioCommand(command, sizeof(command), answer, sizeof(answer))){
    return 0; // Radio was configured properly
  }
  else {
    return -1; // Error during the configuration
  }
}

//This function will check if the transceiver is ready to accept a new command
bool radioReady(){
  bool stat = false; //If stat = true then the transceiver is ready to receive a new command.
  int count =0;

  //We will give it a thousand try before given up, to let it time to finish what is is doing 
  while ((false!=true) && (count < 1000)){
      digitalWriteFunc(SSradio, LOW);
      delay(1);
      SPI.transfer(READ_CMD_BUFF);
      if (SPI.transfer(0) == CTS){
        stat = true;
      }
      count++;
      digitalWriteFunc(SSradio, HIGH);
    }
  return stat;
}

//This function forces the transceiver to shutdown
void radioShutdown(){
  digitalWriteFunc(SDN, HIGH);
  delay(100);
   
}

//This function forces the shutdown pin back to LOW, turning on the transceiver
void radioPowerUp(){
  //Power On Reset
  digitalWriteFunc(SDN, LOW);
  delay(20); //the si4464 needs a maximum of 6ms before being able to do anything after powering up
}



/*
 * This function will send a command to the transceiver and recover potential answers
 * 
 * const char* write_buf: address of the array containing the command bytes
 * char write_len: Number of bytes in the command
 * char* read_buf: (Optional) address of the array that will store the answer
 * char* read_len: (Optional) length of the answer
 */
bool radioCommand(const uint8_t* write_buf, uint8_t write_len, uint8_t* read_buf, uint8_t read_len)
{

    bool done = false; //if command is executed then done= true
    int count=0;

    //Let send the command bytes to the transceiver over SPI
    digitalWriteFunc(SSradio, LOW);

    delay(1);
    if (write_buf && write_len){ 
      while (write_len--){
        SPI.transfer(*write_buf++);
      }
    }

    digitalWriteFunc(SSradio, HIGH);
    delay(3);

    
    //Now let's make sure the command was executed and read the answer if there is any
    while ((done!=true) && (count < 2500)){ // Up to 1500 retry to check if the radio is ready 
      
      //delayMicroseconds(5);
      digitalWriteFunc(SSradio, LOW);
      delayMicroseconds(5);
      //delay(1);
      SPI.transfer(READ_CMD_BUFF);
      //Let's check if the radio is ready
      if (SPI.transfer(0) == CTS){
        //and receive the answer if there is any
        if (read_buf && read_len){ 
          while (read_len--){
            *read_buf++ = SPI.transfer(0x00); 
          }
        }
        done = true;
      }
      count++;
      digitalWriteFunc(SSradio, HIGH);
      //delay(5);

    }
    
    return done; // False if too many attempts at CTS

}


/*
 * This function reads the ADC internal to the transceiver to access the voltage and the temperature of the transceiver
 * 
 * float *battery: address of the variable to store the voltage of the transceiver
 * float *degree: address of the variable to store the temperature of the transceiever
 */
void radioBatTemp(float *battery, float *degree){
  
   if (radioReady()==true){ //Checking that the transceiver is not busy
      uint8_t cmd[]={GET_ADC_READING, 0x18}; // reads only temperature and battery voltage
      uint8_t info[6];
      radioCommand(cmd, sizeof(cmd), info, sizeof(info));
      
      // The raw values of voltage is store on bytes 2 & 3; temperature on bytes 4 & 5
      uint16_t batByte = (info[2]<<8) | info[3];
      uint16_t tempByte = (info[4]<<8) | info[5];

      //Now let's convert that into actual volts and degrees
      *battery = (3*float(batByte))/float(1280);
      *degree = (float(tempByte)*(899/float(4096)))-293;
   }
}


bool checkForNewPacket(){
  uint8_t PHStatus[]= {GET_PH_STATUS, 0x00};
  uint8_t PHint[2];  

    // First we check the interrupt status to see if any packet was received properly
    if (radioCommand(PHStatus, sizeof(PHStatus), PHint, sizeof(PHint))){
      if ((PHint[0] & 0x10)== 0x10){  
        return true;    
      }
    }
  return false;

}

char checkRxFIFOsize(){
  uint8_t info[] = {FIFO_INFO, 0x00};
  uint8_t siz[2];
  if (radioCommand(info, sizeof(info), siz, sizeof(siz))){
    //Serial.print(F("Size of FIFO 0: "));
    //Serial.println(siz[0]);
    //Serial.print(F("Size of FIFO 1: "));
    //Serial.println(siz[1]);
    //Serial.print(F("Size of FIFO 2: "));
    //Serial.println(siz[2]);
    return siz[0];
  }
  else return 0;
}

void getReceivedPacket(uint8_t *arrayRX, int sizeArray){
  for (int i = 0; i< sizeArray; i++){ 
     arrayRX[i]=0xFF;
  }
  //The FIFO buffer can now be read
  digitalWriteFunc(SSradio, LOW);
  delay(1);
  SPI.transfer(READ_RX_FIFO);
  for (int i = 0; i< sizeArray; i++){ //initializing storage
     arrayRX[i]=SPI.transfer(0x00);
  }          
  digitalWriteFunc(SSradio, HIGH);

  //uint8_t infoClear[] = {FIFO_INFO, 0x02};
  //radioCommand(infoClear, sizeof(infoClear), siz, sizeof(siz)); //clears RX FIFO

}


/*
 * This fonction is here for debugging purpose.
 * Its only job is to display on the serial port how the tranceiver was configured.
 */
void printRadioConfig(){
  //Serial.println("\n========== RADIO CONFIGURATION DATA ARRAY ==========");
  Serial.println(F("Radio Config Array"));
  int byteNumber = 0;
  int cmdNumber =1;

  while(cfg[byteNumber] != 0x00){ // while the length of the command is different of zero (while there is any command left) 

    
    Serial.print("Command #" + String(cmdNumber,DEC) + " ");
    int cmdLength = cfg[byteNumber]; //the first byte is the length of the command
    Serial.print("\t(Length " + String(cmdLength,DEC) + " Bytes): \t");
    
    uint8_t cmd[cmdLength]; //creates a table to save the current command
    byteNumber++;
    
    memcpy(cmd, &cfg[byteNumber], cmdLength);

    for (int i=0; i<cmdLength; i++){
      Serial.print(cmd[i],HEX);
      Serial.print(" ");
    }
    byteNumber += cmdLength;
    cmdNumber ++;
    Serial.println();
   
  }
  //Serial.println(F("===================================================="));
  Serial.println(F("===="));
}

/*
 * This function is here for debugging purpose.
 * Its only job is to display the info of the transceiver
 */
void printRadioInfo() {
  // Reading radio part info
  if (radioReady()) {
    uint8_t cmd[] = {0x01};
    uint8_t info[8];
    radioCommand(cmd, sizeof(cmd), info, sizeof(info));

    Serial.println(F("\n========= RADIO INFO ========="));
    Serial.print(F("Model\t\tSi"));
    Serial.println(String(info[1], HEX) + String(info[2], HEX));

    Serial.print(F("Revision\t"));
    Serial.println(String(info[0], HEX));

    Serial.print(F("PBuild\t\t"));
    Serial.println(String(info[3], HEX));

    Serial.print(F("ROM ID\t\t"));
    Serial.println(String(info[7], HEX));

    Serial.print(F("ID \t\t"));
    Serial.println(String(info[4], HEX) + String(info[5], HEX));

    Serial.print(F("Customer ID\t"));
    Serial.println(String(info[6], HEX));

    Serial.println(F("=============================="));
  } else {
    Serial.println(F("Radio not ready"));
  }

  float radioVoltage = 0;
  float radioTemp = 0;
  radioBatTemp(&radioVoltage, &radioTemp);

  Serial.print(F("Radio Voltage: \t\t"));
  Serial.print(radioVoltage);
  Serial.println(F("V"));

  Serial.print(F("Radio Temperature: \t"));
  Serial.print(radioTemp);
  Serial.println(F("°C"));
}

//Taken from the setup() file from demo
void Radio_Init_Verbose(RadioState state){
  //Serial.println(F("\nHELLO WORLD!  Initializing...\n"));
  SPI.begin();
  if (USE_MCP) {
    if(!mcp.begin_I2C()) {
      Serial.println(F("Failed to find MCP23X17 chip"));
      return;
    }
  }

  // displays the configuration of the radio
  printRadioConfig(); 

  //Let's initialize the transceiver
  //int cfgResult = Radio_Init(state); 


  pinModeFunc(SSradio, OUTPUT);
  pinModeFunc(SDN, OUTPUT);
  pinModeFunc(nIRQ, INPUT);

  digitalWriteFunc(SSradio, HIGH);
  digitalWriteFunc(SDN, HIGH);

  //forces the transceiver to reset
  radioShutdown();
  radioPowerUp();

  //Configures the transceiver
  int cfgResult = radioConfig();

  
  if (cfgResult ==0){ // if the configuration was successful
    uint8_t cmd[]= {GET_INT_STATUS, 0xFB, 0x7F, 0x7F}; //Cmd: Clear all interrupt flag
    if (radioCommand(cmd, sizeof(cmd))){
      Serial.println(F("Interrupt flag cleared"));
    }
    else{
      Serial.println(F("Failed to clear interrupt flag"));
      return;
    }
  }
  else{
    Serial.println(F("Failed to configure radio"));
    return;
  }
  //End Taken From Radio Init
  
  //Now let's make sure the radio was initialized properly
  if (cfgResult == 0){ 
    uint8_t cmd[]= {REQUEST_DEVICE_STATE};
    uint8_t ans[2];

    //Let's check for the transceiver state
    if (radioCommand(cmd, sizeof(cmd), ans, sizeof(ans))){
      if ((ans[0] == 3) || (ans[0] ==4)){
         Serial.println(F("Radio init \t\t\tDONE!"));
         Serial.println("Radio current channel: \t\t" + String(ans[1]));
         printRadioInfo(); //displays the part info
      }
      else {
        Serial.println("ERROR DURING CONFIG: Transceiver sate: 0:");
        Serial.println(" 0:" + String(ans[0],DEC));
        Serial.println(" 1:" + String(ans[1],DEC));
        Serial.println(" 2:" + String(ans[2],DEC));
        return;
      }
    }
    
  }
  else{
    Serial.println("ERROR DURING CONFIG: " + String(cfgResult, DEC));
    return;
  }

  Serial.println();

  
  uint8_t intStatus[]= {GET_INT_STATUS, 0xFB, 0x7F, 0x7F}; //This is the command for reading the interruption status without clearing them
  //uint8_t intStatusClear[]= {GET_INT_STATUS, 0x00, 0x00, 0x00}; //This command is the same as the previous one but will clear the interruption status
  uint8_t interr[8];

  //We will now read and print on the serial port the interruptions status
  //This is use to make sure one more time that the transceiver was initialized correctly
  Serial.print(F("Checking interrupt status: \t"));
  if (radioCommand(intStatus, sizeof(intStatus), interr, sizeof(interr))){
    for (size_t i = 0; i< sizeof(interr); i++){
      Serial.print(interr[i],HEX);
      Serial.print("\t");  
    }
    Serial.println();
  }
  else{
    Serial.println(F("FAILED To Check Interrupt Status"));
  }

  //Now let's check the chip status
  //It is not strictly necessary but gets a bit more  info on the chip status that are not given by the interruption status
  uint8_t cmd2[] = {GET_CHIP_STATUS};
  uint8_t ans[4];
  Serial.print(F("Checking chip status: \t\t"));
  if (radioCommand(cmd2, sizeof(cmd2), ans, sizeof(ans))){
    for (size_t i = 0; i< sizeof(ans); i++){
      Serial.print(ans[i],HEX);
      Serial.print("\t");  
    }
    Serial.println();
  }
  else{
    Serial.println(F("FAILED To Check Chip Status"));
  }
  Serial.println();



  if (state == RADIO_TRANSMIT){
    Serial.println(F("SET as a transmitter"));
  }

  //This will set the transceiver into RX mode and command it to start listening on channel 0
  else if (state == RADIO_RECEIVE){
    Serial.println(F("SET as a receiver"));
    uint8_t channel= 0x00;
    uint8_t RxCmd[] = {START_RX, channel, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08};
    if (radioCommand(RxCmd,sizeof(RxCmd))!= true){
      Serial.println(F("FAIL to enter RX!"));
    }
    
  }
  Serial.println(F("End of setup"));
}


void Radio_Receive_Verbose(){
  int packetSize;
  uint8_t* data = Radio_Receive(&packetSize);
  if (data) {
    Serial.print(F("Received Packet: "));

    // Print HEX values
    Serial.print(F("Char Dump: "));
    for (int i = 0; i < packetSize; i++) {
        Serial.print((char)data[i]);    //Prints in ASCII
        //Serial.print(data[i], HEX);   //Prints in HEX
        Serial.print(F(" "));
    }
    Serial.println();

    free(data);  // Free memory after use
  } else {
      Serial.println(F("No new packet received"));
  }
}


void Radio_Transmit_Test(){
  uint8_t message[] = {'T', 'E', 'S', 'T', '!', '!', '\n'};
  bool success = Radio_Transmit(message, sizeof(message));
  Serial.println(success ? F("Transmission successful") : F("Transmission failed"));
}

void Radio_Large_Transmit_Test(){
  // Create a large message of ~400 bytes
  uint8_t message[60];

  // Fill the message with a repeating pattern
  const char pattern[] = "LARGE_MESSAGE_TEST_"; // 20 bytes (including underscore)
  int patternSize = sizeof(pattern) - 1; // Excluding null terminator
  
  Serial.print(F("Message to Transmit: "));
  for (size_t i = 0; i < sizeof(message); i++) {
    message[i] = pattern[i % patternSize];
    Serial.print((char)message[i]);
  }


  // Transmit the large message
  bool success = Radio_Transmit(message, sizeof(message));

  // Print success or failure
  Serial.println(success ? F("Large Transmission successful") : F("Large Transmission failed"));
}

//Taken from the setup() file from demo
void Radio_Transmit_Hello(){
  if(sendHello()){
    Serial.println(F("Packet sent!"));
  }
  else Serial.println(F("Sending FAILED!"));
}


bool sendHello(){
  bool success = false;
  //The message that will be sent is "HELLO!!"
  uint8_t FIFOCmd[] = {WRITE_TX_FIFO, 0x48, 0x45, 0x4C, 0x4C, 0x4F, 0x21, 0x21}; 

  //It needs to be written to the FIFO buffer before being transmitted
  //RADIO_CTRL_PORT &= ~SS_radio; 
  digitalWriteFunc(SSradio, LOW);
  delay(1);
  for( uint8_t i =0; i<sizeof(FIFOCmd); i++){
    SPI.transfer(FIFOCmd[i]);
  }
  //RADIO_CTRL_PORT |= SS_radio;
  digitalWriteFunc(SSradio, HIGH);

  //Now the radio can be set in Tx mode and start transmitting what is in the FIFO buffer
  uint8_t channel = 0x00;
  uint8_t txCmd[] = {START_TX, channel, 0x00, 0x00, 0x00};
  if (radioCommand(txCmd,sizeof(txCmd))){
    success = true;
  }
  else success = false;

  //You might want to clear the FIFO buffer after transmittion
  delay(1000); //gives time to send data
  uint8_t infoClear[] = {FIFO_INFO, 0x02};
  uint8_t siz[2];
  radioCommand(infoClear, sizeof(infoClear), siz, sizeof(siz));

  return success;

}
