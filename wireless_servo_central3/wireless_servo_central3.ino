#include<cppQueue.h>
#include<ArduinoBLE.h>

// PARAMETERS
const int RXCHARACTERISTICSIZE = 64; // size of the incoming strings via bluetooth/serial
const int TXCHARACTERISTICSIZE = 8; // size of the outgoing strings via bluetoothconst unsigned int COMMANDSIZE = 64; // size of the command strings
const int COMMANDSIZE = 64; // size of the command strings, should be the same as RXCHARACTERISTICSIZE
const char LOCALNAME[25] = "wireless_servo_central"; // name of the Arduino as a bluetooth device
const int DATARATE = 500; // ms after which a data packages are send via serial
const int PERIPHERALCOUNT = 6; // number of peripherals to connect to
const char PERIPHERALNAMES[PERIPHERALCOUNT][25] = {
  "wireless_servo0",
  "wireless_servo1",
  "wireless_servo2",
  "wireless_servo3",
  "wireless_servo4",
  "wireless_servo5"
}; // local names of the peripherals

// TYPEDEFINITIONS
typedef struct Command {
  String name;
  String data;
};

typedef struct Error {
  bool status;
  String message;
  byte code;
};
// error codes:
// 0: no error
// 1: queue overflow
// 2: bluetooth disconnected
// 5: unknown command
// 6: transmission of command failed

typedef struct Data {
  bool connected;
  Error error;
};

typedef struct Device {
  String address;
  String localname;
  BLEDevice device;
  BLEService service;
  BLECharacteristic rxcharacteristic;
  BLECharacteristic txcharacteristic;
};


// GLOBALS
cppQueue QUEUE(sizeof(char[COMMANDSIZE]), 100, FIFO);
Command QUEUEDCOMMAND;
Device PERIPHERALS[PERIPHERALCOUNT];
unsigned long TIMESTAMP;


void dequeue(cppQueue &queue, Command &command, Error &error) {
  // dequeue command from the global queue
  // returns command "_timeout" if the queue is empty
  // returns command "_error" if error.status is true

  // throw an error if the queue is full
  if (queue.isFull()){
    error.status = true;
    error.code = 1;
    error.message = "queue overflow";
  }

  // output error command if an error is present
  if (error.status) {
    command.name = "_error";
    command.data = error.message;
  }
  // else dequeue command from queue
  else {
    // commands in queue
    if (queue.nbRecs() > 0) {
      // dequeue character array from queue
      char string[COMMANDSIZE];
      queue.pop(&string);

      // deserialize character array to local command variable
      deserializeCommand(command, string);
    }
    else {
      command.name = "_timeout";
      command.data = "";
    }  
  }
}  


void enqueue(cppQueue &queue, String name, String data = "") {
  // enqueue command with name and data into the global queue

  Command command;
  command.name = name;
  command.data = data;

  // serialize command to character array
  char string[COMMANDSIZE];
  serializeCommand(string, command);

  // enqueue character array to queue
  queue.push(&string);
}


void setup() {

  // SERIAL
  // initialize
  Serial.begin(115200);
  Serial.setTimeout(0);
  Serial1.begin(115200);
  Serial1.setTimeout(0);

  // BLUETOOTH 
  // initialize 
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while(true);
  }

  // local name of the arduino
  BLE.setLocalName(LOCALNAME);

  // assign event handlers
  BLE.setEventHandler(BLEDisconnected, disconnectEvent);

  // scan for peripherals
  Serial.println("scanning for peripherals...");
  
  int i = 0;

  while (i < PERIPHERALCOUNT) {
    BLE.scanForName(PERIPHERALNAMES[i]);
    
    PERIPHERALS[i].device = BLE.available();

    if (PERIPHERALS[i].device) {
        BLE.stopScan();

        PERIPHERALS[i].address = PERIPHERALS[i].device.address();
        PERIPHERALS[i].localname = PERIPHERALNAMES[i];

        Serial.println(" found " + PERIPHERALS[i].localname);
        i++;
      }
    delay(10);
  }
  Serial.println("...done");

  // PINS
  // connect LED
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("setup complete");
}


void loop() {

  // initialize state machine variables
  Command command = {"", ""};
  Error error = {false, "", 0};
  Data data = {false, error};

  // enqueue first command
  enqueue(QUEUE, "_initialize");

  while(true){

    // dequeue command from the queue
    dequeue(QUEUE, command, error);

    // call the state machine
    statemachine(command, data, error);

    // handle digital inputs/outputs
    readInputs();
    writeOutputs(command, data);
    
    // process BLE events
    for (int i = 0; i < PERIPHERALCOUNT; i++) {
      PERIPHERALS[i].device.poll();
    }

    // handle serial interfaces
    serialRead(QUEUE);
    serial1Read(QUEUE);
    serialWrite(command, data);
  }
}


void statemachine(Command &command, Data &data, Error &error) {
  // queued state machine
  // command list:
  // name:          | data (string):
  // _initialize    | none
  // _timeout       | none
  // _connect       | none
  // _disconnect    | none
  // _error         | error message
  // _receive       | none
  // send           | none
  // clear          | none
  // reset          | none
  // Note: commands with a "_" should only be enqueued by the state machine itself

  // initialize the data highway
  if (command.name == "_initialize") {   

    // set data variable
    data.connected = false;

    // enqueue connecte state
    enqueue(QUEUE, "_connect");
  }

  // timeout state, when no command is received
  else if (command.name == "_timeout") {
    
    // don't do anything
  }

  // called after _disconnect and _initialize
  else if (command.name == "_connect") { 
 
    data.connected = true;    

    for (int i = 0; i < PERIPHERALCOUNT; i++) {

      // check if peripheral is connected
      if (!PERIPHERALS[i].device.connected()) {

        // if not connected, attempt establishing a connection
        if (PERIPHERALS[i].device.connect()) {

          // discover string service
          PERIPHERALS[i].device.discoverService("7def8317-7300-4ee6-8849-46face74ca2a");
          PERIPHERALS[i].service = PERIPHERALS[i].device.service("7def8317-7300-4ee6-8849-46face74ca2a");

          // get characteristic to receive data
          PERIPHERALS[i].txcharacteristic = PERIPHERALS[i].service.characteristic("7def8317-7301-4ee6-8849-46face74ca2a");
          PERIPHERALS[i].rxcharacteristic = PERIPHERALS[i].service.characteristic("7def8317-7302-4ee6-8849-46face74ca2a");

          // add and subcribe receive handler
          PERIPHERALS[i].txcharacteristic.setEventHandler(BLEWritten, receiveEvent);
          PERIPHERALS[i].txcharacteristic.subscribe();
        }
        else {
          data.connected = false;
        }
      }
    }    
    // stay in state until all peripherals are connected
    if (!data.connected) {
      enqueue(QUEUE, "_connect");   
    }
  }

  // called when bluetooth disconnects
  else if (command.name == "_disconnect") {   

    // set data variable and throw an error
    data.connected = false;
    error.status = true;
    error.code = 2;
    error.message = "bluetooth disconnected"; 
  }

  // handle error, called when error.status is true
  else if (command.name == "_error") {
    
    // empty the queue
    QUEUE.flush(); 
    
    // report to data variable that an error has occured
    data.error.status = error.status;
    data.error.message = error.message;
    data.error.code = error.code;
    
    // reset error variable
    error.status = false;
    error.code = 0;
    error.message = "";

    // enqueue connect state
    enqueue(QUEUE, "_connect");
  }

  // send to bluetooth peripheral
  else if (command.name == "send") {
    
    // pass data (which contains the command for the peripheral) to the transmit handler if connected
    if (data.connected) {
      if (!sendData(command.data)) {
        error.status = true;
        error.code = 6;
        error.message = "transmission of command '" + command.data + "' failed";
      }
    }
    // data is discarded when sending failes
  }

  // clear error on the data highway
  else if (command.name == "clear") { 

    // reset error variable of the data variable
    data.error.status = false;
    data.error.code = 0;
    data.error.message = "";
  }

  // receive data from peripheral peripheral
  else if (command.name == "_receive") {
    
    receiveData();
  }

  // reset the arduino
  else if (command.name == "reset") {
    
    Serial.println("resetting arduino");
    NVIC_SystemReset();
  }

  // handle unknown commands
  else {

    // throw error
    error.status = true;
    error.code = 5;
    error.message = "unknown command '" + command.name + "'";
  }
}


void disconnectEvent(BLEDevice peripheral) {
  // callback function, called when bluetooth is disconnected

  // enqueue disconnect state
  enqueue(QUEUE, "_disconnect");
}


void receiveEvent(BLEDevice peripheral, BLECharacteristic characteristic) {
  // callback function, called when bluetooth receives a string message
  
  // enqueue transmit peripheral data state
  enqueue(QUEUE, "_receive");
}


void serialRead(cppQueue &queue) {
  // read character string with start '<' and end marker '>' from serial

  static char serialbuffer[RXCHARACTERISTICSIZE];
  String string;
  static boolean inprogress = false;
  static int i = 0;
  int j;
  char startmarker = '<';
  char endmarker = '>';
  char character;

  while (Serial.available() > 0) {

    character = Serial.read();

    if (inprogress == true) {
      if (character != endmarker) {
        serialbuffer[i] = character;
        i++;
        if (i >= RXCHARACTERISTICSIZE) {
          i = RXCHARACTERISTICSIZE - 1;
        }
      } else {
        // terminate string in serialbuffer and reset variables
        serialbuffer[i] = '\0';
        inprogress = false;
        i = 0;

        // put string into the queue
        string = String(serialbuffer);
        j = string.indexOf("/");
        
        enqueue(queue, string.substring(0,j), string.substring(j+1));
      }   
    } else if (character == startmarker) {
      inprogress = true;
    }
  }
}


void serial1Read(cppQueue &queue) {
  // read character string with start '<' and end marker '>' from serial

  static char serialbuffer[RXCHARACTERISTICSIZE];
  String string;
  static boolean inprogress = false;
  static int i = 0;
  int j;
  char startmarker = '<';
  char endmarker = '>';
  char character;

  while (Serial1.available() > 0) {

    character = Serial1.read();

    if (inprogress == true) {
      if (character != endmarker) {
        serialbuffer[i] = character;
        i++;
        if (i >= RXCHARACTERISTICSIZE) {
          i = RXCHARACTERISTICSIZE - 1;
        }
      } else {
        // terminate string in serialbuffer and reset variables
        serialbuffer[i] = '\0';
        inprogress = false;
        i = 0;

        // put string into the queue
        string = String(serialbuffer);
        j = string.indexOf("/");
        
        enqueue(queue, string.substring(0,j), string.substring(j+1));
      }   
    } else if (character == startmarker) {
      inprogress = true;
    }
  }
}


void serialWrite(Command &command, Data &data) {
  // trigger serial write state according to the set data rate

  static unsigned int j = 0;

  if (millis() / DATARATE > j){
    j++;

    // parse data to string variable 
    String string_serial;

    string_serial = string_serial + "<LOCALNAME:" + LOCALNAME + "\n" +
            "timestamp:" + TIMESTAMP + "\n" +
            "data.connected:" + data.connected + "\n" +
            "data.error.status:" + data.error.status + "\n" +
            "data.error.code:" + data.error.code + "\n" +
            "data.error.message:" + data.error.message + "\n" +
            "command.name:" + command.name + "\n" +
            "command.data:" + command.data + ">\n"; 

    // serial output
    Serial.print(string_serial); 

    // parse data to string variable
    String string_serial1;

    // parse data to string variable
    string_serial1 = string_serial1 + "<localname:" + LOCALNAME + "\n" +
            "timestamp:" + TIMESTAMP + "\n" +
            "connected:" + data.connected + "\n" +
            "error:" + data.error.code + ">\n";

    // serial output
    Serial1.print(string_serial1); 
  }
}


void readInputs() {
  // read digital inputs

  TIMESTAMP = millis();
}


void writeOutputs(Command &command, Data &data) {
  // Built-in LED
  if (data.error.status) {
    digitalWrite(LED_BUILTIN, (millis()/100) % 2);  // LED fast blinking = error
  }  
  else if (data.connected) {
    digitalWrite(LED_BUILTIN, 1); // LED on = connected
  }
  else {
    digitalWrite(LED_BUILTIN, 0); // LED off = disconnected
  }
}


bool sendData(String data) {
  // send data to a bluetooth peripheral
  // data string has the format peripheral/command
  // returns true if the data was transmitted
  
  static byte buffer[RXCHARACTERISTICSIZE]; 
  String command;
  String peripheral;
  int i;

  // split data string into peripheral and command
  i = data.indexOf("/");
  peripheral = data.substring(0,i);
  command = data.substring(i+1);
    
  // get index of the peripheral
  for (int j = 0; j < PERIPHERALCOUNT; j++) {
    if (peripheral == PERIPHERALS[j].localname) {
      i = j;
      break;
    }
  }

  if (i > -1) {
    // copy command into buffer
    command.getBytes(buffer, command.length() + 1);
    // send data via BLE
    return PERIPHERALS[i].rxcharacteristic.writeValue(buffer, command.length());
  }
  
  return false;
}


void receiveData(){
  // receive data from bluetooth peripheral
  
  byte buffer[TXCHARACTERISTICSIZE];

  for (int i = 0; i < PERIPHERALCOUNT; i++) {
    
    // check if new data is available
    if (PERIPHERALS[i].txcharacteristic.valueUpdated()){
      
      // read characteristic value into byte array
      PERIPHERALS[i].txcharacteristic.readValue(buffer, TXCHARACTERISTICSIZE);

      // mark the end of the read characteristic value with a 0
      buffer[PERIPHERALS[i].txcharacteristic.valueLength()] = 0;

      // parse data to string variable
      String string_ble;
    
      string_ble = string_ble + "<localname:" + PERIPHERALS[i].localname + "\n" +
            "timestamp:" + TIMESTAMP + "\n" +
            "mode:" + buffer[0] + "\n" +
            "limitswitch0:" + bool(buffer[1]) + "\n" +
            "limitswitch1:" + bool(buffer[2]) + "\n" + 
            "connected:" + bool(buffer[3]) + "\n" +
            "running:" + bool(buffer[4]) + "\n" +
            "error:" + buffer[5] + "\n" +
            "closed:" + bool(buffer[6]) + ">\n";

      Serial1.print(string_ble);
      Serial.print(string_ble);
    }
  }
}


void serializeCommand(char* string, Command &command) {
  // serialize command into character array

  // convert command name and data into character array
  char name_string[COMMANDSIZE];
  char data_string[COMMANDSIZE];
  command.name.toCharArray(name_string, COMMANDSIZE);
  command.data.toCharArray(data_string, COMMANDSIZE);

  // combine into single character array
  strcpy (string, name_string);
  strcat (string, "/");
  strcat (string, data_string);
  // Note: Characters beyond the size of the character array will be cut off
}


void deserializeCommand(Command &command, char* string) {
  // deserialize character array to command
  
  // convert character array to string object
  String command_string;
  command_string = String(string);
  
  // split string object into command name and data
  int i = command_string.indexOf("/");
  command.name = command_string.substring(0,i);
  command.data = command_string.substring(i+1);
}