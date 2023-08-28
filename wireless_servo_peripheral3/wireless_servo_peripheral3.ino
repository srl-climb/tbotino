#include<cppQueue.h>
#include<ArduinoBLE.h>
#include<Servo.h>

// SOURCES
// https://github.com/tigoe/BluetoothLE-Examples/blob/main/ArduinoBLE_library_examples/BLE_String/BLE_String.ino
// https://arduino.stackexchange.com/questions/24033/proper-use-of-and-when-passing-objects-in-methods
// https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
// https://forum.arduino.cc/t/solved-upload-starts-but-fails/937150/3

// PARAMETERS
const char LOCALNAME[25] = "wireless_servo1"; // name of the Arduino as a bluetooth device, max. number of characters: 24
const byte MOTORPIN = 5; // digital output pin for the servo pwm signal
const byte MOTORMODE = 0; // servo motor mode (0: intermittend, 1: continous without limit switch, 2: continous with limit switch)
const byte MOTORVAL0 = 165; // open value of the servo (angle in degrees/speed (0: full speed one direction, 90: no movement, 180: full speed in other direction)
const byte MOTORVAL1 = 20; // close value of the servo (angle in degrees/speed (0: full speed one direction, 90: no movement, 180: full speed in other direction)
const int MOTORTIMEOUT = 1000; // Timeout in ms (time the Arduino waits to open and close; in mode 2, if the timeout is reached before the limitswitch an error is thrown)
const byte LIMITSWITCH0PIN = 2; // digital input pin of the open limit switch
const byte LIMITSWITCH1PIN = 3; // digital input pin of the close limit switch
const int RXCHARACTERISTICSIZE = 64; // size of the incoming strings via bluetooth/serial
const int TXCHARACTERISTICSIZE = 8; // size of the outgoing strings via bluetooth
const int COMMANDSIZE = 64; // size of the command strings, should be the same as RXCHARACTERISTICSIZE
const int DATARATE = 500; // ms after which a data packages are send via serial

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
// 3: unable to parse 'start' command data
// 4: timeout when waiting for motor in mode 2
// 5: unknown command

typedef struct Motor {
  int pin;
  Servo interface;
  byte mode;
  byte val0;
  byte val1;
  byte val;
  int timeout;
};

typedef struct Data {
  bool connected;
  bool running;
  byte closed; //0: open, 1: closed, 2: unknown
  unsigned long starttime;
  Error error;
};

typedef struct Limitswitch {
  int pin;
  bool state;
};

// GLOBALS
cppQueue QUEUE(sizeof(char[COMMANDSIZE]), 100, FIFO);
Motor MOTOR = {MOTORPIN, Servo(), MOTORMODE, MOTORVAL0, MOTORVAL1, 90, MOTORTIMEOUT};
Limitswitch LIMITSWITCH0 = {LIMITSWITCH0PIN, false}; 
Limitswitch LIMITSWITCH1 = {LIMITSWITCH1PIN, false};
unsigned long TIMESTAMP;

// BLUETOOTH SERVICES AND CHARACTERISICS
BLEService stringService("7def8317-7300-4ee6-8849-46face74ca2a");
BLECharacteristic txCharacteristic("7def8317-7301-4ee6-8849-46face74ca2a", BLERead | BLENotify, TXCHARACTERISTICSIZE); 
BLECharacteristic rxCharacteristic("7def8317-7302-4ee6-8849-46face74ca2a", BLEWrite, RXCHARACTERISTICSIZE);


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
  // enqueue command with name and data into the queue

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
  
  // BLUETOOTH 
  // initialize 
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while(true);
  }
  
  // local/device name of the arduino
  BLE.setLocalName(LOCALNAME);
  //

  // set the UUID for the service the arduino advertises
  BLE.setAdvertisedService(stringService);

  // add the characteristics to the service
  stringService.addCharacteristic(txCharacteristic);
  stringService.addCharacteristic(rxCharacteristic);
  
  // assign event handlers
  BLE.setEventHandler(BLEConnected, connectEvent);
  BLE.setEventHandler(BLEDisconnected, disconnectEvent);
  rxCharacteristic.setEventHandler(BLEWritten, receiveEvent);

  // add the service and set a value for the characteristic:
  BLE.addService(stringService);
  
  // start advertising
  BLE.advertise();

  // PINS
  // connect LED
  pinMode(LED_BUILTIN, OUTPUT);

  // switches
  pinMode(LIMITSWITCH0.pin, INPUT);
  pinMode(LIMITSWITCH1.pin, INPUT);

  // motor
  MOTOR.interface.attach(MOTOR.pin);

  Serial.println("setup complete");
}


void loop() {

  // initialize state machine variables
  Command command = {"", ""};
  Error error = {false, "", 0};
  Data data = {false, false, false, 0, error};
  
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
    BLE.poll();

    // handle serial interfaces
    serialRead(QUEUE);
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
  // _wait          | "open", "close"
  // clear          | none
  // start          | "open", "close"
  // stop           | none
  // reset          | none
  // Note: commands with a "_" should only be enqueued by the state machine itself

  // initialize the data highway
  if (command.name == "_initialize") {   

    // set data variable
    data.connected = false;
    data.running = false;
    data.closed = 2; // unknown state
    data.starttime = 0;

    // intermittend servo -> move to closed position
    if (MOTOR.mode == 0) {
      enqueue(QUEUE, "start","close");
    }
    // continous servo without limit switches -> do nothing
    else if (MOTOR.mode == 1) {
      data.closed = 2; // unkown state
    }
    // continous servo with limit switches -> move to closed position if not closed
    else if (MOTOR.mode == 2) {
      if (!LIMITSWITCH1.state) {
        enqueue(QUEUE, "start","close");
      }
      else {
        data.closed = 1; // closed state
      }
    }
  } 

  // timeout state, when no command is received
  else if (command.name == "_timeout") {
    // don't do anything
  }

  // called when bluetooth connects
  else if (command.name == "_connect") {     

    // set data variable
    data.connected = true;
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
    // make sure that running motors are stopped
    enqueue(QUEUE, "stop");
    
    // report to data variable that an error has occured
    data.error.status = error.status;
    data.error.message = error.message;
    data.error.code = error.code;
    
    // reset error variable
    error.status = false;
    error.code = 0;
    error.message = "";
  }

  // clear error on the data highway
  else if (command.name == "clear") { 

    // reset error variable of the data variable
    data.error.status = false;
    data.error.code = 0;
    data.error.message = "";
  }

  // start the servo
  else if (command.name == "start") {
    
    // only start when no error has occured
    if (!data.error.status) {      
      // if the motor is still running, first stop it
      if (data.running) { 
        enqueue(QUEUE, "stop");
        enqueue(QUEUE, command.name, command.data);
      }   
      // else start the motor
      else {
        // set data variables   
        data.running = true;
        data.closed = 2;
        data.starttime = millis();      
        // write to the motor interface and enqueue "_wait" state     
        if (command.data == "open") {
          MOTOR.interface.write(MOTOR.val0);
          enqueue(QUEUE, "_wait", command.data);
        }        
        else if (command.data == "close") {
          MOTOR.interface.write(MOTOR.val1);  
          enqueue(QUEUE, "_wait", command.data);   
        }      
        // throw an error if unkown command data was received
        else {
          error.status = true;
          error.code = 3;
          error.message = "unable to parse 'start' command data";
        }         
      }  
    }
  }

  // wait for the servo to finish
  else if (command.name == "_wait") {
    
    // only wait if servo is running (it might have been stopped by the user)
    if (data.running) {  
      
      // intermittend servo or continous servo without limit switches   
      if (MOTOR.mode == 0 || MOTOR.mode == 1) {
        // stop if the timeout is reached
        if (millis()-data.starttime>MOTOR.timeout) {
          enqueue(QUEUE, "stop", command.data);
        } 
        else {
          enqueue(QUEUE, "_wait", command.data);
        }
        // Note: Also for the intermittend servo we want to wait a little,
        // since it will need some time to reach its final position
      } 
      
      // continous servo with limit switches
      else if (MOTOR.mode == 2) {
        // stop if the limit switch is reached
        if (command.data == "open") {
          if (LIMITSWITCH0.state) {
            enqueue(QUEUE, "stop", "open");
          } 
          else {
            enqueue(QUEUE, "_wait", "open");
          }
        } 
        else if (command.data == "close") {
          if (LIMITSWITCH1.state) {
            enqueue(QUEUE, "stop", "close");
          } 
          else {
            enqueue(QUEUE, "_wait", "close");
          }
        }
        // throw an error if the timeout is reached
        if (millis()-data.starttime>MOTOR.timeout) {
            error.status = true;
            error.code = 4;
            error.message = "timeout when waiting for motor in mode 2"; 
        }
      } 
    }
  }

  // stop the servo
  else if (command.name == "stop") {

    // stop the servo only if it is running
    if (data.running) {
      // set data variable
      data.running = false;
      if (command.data == "close") {
        data.closed = 1;
        }
      else if (command.data == "open") {
        data.closed = 0;
      }
      // Note: if the user calls the stop state, command.data is "", so that
      // data.closed remains 2 (unkown state)
      
      // stop continous servo
      if (MOTOR.mode == 1 || MOTOR.mode == 2) {
        MOTOR.interface.write(90);
      }
    }
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


void connectEvent(BLEDevice central) {
  // callback function, called when bluetooth is connected

  enqueue(QUEUE, "_connect");
}


void disconnectEvent(BLEDevice central) {
  // callback function, called when bluetooth is disconnected
  
  enqueue(QUEUE, "_disconnect");
}


void receiveEvent(BLEDevice central, BLECharacteristic characteristic) {
  // callback function, called when bluetooth receives a string message

  byte buffer[RXCHARACTERISTICSIZE];
  String string;
  char* data = (char*)buffer; // char pointer, pointing to the buffer
  byte i;

  // read characteristic value into byte array
  rxCharacteristic.readValue(buffer, RXCHARACTERISTICSIZE);

  // mark the end of the read characteristic value with a 0
  buffer[rxCharacteristic.valueLength()] = 0;
  
  string = String(data);
  i = string.indexOf("/");

  enqueue(QUEUE, string.substring(0,i), string.substring(i+1));
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


void serialWrite(Command &command, Data &data) {
  // trigger serial write state according to the set data rate

  static unsigned int j = 0;

  if (millis() / DATARATE > j){
    j++;

    // transmit data via serial, bluetooth

    // parse data to string variable
    String string_serial;
    
    string_serial = string_serial + "<LOCALNAME:" + LOCALNAME + "\n" +
            "timestamp:" + TIMESTAMP + "\n" +
            "MOTOR.pin:" + MOTOR.pin + "\n" +
            "MOTOR.mode:" + MOTOR.mode + "\n" +
            "MOTOR.val0:" + MOTOR.val0 + "\n" +
            "MOTOR.val1:" + MOTOR.val1 + "\n" +
            "MOTOR.val:" + MOTOR.val + "\n" +
            "MOTOR.timeout:" + MOTOR.timeout + "\n" +
            "LIMITSWITCH0.pin:" + LIMITSWITCH0.pin + "\n" +
            "LIMITSWITCH0.state:" + LIMITSWITCH0.state + "\n" +
            "LIMITSWITCH1.pin:" + LIMITSWITCH1.pin + "\n" +
            "LIMITSWITCH1.state:" + LIMITSWITCH1.state + "\n" + 
            "data.connected:" + data.connected + "\n" +
            "data.running:" + data.running + "\n" +
            "data.error.status:" + data.error.status + "\n" +
            "data.error.code:" + data.error.code + "\n" +
            "data.error.message:" + data.error.message + "\n" +
            "data.closed:" + data.closed + "\n" +
            "command.name:" + command.name + "\n" +
            "command.data:" + command.data + ">\n"; // about 500 bytes long
    
    // serial output
    Serial.print(string_serial);
    
    // parse data to byte buffer
    static byte buffer[TXCHARACTERISTICSIZE];

    buffer[0] = MOTOR.mode;
    buffer[1] = byte(LIMITSWITCH0.state);
    buffer[2] = byte(LIMITSWITCH1.state);
    buffer[3] = byte(data.connected);
    buffer[4] = byte(data.running);
    buffer[5] = data.error.code;
    buffer[6] = byte(data.closed);
    
    // send via bluetooth
    txCharacteristic.writeValue(buffer, TXCHARACTERISTICSIZE);
  }
}


void readInputs() {
  // read digital inputs, IMU

  TIMESTAMP = millis();

  // limit switches
  LIMITSWITCH0.state = digitalRead(LIMITSWITCH0.pin);
  LIMITSWITCH1.state = digitalRead(LIMITSWITCH1.pin);

  // servo motor
  MOTOR.val = MOTOR.interface.read();

}


void writeOutputs(Command &command, Data &data) {

  // Built-in LED
  if (data.error.status) {
    digitalWrite(LED_BUILTIN, (millis()/100) % 2);  // LED fast blinking = error
  } 
  else if (!data.running && !data.error.status) {
    digitalWrite(LED_BUILTIN, 1); // LED on = no error, idle
  } 
  else if (data.running && !data.error.status) {
    digitalWrite(LED_BUILTIN, (millis()/250) % 2); // LED blinking = no error, moving servo
  } 
  else 
  {
    digitalWrite(LED_BUILTIN, 0); // LED off = undefined state
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
