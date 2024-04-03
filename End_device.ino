#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <EEPROM.h>
#include <semphr.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <avr/sleep.h>

//LoR32u4II 868MHz or 915MHz (black board)
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    869.3E6  // 915E6 //868E6 //869300000Hz
#define PABOOST true 


typedef struct Data_t
{
    int temperature;
    int voltage;
    int wakeTime;
} GenericData_t;


SemaphoreHandle_t xMutex;

int msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBE;     // address of this device
byte destination = 0xBC;      // destination to send to
long lastSendTime = 0;        // last send time
String incoming = "";

int readVolt()
{
  int rawValue = analogRead(A0);
  float voltage = rawValue * (3.3/ 1023); //scale voltage to range  of arduino, Vmin = 0.1
  //Serial.print("Voltage solar panel: ");
  //Serial.print(voltage); // 0.02 V delay
  //Serial.println(" V");
  //delay(1000);
  return voltage*1000;
}

int readTemp()
{
  unsigned int wADC; //store initial temp
  int temp; //temp after correction
  ADCSRA |= _BV(ADSC); // start the conversion; _BV = 1<<(bit); Bit 6 – ADSC; 
  while (bit_is_set(ADCSRA, ADSC)); // ADSC is cleared when the conversion finishes, wait till conversion is done; first conversion 25 clk cycles after 13
  wADC = ADCL | (ADCH << 8); //result in ADCH & ADCL; shift ADCH & sum registers; ADC register; test ADCW
  temp = wADC;
  float T_off = -8; //offset
  float k = 1; //one point callibation, offset compensation
  temp = (wADC-273 - T_off)*k; 
  return temp; 
}

void pendingFunction(void* pvParameters)
{ 
  int interval;
  while(1) {
    
    if(msgCount < 20) {
      if(Serial.available() > 0) {
        executeCommand();        
      }
      incoming = "";
      while(incoming.equals("")){
        receiveBeacon(LoRa.parsePacket());
      }
    
      int temp = readTemp();        
      int volt = readVolt();
      char message[15];
      sprintf (message, "%d_%d", temp, volt);
      sendBeacon(message);
      writeData(msgCount, temp, volt, interval);
      msgCount++;

      delay(interval * 1000);
    } else {
      Serial.println("Sleeping");
      //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      //sleep_mode();
    }
  }
}

int receiveBeacon(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }


  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.print("This message is not for me.");
    Serial.print(localAddress);    
    return;                             // skip rest of function
  }

  String teamId = incoming.substring(2, 4);
  if(!teamId.equals("17")){
    return;
  }

  int interval = (incoming.substring(4,incomingLength)).toInt();
  return interval;
}

void sendBeacon(String outgoing)
{
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(msgCount);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
}

// Read Temp & Volt Solar Panel
void readData(GenericData_t* data, uint8_t index) {
  xSemaphoreTake(xMutex, portMAX_DELAY);
  data->temperature =  EEPROM.read((index * 3) * sizeof(int));
  Serial.println(data->temperature);
  data->voltage =  EEPROM.read((index * 3 + 1) * sizeof(int));
  data-> wakeTime = EEPROM.read((index * 3 + 2)* sizeof(int));
  xSemaphoreGive(xMutex);
}

// Write temp & volt solar & wakeup time panel to database
void writeData(int index, int temp, int volt, int wakeTime)
{
  xSemaphoreTake(xMutex, portMAX_DELAY);
  Serial.println(temp);
  EEPROM.write((index * 3) * sizeof(int), temp);
  Serial.println(EEPROM.read((index * 3) * sizeof(int)));
  EEPROM.write((index* 3 + 1) * sizeof(int), volt);
  EEPROM.write((index* 3 + 2) * sizeof(int), wakeTime);
  xSemaphoreGive(xMutex);
}

  void executeCommand() {
    String serialCommand = Serial.readString();
    serialCommand.trim();
    
    if(serialCommand.equals("1")) {
      xTaskCreate(
        command1, // Task function
        "command1", // Task name
        configMINIMAL_STACK_SIZE, // Stack size
        NULL, // Task parameter
        2, // Task priority
        NULL
      ); // Task handle
    }

    if(serialCommand.equals("2")){
      xTaskCreate(
        command2, // Task function
        "command2", // Task name
        configMINIMAL_STACK_SIZE, // Stack size
        NULL, // Task parameter
        2, // Task priority
        NULL
      ); // Task handle
    }

    if(serialCommand.equals("3")){
      xTaskCreate(
        command3, // Task function
        "command3", // Task name
        configMINIMAL_STACK_SIZE, // Stack size
        NULL, // Task parameter
        2, // Task priority
        NULL
      ); // Task handle
    }
  }

void command1() {
  GenericData_t data;
  readData(&data, msgCount-1);
  printData(data);
  vTaskDelete( NULL );
}

void command2() {
  GenericData_t data;
  for(int i = 0; i < msgCount; i++){
    readData(&data,i);
    Serial.print(i);
    Serial.print(" : ");
    printData(data);
  }
  vTaskDelete( NULL );
}

void command3() {
  Serial.println("Sleeping");
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //sleep_mode();
  vTaskDelete( NULL );
}

void printData(GenericData_t data) {
  Serial.print(data.temperature);
  Serial.print(" ");
  Serial.print(data.voltage);
  Serial.print(" ");
  Serial.println(data.wakeTime);
}

void setup()
{
  // Initialize the serial port
  Serial.begin(9600);
 
  while(!Serial);  // Wait for Serial terminal to open port before starting program
 
  Serial.println("");
  Serial.println("******************************");
  Serial.println("        Program start         ");
  Serial.println("******************************");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(SS,RST,DI0);// set CS, reset, IRQ pin

  if (!LoRa.begin(BAND,PABOOST)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  Serial.println("LoRa init succeeded.");
  xMutex = xSemaphoreCreateMutex();
  set_sleep_mode(SLEEP_MODE_IDLE);
  
  // Setup read temp
  ADMUX = 0xC7; // 11 0 00111 ; turn on internal reference 2.56V, right-shift ADC buffer, ADC channel = internal temp sensor 100111 (bit 5 in ADCSRB)
  delay(10);  // wait a sec for the analog reference to stabilize
  ADCSRB = 0x20; // 0 0 1 0 0000 ; High speed mode off, ADMUX extra bit set temperature, n/a, Free Running mode.
  delay(10);  // wait for stabilization

  // Create the task to create other tasks based on serial input
  xTaskCreate(
    pendingFunction, // Task function
    "pendingFunction", // Task name
    configMINIMAL_STACK_SIZE, // Stack size
    NULL, // Task parameter
    1, // Task priority
    NULL); // Task handle

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop()
{
  // This function is left empty, since all tasks are handled by the FreeRTOS scheduler
}
