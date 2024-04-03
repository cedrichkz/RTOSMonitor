#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    869.3E6   // 915E6
#define PABOOST true


long wait;
int nbSending = 0;
int nbReceived = 0;

byte localAddress = 0xBC;     // address of this device
byte destination = 0xBE;      // destination to send to
SemaphoreHandle_t xMutex;
String incomingMessage;

typedef struct Data_t
{
    int temperature;
    int voltage;
    int wake;
} GenericData_t;



void pendingFunction(void *pvParameters) {
  while(1) {
    if(nbSending == 20) {
      Serial.print("Number sending packets : ");
      Serial.println(nbSending);
      Serial.print("Number received acks : ");
      Serial.println(nbReceived);
      GenericData_t data;
      for(int i = 0; i < nbReceived; i++) {
        readData(&data,i);
        Serial.print(data.temperature); 
        Serial.print(" "); 
        Serial.print(data.voltage);
        Serial.print(" "); 
        Serial.println(data.wake);    
      }
      
      nbSending = 0;
      nbReceived = 0;
      wait = 1000; 
    }
  
    delay(wait);
    
    wait = random(2,11);
    char message[40];
    sprintf(message, "GW17%ld",wait);
    Serial.print("Sending beacon ");
    Serial.println(message);
    sendMessage(message);
    wait = wait * 1000;
    incomingMessage = "";

    // wait for the answer or wait 1s max
    unsigned long time = millis();
    while(incomingMessage.equals("") && millis() - time < 1000) {
      onReceive(LoRa.parsePacket());  
    }
  }  
}

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  xMutex = xSemaphoreCreateMutex();
  set_sleep_mode(SLEEP_MODE_IDLE);

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(SS,RST,DI0);// set CS, reset, IRQ pin

  if (!LoRa.begin(BAND,PABOOST)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true); 
  } 


  Serial.println("Start Gateway");

  xTaskCreate(
    pendingFunction, // Task function
    "pendingFunction", // Task name
    configMINIMAL_STACK_SIZE, // Stack size
    NULL, // Task parameter
    1, // Task priority
    NULL); // Task handle
    
    wait = 100;
    vTaskStartScheduler();
}


void sendMessage(String outgoing) {
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(nbSending);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
  nbSending++;
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  incomingMessage = "";
  while(LoRa.available()){
    incomingMessage += (char)LoRa.read();
  }

  if (incomingLength != incomingMessage.length()) {
    Serial.println("error: message length does not match length");
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.print("This message is not for me. ");
    Serial.println(recipient);
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  String stemp = incomingMessage.substring(0,incomingMessage.indexOf("_"));
  int temp = stemp.toInt();
  int vol = incomingMessage.substring(stemp.length()+1,incomingMessage.length()).toInt();
  writeData(nbReceived,temp, vol); 
  nbReceived++; 

  // Print the incoming message values
  Serial.print(temp);
  Serial.print(" ");
  Serial.println(vol);
}

void writeData(int index, int temperature, int voltage) {
  xSemaphoreTake(xMutex, portMAX_DELAY);
  EEPROM.write((index * 2) * sizeof(int), temperature);
  EEPROM.write((index * 2 + 1) * sizeof(int), voltage);
  xSemaphoreGive(xMutex); 
}

void readData(GenericData_t* data, uint8_t index) {
  xSemaphoreTake(xMutex, portMAX_DELAY);
  data->temperature =  EEPROM.read((index * 2) * sizeof(int));
  data->voltage =  EEPROM.read((index * 2 + 1) * sizeof(int));
  xSemaphoreGive(xMutex);
}

void loop() {

}
