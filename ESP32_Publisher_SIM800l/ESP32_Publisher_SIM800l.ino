// MCP2515 Connection with ESP32
/*
 * MCP2515  = WEMOS ESP32
 * VCC      =       5V
 * GND      =      GND
 * CS       =       P5
 * SO       =      P19
 * SI       =      P23
 * SCK      =      P18
 * INT      =      P22
 */
//Neo 6m GPS Connection with ESP32
 /*
  *  Neo GPS = ESP32
  * VCC      =     3V
  * GND      =    GND
  * RX       =   17(TX2) 
  * TX       =   16(RX2)  
 */

//SIM800l Connection with ESP32
 /*
  *  SIM800l = ESP32
  * VCC      =     5V (External Battery)
  * GND      =    GND (External Battery)
  * RX       =   26 
  * TX       =   25  
  * GND      =    GND (ESP32)
 */
#define TINY_GSM_MODEM_SIM800 // Define SIM Module
//Libraries to include 
#include <WiFi.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <mcp_can.h>  // https://github.com/coryjfowler/MCP_CAN_lib
#include <SPI.h>
#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
#include <TinyGsmClient.h> //Include TinyGSM Library

/*GPS init*/
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(2);
TinyGPSPlus gps;
/*End GPS init*/

long unsigned int rxId; // for CAN Packet ID
unsigned char len = 0; // Length of CAN Packet
unsigned char rxBuf[8]; // Default Buffer Size


#define CS 5  // Define CS PIN
MCP_CAN CAN0(CS);  // Initialize MCP_CAN                              

//WiFi Connection
const char* ssid = "SecureTechware2"; 
const char* password = "$7001123";
//MQTT Server Connection
#define mqtt_server "broker.hivemq.com"
#define owner "ArsalanSaleemSecureTechwarePublisher_3_2"
#define mqtt_user "ArsalanSaleemSecureTechwarePublisher_3_2"
#define mqtt_password "ArsalanSaleemSecureTechware_3_2"
#define status_topic "SecureTechware_MCP2515_Status_3_2"
#define gps_data_topic "SecureTechware_GPS_Data_3_2"
#define data_topic "SecureTechware_MCP2515_Data_3_2"
#define mqtt_port 1883
HardwareSerial SerialGSM(1); //serial channel we are going to use to communicate with the modem is channel 1
TinyGsm modemGSM(SerialGSM);
TinyGsmClient gsmClient(modemGSM);

PubSubClient client(mqtt_server, mqtt_port, gsmClient); //GSM Connection with PubSubClient
//Variable sent over MQTT Server
String SendData;



void mqttconnect() {   // Loop until reconnected 
  while (!client.connected()) {
    Serial.print("connecting to MQTT Server ...");
    /* connect now */
    if (client.connect( owner, mqtt_user, mqtt_password, status_topic, 2, true, "offline")) {
      client.publish(status_topic, "Publisher is Connected!", true);
      Serial.println("Publisher is Connected!");
    } else {
      Serial.print("Failed, status code =");
      Serial.print(client.state());
      Serial.println("Try again in 5sec ...");  // Wait 5 seconds before retrying
      delay(3000);
    }
  }
}

void setupGSM(){
  //Inicializamos a serial onde está o modem
  SerialGSM.begin(9600, SERIAL_8N1, 25, 26, false);
  delay(2000);

  //Mostra informação sobre o modem
  Serial.println(modemGSM.getModemInfo());

  //Inicializa o modem
  if (!modemGSM.restart())
  {
    
    Serial.println("Restarting GSM Modem failed");
    delay(1000);
    ESP.restart();
    return;
  }

  //Espera pela rede
  if (!modemGSM.waitForNetwork()) 
  {
    Serial.println("Failed to connect to network");
    delay(3000);
    ESP.restart();
    return;
  }

  //Conecta à rede gprs (APN, usuário, senha)
  if (!modemGSM.gprsConnect("", "", "")) {
    Serial.println("GPRS Connection Failed");
    delay(1000);
    ESP.restart();
    return;
  }

  Serial.println("Setup GSM Success");
}

void setup()
{
  Serial.begin(115200); //baud rate 

 // setupGSM(); //initialize and configure a GSM modem
 // mqttconnect();

  pinMode(CS, OUTPUT);  
  // Initialize MCP2515 running at MCP_8MHZ or MCP_16MHZ with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(22, INPUT);                            // Setting pin 22 for /INT input
  
  Serial.println("ESP32 --> MCP2515 --> MQTT Initialization. Complete Publisher Successed.");

  /*GPS Setup*/
   neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop()
{
    /* if MQTT client was disconnected then try to reconnect again */
    if (!client.connected()) {
          Serial.print("MQTT reconnecting ...");
          
          setupGSM();
          
          mqttconnect();
    } 

    String DATENSATZ;  //Temp Variable
    
    if(!digitalRead(22))                        // If pin 22 is low, read receive buffer
    {
      CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
     
      DATENSATZ+=String(rxId,HEX).c_str(); //Set ID
      DATENSATZ+=String("#").c_str();

      for(int i=0;i<len;i++){
        DATENSATZ+=String(rxBuf[i],HEX).c_str(); //Set CAN Frame
        DATENSATZ+=" ";
      }

      if(SendData!=DATENSATZ){
            SendData=DATENSATZ;
            Serial.println(SendData); 
            client.publish(data_topic, String(SendData).c_str(), true); // MQTT-Server send
      }
    }
  /*GPS data*/  
  boolean newData = false;
  String Location="";
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  //If newData is true
  if(newData == true)
  {
    newData = false;
    Serial.println(gps.satellites.value());
   // print_speed();
       if (gps.location.isValid() == 1){
        Location= "Lat: "+ String(gps.location.lat()) + " "+ "Long: "+ String(gps.location.lng()) + " " + "Speed km/h: " + String(gps.speed.kmph()) + " "+ "Satellite: "+ String(gps.satellites.value()) + "ALT: "+ String(gps.altitude.meters());
       }else{
          Location="GPS Data Not Available";
       }
  
  client.publish(gps_data_topic, String(Location).c_str(), true); // MQTT-Server send

  }

   /*End GPS data*/   
   

}



