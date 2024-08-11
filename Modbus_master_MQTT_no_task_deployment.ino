#include <ModbusMaster.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char* ssid     = "Swift Dynamics";
const char* password = "swd12345";

// MQTT
const char* mqtt_broker = "18.140.161.60"; 
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/powermeter";

WiFiClient espClient;
PubSubClient client(espClient);

// RS485 
#define RXD2 16
#define TXD2 17

#define MAX485_DE      5
#define MAX485_RE_NEG  18

unsigned long last_time = 0;

ModbusMaster node; 


void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}


void postTransmission(){
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}


// Check communication status
bool getResultMsg(ModbusMaster *node, uint8_t result) 
{
  String tmpstr2 = "\r\n";
  switch (result) 
  {
  case node->ku8MBSuccess:
    return true;
    break;
  case node->ku8MBIllegalFunction:
    tmpstr2 += "Illegal Function";
    break;
  case node->ku8MBIllegalDataAddress:
    tmpstr2 += "Illegal Data Address";
    break;
  case node->ku8MBIllegalDataValue:
    tmpstr2 += "Illegal Data Value"; // Wrong quantity in code
    break;
  case node->ku8MBSlaveDeviceFailure:
    tmpstr2 += "Slave Device Failure"; 
    break;
  case node->ku8MBInvalidSlaveID:
    tmpstr2 += "Invalid Slave ID";
    break;
  case node->ku8MBInvalidFunction:
    tmpstr2 += "Invalid Function";
    break;
  case node->ku8MBResponseTimedOut:
    tmpstr2 += "Response Timed Out"; // Wrong slave ID in Modbus slave
    break;
  case node->ku8MBInvalidCRC:
    tmpstr2 += "Invalid CRC";
    break;
  default:
    tmpstr2 += "Unknown error: " + String(result);
    break;
  }
  Serial.println(tmpstr2);
  return false;
}

void reconnect() {
  while (!client.connected()) {
    Serial.println(F("Attempting MQTT connection..."));
    if (client.connect(mqtt_broker)) {
      Serial.println(F("connected"));
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      delay(5000);
    }
  }
}

void setup()
{
  // WiFi -------------------------------------------
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(WiFi.status());
    Serial.print(F("."));
  }

  // MQTT -------------------------------------------
  client.setServer(mqtt_broker, mqtt_port);

  // RS485 ------------------------------------------
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud
  Serial.begin(115200);

  // Slave device send data at 9600 (the value I configured in Mosbus Slave)
  // SERIAL_8N1 means 8 data bits, no parity, and 1 stop bit
  // We use Serial2 because the MAX485 module is connected to  RX2 and TX2 pins of ESP32
  Serial2.begin(9600,SERIAL_8N1, RXD2, TXD2);
  
  // Modbus slave ID 2, using RX2, TX2 as comm port
  node.begin(1, Serial2);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

}

void loop()
{ 
  uint8_t result = 0;
  uint16_t data[60]; // array เก็บ data 60 ตัว

  // Poll data in register every 5 seconds
  if(millis()- last_time > 5000){

    result = node.readHoldingRegisters(0x40002, 60);
    
  // เท่าที่ลอง มันน่าจะอ่านค่าได้ไม่จำกัด register เลย ถ้างั้นอ่านค่ารวดเดียวเก็บใส่ array แล้วมาคัดเอาทีหลังอาจจะดีกว่า
    if (getResultMsg(&node, result)) {

      Serial.println("Read data from Slave");
   
      // ค่าที่ได้ (result) จะ return กลับมาเป็น array จึงต้องใช้ loop เพื่ออ่านข้อมูลออกมาทีละตัว
      for (int j = 0; j < 60; j++){
        data[j] = node.getResponseBuffer(j); // ข้อมูลที่ดึงมาได้จาก register ของ Modbus device
        Serial.println(data[j]); // แสดงค่าที่ ESP32 poll มาได้บน Serial monitor
      }
    }
    
    last_time = millis();

  }

  // ต้องแปลงค่าที่อ่านได้ที่เก็บใน register จาก uint16_t (unsigned 16-bit integer) เป็น float ก่อน แม้เราจะหารร้อยไปมันก้ปัดให้เราเป็นจำนวนเต็มอยู่ดี ซึ่งทำให้ค่าไม่ละเอียดเลย
  float L1_phase_voltage = ((float)data[1])/100;
  float L2_phase_voltage = ((float)data[3])/100;
  float L3_phase_voltage = ((float)data[5])/100;

  float L3_current = ((float)data[11])/10000;

  float L12_voltage = ((float)data[13])/100;
  float L23_voltage = ((float)data[15])/100;
  float L31_voltage = ((float)data[17])/100;

  float L3_power_factor = ((float)data[41])/10000;

  float frequency = ((float)data[49])/1000;

  char JSONdata[500]; // ถ้าไม่พอ จะขึ้น Stack smashing protect failure! ใน serial monitor
  const char JSONtemplate[] = "{\"L1_phase_voltage\": %.2f,"
                            "\"L2_phase_voltage\": %.2f," 
                            "\"L3_phase_voltage\": %.2f,"
                            "\"L3_current\": %.2f," 
                            "\"L12_voltage\": %.2f,"
                            "\"L23_voltage\": %.2f," 
                            "\"L31_voltage\": %.2f,"
                            "\"L3_power_factor\": %.2f," 
                            "\"frequency\": %.2f}" ;

 
  sprintf(JSONdata, JSONtemplate, L1_phase_voltage, L2_phase_voltage, L3_phase_voltage, L3_current, L12_voltage, L23_voltage, L31_voltage, L3_power_factor, frequency );

  //Make sure we stay connected to the mqtt broker
  if (!client.connected()) {
    reconnect();
  }
  if (!client.loop()) {
    client.connect(mqtt_broker);
  }

  // Publish data to MQTT broker every minute
  if (client.connected()) {
    client.publish(mqtt_topic, JSONdata);
  } else {
    reconnect();
  }

  delay(60000); 
}

