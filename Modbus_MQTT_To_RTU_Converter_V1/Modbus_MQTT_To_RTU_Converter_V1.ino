/* 
 *  Modbus MQTT to RTU Converter V1.0 developed by Somsak Elect on 24 Apr 2020
 *  Main code from Modbus TCP to RTU Converter V9.0
 *  
 *  Feature:
 *  1. Hold press 'Clear BTN' for reset to AP mode
 *  2. Modbus TCP(Slave) and Modbus RTU(Master) 
 *  3. Setting via web page (AP mode: 192.168.4.1) or MB Converter Settings (VB app)
 *  4. Max. 125 registers for executing with Modbus function #3
 *  
 *  LED:
 *  - ON = First power in
 *  - OFF = Received data from client (Master) and Modbus frame is verified.
 *  - ON = Received data from serial (Slave) and Modbus frame is verified.
 *  
 *  Support board:
 *  1. AIR-WIFI-ES00 for RS-485 must set DR_Pin = 5, TTL must set DR_Pin = 4 and remove RS-485 chip (in case use via CN2)
 *  2. AIR-WIFI-R0 must set DR_Pin = 5 for RS-485 no modify, TTL must remove R7 or set DR_Pin = 4
 *  
 *  
 *  
*/
//========================== Include Library ======================================//
//Wifi
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <DNSServer.h>
//EEPROM
#include <EEPROM.h>
////OTA upload
//#include <WiFiUdp.h>
//#include <ArduinoOTA.h>
//#include <ESP8266mDNS.h>
//Serial and MQTT
#include <MySerialManager.h>
#include <PubSubClient.h>

//=================================== EEPROM ===================================//
//Size
const int EEPROM_SIZE = 251;
//Start Address on EEPROM, length each data look at: unsigned short getEEPLength(unsigned short address)
#define SSID_ADDR               0
#define PASS_ADDR               40
#define PC_ADDR                 80
#define WIFI_MODE_ADDR          120
#define TRY_ADDR                122
#define UPDATED_ADDR            123
#define BR_ADDR                 124   //Index of BAUD_L[] + 1
#define DF_ADDR                 125   //Index of D_FORMAT_L[] + 1
#define BROKER_HOST_ADDR        126
#define BROKER_PORT_ADDR        166
#define BROKER_USERNAME_ADDR    171
#define BROKER_PASSWORD_ADDR    211

//======================= Connection =====================//
//WiFi accesss point
#define PORT 502  //Port for TCP connection
String deviceID = "MB_MQTT_";
const char *AP_PASS = "";
//Mode
const String ST_MODE = "ST";
const String AP_MODE = "AP";
bool runStation = false;
String stationName = "", stErr = "";
IPAddress ip;
unsigned long lastInActivity = 0;
//Serial
MySerialManager MySerial(Serial);

//====================== MQTT ==========================================//
WiFiClient client;
PubSubClient mqtt(client);
/*
#define MQTT_SERVER   "apyeng.ddns.net"
#define MQTT_PORT     1883                    //Port
#define MQTT_USERNAME "frecon"                //User name, get from Users and ACL
#define MQTT_PASSWORD "frecon"                //User password  
*/
const int BROKER_SIZE = 4; //0 = Host name, 1 = Port, 2 = Username, 3 = Password
int brokerAddress[BROKER_SIZE] = { BROKER_HOST_ADDR, BROKER_PORT_ADDR, BROKER_USERNAME_ADDR, BROKER_PASSWORD_ADDR };
String brokerData[BROKER_SIZE] = { "broker.hivemq.com", "1883", "", "" };
//Modbus poll topic
String modbusPollRequest = "modbus_poll/request/"; //Request by Modbus Poll
String modbusPollResponse = "modbus_poll/response/"; //Response to Modbus Poll in slave format

//===================== Modbus ====================//
//Default
#define BR_INDEX_DEFAULT 5
#define DF_INDEX_DEFAULT 1 
uint8_t BR_INDEX = BR_INDEX_DEFAULT; //Index of BAUD_L[]
uint8_t DF_INDEX = DF_INDEX_DEFAULT; //Index of D_FORMAT_L[]
//Port
#define LED_COM    2 
#define DR_Pin     4     //Direction pin for connection with RE, DE pin of SN75176, GPIO5 = WiFi board, D8(GPIO15) = Evaluation board, 4 = Unuser
#define SetLedCom(x)  digitalWrite(LED_COM, (x)>0? 0:1);     //x=1 => ON, x=0 => OFF
#define SetDR(x) digitalWrite(DR_Pin, (x)>0? 1:0);     //x=1 => Driving mode, x=0 => Receiving mode
//Avaliable 
#define BAUD_L_LEN      14
#define D_FORMAT_L_LEN  12
const unsigned long BAUD_L[BAUD_L_LEN] = { 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 56000, 57600, 115200, 128000, 256000 };
const SerialConfig D_FORMAT_L[D_FORMAT_L_LEN] = { SERIAL_7N1, SERIAL_8N1, SERIAL_7N2, SERIAL_8N2, SERIAL_7O1, SERIAL_8O1, SERIAL_7O2, SERIAL_8O2, SERIAL_7E1, SERIAL_8E1, SERIAL_7E2, SERIAL_8E2 };
const String D_FORMAT_N_L[D_FORMAT_L_LEN] = {"7-N-1", "8-N-1", "7-N-2", "8-N-2", "7-O-1", "8-O-1", "7-O-2", "8-O-2", "7-E-1", "8-E-1", "7-E-2", "8-E-2" };
/**
 * Modbus function codes
 */
enum ModbusFunction {
  FC_INVALID = 0,
  FC_READ_COILS = 1,
  FC_READ_DISCRETE_INPUT = 2,
  FC_READ_HOLDING_REGISTERS = 3,
  FC_READ_INPUT_REGISTERS = 4,
  FC_WRITE_COIL = 5,
  FC_WRITE_REGISTER = 6,  
  FC_READ_EXCEPTION_STATUS = 7,
  FC_WRITE_MULTIPLE_COILS = 15,
  FC_WRITE_MULTIPLE_REGISTERS = 16
};

#define readUInt16(arr, index) word(arr[index], arr[index + 1])


//TCP
//Format: {<Transaction Identifier (2 bytes)>, 0, 0, <Length (2 bytes)>, <RTU message without CRC>}
//Thank: https://m.ipc2u.com/articles/knowledge-base/detailed-description-of-the-modbus-tcp-protocol-with-command-examples/
//Header
#define HEAD_LEN 6
static const uint16_t TRANS_ID_INDEX = 0;
static const uint16_t BYTE_COUNT_INDEX = 4;
uint8_t head[HEAD_LEN] = { 0, 0, 0, 0, 0, 0 };  

//======================= Reset ===========================//
#define CLEAR_PIN   0      //Press holding > CLEAR_TIME => set to WiFi AP Mode
bool action, isSwitchMode;
int lastState;
unsigned long pressTime, startPressTime, startDelaySwitchMode;
const unsigned long CLEAR_TIME = 5000, RESTART_TIME = 250; //ms: CLEAR_TIME = Time for reset board (clear all), RESTART_TIME = Time for restart board

//============================== Web Setting ==============================//
//DNS server
const byte DNS_PORT = 53;
DNSServer dns;
//Server
ESP8266WebServer web(80);
//Page
const char *webHeader = "<!DOCTYPE html>\
<html>\
<head>\
<title>Modbus MQTT to RTU Converter</title>\
<meta http-equiv=\"X-UA-Compatible\" content=\"IE=edge\">\
<meta charset=\"UTF-8\">\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
<link rel = \"icon\" href=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACQAAAAkCAYAAADhAJiYAAAABGdBTUEAALGPC/xhBQAAACBjSFJNAAB6JgAAgIQAAPoAAACA6AAAdTAAAOpgAAA6mAAAF3CculE8AAAABmJLR0QA/wD/AP+gvaeTAAAACXBIWXMAAA3XAAAN1wFCKJt4AAAAB3RJTUUH5AIaCTUHN+F7NQAACURJREFUWMO1l1lsXdUVhr+1zzl3dmzfDHZsQ5xZShoCbRISglNUBgFlrCgPHST6BEiVaGnpUytVqtRKqKqKSiX60iKVNlBoGQoUUgihtEpC2kCcwbEzeCDxeK+vfe07nWHvPpzrKTiO1YolrYeru/fZ/1nr//+1jzArMs8+hdEasW0Eg0yMUzp1DL9cAiRcJIARFhWXLHNicVbe0EbFiXHgRCdP/G4v7b0XEJlZaAMM//pnqHgCEIKBASLrN7TqSrlNx+KbIlu3pyJVGIs6WRZcZAYmS6WcN3luzPUOHu+72PHcdx72Zi+yAcSJgQgqnkjajSsfdrvPPuoNXlgb5HNiPHdBKIuulAFEEMehZkk9O5Y29r/34yeef+rtA08bY7qnqiRDTz+JikVR8UQyGB15snL+9CNuzzllKqX/E8XCoaJxrFVr6U2m3/jRa/t+8P7xU6cAFJbC/fgIQXbk4XLHiUcqnaeVKbvV4n12qcsuXtdprs4Nf/nbe3Y9DjR87qpmlPiayDU7Wt2+vkfd3l6FUeEmY322iQ1aEfT1sUmZe7/RdsPuE59cdFSQG8PPZNq8/oG1+Ka6QS0+tYJAwtQS/l7sXiyMp4lns8t2NTXuAWpt99BBrE1bNumJooTIFyFpY6r24KCSCVQiAUphKmX0xATadUMpK7WoZzFZpDFZuwaos0fffI9lretT+AYx1pX3a4NVnyaxbTvx7dcTaWlB1SxBLAtdKuH29VI6+m+Khw/jDw+FwBZ6RwPiGWzXTyISs30AT0tYamthiRtDYsd20g99i+i69fNWIHL1KlI37sHt6WHspT9R2L8f43kgcvlnGwWBEUCUAASYmd7OR8IpvggqniLSuvqK7Yi0trL8se9S982HkEicmRee59laYQLDNCATMAvMfOSzkSX1YEUofnCI/N/2TR/sjueZON9N/uw5KrncHFDiONQ9+FUS99yHvqxYQmAmQACxVVgh0Nb8M0pr7I3rid9/F/lXX8ec7GT8hb8gK1YwcPIUQwf+QWl4BLQmumwpK3btpPWB+0k0rwxBWRb1Dz5A5qN2nI4ubMuet2VGh4SyhSgmkCraT3NGamqJ3X4rTusqorffxmjfAKn+Efp/8nN6Mxkqvjc9HCujefJd3WT/c4ytP3yCJRvWhe2rr6P2nrvoOfkLGkSw5JIXNxYm0AIoJSQgIFTYPOYVbduNs34txg9IrluDfVMbOd8QKbg0RZNE7ShGLBALUTaibHLtHZz85TN4+fz0mcuv34bf1ER/sUyg1dzztMJoBANKEUeqHJLZGQjO+rXE9uwKFRI2gOU37cbduI7RikdKRWmOpIiKg6lywuiw2plDRxn64PA0oGi6ntoNG8gVy1wsFPGrRQjPs0LagNjKjs9qmcxqVZLYHTcjS2pAayCUrZNM0XzvHXT19iOTZdKRGC0RxcVKkXLgk7q6haXbtiJKYcXjsxguNN12E3YqhQkCcgePUp8rYKspsoduZUskMYvUZqoQRPfsxF6/GgI9t92BpmZNKyvvuJneva8BHmknRnPE4kJxgljjSjY//ih2Ms6l0dC2k4a2nVRy4xw+cYb+/gxN8RQRsUKlgyhx4hhfZmQYCFZLM5E9OxBlgagQ4awUERr27CK1YR2DpTKjrkdCRWiJ11I42kHfK/tYKHqe/yvjHefJB5qBchlfMyN7cWJhhcxUhQx6vMTQy+9QRFO/ZSPpz28O/xMhe6SdXHsnGEMlM0aAYrBSAaNIOzGalOLi718lfe0m6jav/xSY7JHj9LzwZjiIRTHuB6hyBVd01Yfs6DQRp2RvRguo99sZK+bxC2XS124Kb3uiCMoug+8eCoerUohYaEMVlEXaiUG2TO9vXiT508dwUskZEx3L0/nM81Qy44g1M6bGPY9hXQZAWU4cApnFeIVgEbOiXJWoR32SxctPhl5jDOlrN1G/eWO4npk92ghDlTI51yVpxYh8eIbhP++f26q9bzJ65CRKOXMUjVG4oQ+Jsu1YqLJ5Zk1URUlnKngne0P+aEOkJsXqr91NzbpWTFAdO1oQLQSBYbBSJu/YxLWF+8J7FNvPVlt1kp4/vhXem+YZIVOktr6/7Dr8ZOxOCdhxKXlBUAFIvoyzdQ0Si4IxRNNLqNu8DmU7BCUPo8GKxYg3NdB4y05qb9lB4cwFnJFJTCYPqxvoeOqPTHT1hUK5VCQohvzShXdHu/fZFg5GqxnJzxPBuSEqbx8l/pXdYCmMNsQalrL663fScvcXccfDlkbqarBrEogI5ZEcY3sPIB92ke38FWMDFxFlz3+AEfSUD9lmymXD4TZvaKi8047E40Rvuw4VtTHaAIJTW0OkrmbKT8PbJLD8pm2cPdVL9sh50h40qhoGgwKB1vNc2AQTmrLYNg7GiEyr7HK3u4qh/MqHlPqzRG+/jsRVKxArrOy0n4qAErQXUOwbxiu6ZLwSIjZpOwlYDLoTBNrMnFP9XqsCwk7YCQLtG2cK0EI3Rs9gPjjL0PEe9JYW6q5ZQ7x5OVYsggk0fqFMaSBD7qMz5I6dxRubBFEMuaXQp+wE4igG3MmwUnMqVDXGyNIGCpW+StREUIv4QrUE6sY0Q++eomv/x1ipOCrigNYE5QpByUX7PiKhRwFoA0NuEcEibSURx2LAmyAwehpQUQcuYOzuIMeFykR3yl5KbBGXfACFRUOkDvEmyeQKGAzCzGVexJ5pxwwNGXSL4CjSVhJsNQ3KE8OFSn4AKFgtsXrOlLKmJVJ33xLiyfmkP18KFgkVxQBlHSxqjzFQ0D6WWNRbCRyxKWqf/mAy/3L2+CsZr3DIOl4Y4K3RjtGtqZY1jXbtF2I4gFpEhv6RUFGMEUraX9S+uaCSlCTg9bGOf/091/ki0GlNBhW+1/wl/6Xsse5lTs2WZXZqVUwiKBSyiFQokiqKMVRBXamyCmOESe0yYVwOFfvafzt86LmS9v4pSNYCODjRzZA3MdJVGj6NqKWO2E22WFGMoAHfmAUzMBARB98YJrVLAJdNz2gmtUevOzbxRr7j4LOZw38Y9YvvKOQTEw6xSxwK1uxIrrp1a6zlxpVO7aqY2LGQsVewBBG00YwGRVztX3ZZxfiVQT8/2FEePH62MnIEOKGQfo3xpgDMiSanln5vPAY0ACuAOJe3y/8lNDAJZIFRQQpm1tz6L0OayUPonOd/AAAAJXRFWHRkYXRlOmNyZWF0ZQAyMDIwLTAyLTI2VDA5OjUzOjA3KzAwOjAwJ9e6BwAAACV0RVh0ZGF0ZTptb2RpZnkAMjAyMC0wMi0yNlQwOTo1MzowNyswMDowMFaKArsAAAAZdEVYdFNvZnR3YXJlAHd3dy5pbmtzY2FwZS5vcmeb7jwaAAAAAElFTkSuQmCC\" type = \"image/x-icon\">\
<style>\
body {\
font-family: Arial;\
max-width: 450px;\
margin: auto;\
padding-bottom: 25px;\
}\
h1, h3, h4{\
  color: #006699;\
}\
h3 {\
    margin: 15px 0 10px;\
    font-size: 1.5em;\
    line-height: 1.3;\
    font-weight: 700;\
    font-style: normal;\
    text-decoration: none;\
    border-bottom: 2px solid #ccc;\
}\
.tab {\
  overflow: hidden;\
  border: 1px solid #ccc;\
  background-color: #f1f1f1;\
}\
.tab button {\
  background-color: inherit;\
  float: left;\
  border: none;\
  outline: none;\
  cursor: pointer;\
  padding: 14px 16px;\
  transition: 0.3s;\
  font-size: 17px;\
}\
.tab button:hover {\
  background-color: #ddd;\
}\
.tab button.active {\
  background-color: #ccc;\
}\
.tabcontent {\
  padding: 6px 12px;\
  border: 1px solid #ccc;\
  border-top: none;\
}\
.btn {\
    border: none;\
    padding: 8px 15px 8px 15px;\
    color: #fff;\
    box-shadow: 1px 1px 4px #DADADA;\
    -moz-box-shadow: 1px 1px 4px #DADADA;\
    -webkit-box-shadow: 1px 1px 4px #DADADA;\
    border-radius: 3px;\
    -webkit-border-radius: 3px;\
    -moz-border-radius: 3px;\
    cursor: pointer;\
    font-size: 16px;\
}\
.dark_btn {background-color: #ccc; color: black;} \
.dark_btn:hover {background: #ddd;}\
.bright_btn {background-color: #006699;}\
.bright_btn:hover {background-color: #003c6b;}\
* {\
  box-sizing: border-box;\
}\
label{\
    display: block;\
    margin: 0px 0px 15px 0px;\
    font-size: 16px;\
}\
label > span{\
      width: 100px;\
      font-weight: bold;\
      float: left;\
      padding-top: 8px;\
      padding-right: 5px;\
      font-size: 16px;\
}\
input[type=text], select, input[type=number]{\
    box-sizing: border-box;\
    -webkit-box-sizing: border-box;\
    -moz-box-sizing: border-box;\
    border: 1px solid #C2C2C2;\
    box-shadow: 1px 1px 4px #EBEBEB;\
    -moz-box-shadow: 1px 1px 4px #EBEBEB;\
    -webkit-box-shadow: 1px 1px 4px #EBEBEB;\
    border-radius: 3px;\
    -webkit-border-radius: 3px;\
    -moz-border-radius: 3px;\
    padding: 7px;\
    outline: none;\
    font-size: 16px;\
    width: 40%;\
}\
input[type=text]:focus, select:focus, input[type=number]:focus {\
  border: 1px solid #ff9100;\
}\
table {\
  font-family: arial, sans-serif;\
  border-collapse: collapse;\
  width: 100%;\
}\
th {\
  border: 1px solid #dddddd;  \
  padding: 8px;\
}\
td {\
  border: 1px solid #dddddd;\
  text-align: left;\
  padding: 8px;\
}\
tr:nth-child(even) {\
  background-color: #ffffff;\
}\
.scan_btn{\
  background: #757575;\
  color: white;\
  margin-left: 15px;\
}\
.scan_btn:hover{\
  background: #1b1b1b;\
}\
.modal {\
  display: none;\
  position: fixed;\
  z-index: 1;\
  left: 0;\
  top: 0;\
  width: 100%; \
  height: 100%;\
  overflow: auto;\
  background-color: rgba(0,0,0,0.4);\
  box-shadow: 3px 3px 20px rgba(0,0,0,0.3);\
}\
.modal-content {\
    border-radius: 8px;\
    background-color: #fefefe;\
    margin: 4% auto 15% auto;\
    border: 1px solid #888;\
 width: 80%;\
    max-width: 430px;\
    padding-bottom: 30px;\
    position: relative;\
}\
.close {\
    position: absolute;\
    background: black;\
    color: white;\
    top: -10px;\
    right: -10px;\
    width: 32px;\
    height: 32px;\
    font: 16px Arial;\
    border-radius: 50%;\
    box-shadow: 3px 3px 20px rgba(0,0,0,0.3);\
}\
.close:hover,.close:focus {\
    background: red;\
}\
.animate {\
    animation: zoom 0.6s\
}\
@keyframes zoom {\
    from {transform: scale(0)} \
    to {transform: scale(1)}\
}\
#networkTable{\
  padding: 20px 20px 20px 20px;\
}\
table #networkTable {\
    border-collapse: collapse;   \
}\
#networkTable th {\
    background-color: #fff;\
}\
#networkTable tr:hover {\
    background-color: #ccc;\
}\
#networkTable td:hover {\
    cursor: pointer;\
}\
</style>\
</head>\
<body>\
<h1>Modbus MQTT to RTU Converter</h1>\
<div id=\"networkList\" class=\"modal\">\
  <div class=\"modal-content animate\">\
    <button onclick=\"showNetwork(\'none\')\" class=\"close\" title=\"ปิด\">x</button>\
  <div id=\"networkTable\"></div>\
  </div>\
</div>";
    
const char *webFooter = "<script>\
function setNetworkTable(v){\
  document.getElementById(\'networkTable\').innerHTML = v;\
}\
function showNetwork(v){\
  document.getElementById(\'networkList\').style.display=v;\
  if(v!=\'block\'){ return; }\
  setNetworkTable(\"<center><h4>Wi-Fi network scanning...</h4></center>\");\
  var message = \"/scan\";\
    var request = new XMLHttpRequest();\
    request.onreadystatechange = function() {\
      if (this.readyState == 4 && this.status == 200) {\
         setNetworkTable(this.responseText);\
      }\
    };\
    request.open(\"GET\", message, true);\
    request.send(null);\
}\
function setNetwork(v){\
  var ssid = document.getElementById(\"ssidIn\");\
  ssid.value = v;\
  showNetwork(\'none\');\
}\
</script>\
</body>\
</html>";

const String bodySavedContent = "<center><h3>บันทึกค่าสำเร็จ</h3><p>ถ้าโมดูลเชื่อมต่อ Wi-Fi ไม่สำเร็จ ให้เชื่อมต่อ Wi-Fi จากโมดูลและ<a href=\"/\">ลองใหม่อีกครั้ง</a></p></center>";
const String bodyFailedContent = "<center><h3>บันทึกค่าไม่สำเร็จ</h3><p><a href=\"/\">ลองใหม่อีกครั้ง</a></p></center>";

//============== Other ==================//
#define DB Serial
const String softVersion = "1.0";

//WiFi Event
WiFiEventHandler wifiConnectedHandler;
WiFiEventHandler wifiDisconnectedHandler;
unsigned char countWifiLost = 0;
bool blink_toggle = 0;

//Update IP address on PC
unsigned long startRetryTime = 0, startTime; bool ipUpdated = false;
unsigned short tryTotal = 0;

int getEEPLength(int address) {
  switch (address) {
    case SSID_ADDR:
    case PASS_ADDR:
    case PC_ADDR:     
    case BROKER_HOST_ADDR:
    case BROKER_USERNAME_ADDR:
    case BROKER_PASSWORD_ADDR: return 40;
    case BROKER_PORT_ADDR: return 5;
    case WIFI_MODE_ADDR: return 2;
    case TRY_ADDR:    
    case UPDATED_ADDR:  
    case BR_ADDR:
    case DF_ADDR: return 1;
    default: return EEPROM_SIZE - address; //Max length
  }
}

void createWiFiAccessPoint() {
  //DB.print("Create WiFi Access Point");
  //DB.println();
  //Clear previously config
  WiFi.persistent(false);
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);

  const char* AP_SSID = deviceID.c_str(); //convert String to Char*

  //Create WiFi Access Point without set IP address, default IP: 192.168.4.1
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    //DB.println("failed to create AP, we should reset as see if it connects");
    softwareReset();
  }

  //LAN config
  ip = WiFi.softAPIP();
  //Save state
  runStation = false;
}

bool runSTMode() {
  //Off WiFi, แก้ปัญหากรณีที่ IP คืนค่าเป็น 0.0.0.0 จึงทำให้ WiFi Module restart
  //Thank: https://github.com/tzapu/WiFiManager/issues/31
  WiFi.mode(WIFI_OFF);
  delay(200);
  //Read mode
  String modeRun = readString(WIFI_MODE_ADDR);
  //String modeRun = "ST";
  //DB.print("Wifi start with mode "); DB.println(modeRun);
  if (modeRun != ST_MODE) {
    return false;
  }

  //========== ST mode ==============//
  //Set SSID and PASS for connection
  stationName = readString(SSID_ADDR); delay(20);
  String b2 = readString(PASS_ADDR); delay(20);
  //stationName = "HeyYou";
  //String b2 = "12345678";
  const char* STA_SSID = stationName.c_str();
  const char* STA_PASS = b2.c_str();

  //DB.print("Connecting to "); DB.println(STA_SSID);
  //DB.print("Pass: "); DB.println(STA_PASS);

  //Remove previously WiFi config
  WiFi.persistent(false);
  WiFi.disconnect();
  // Register event handlers.
  wifiConnectedHandler = WiFi.onStationModeConnected(&onWiFiConnected);
  wifiDisconnectedHandler = WiFi.onStationModeDisconnected(&onWiFiDisconnected);
  //Select WiFi station mode
  WiFi.mode(WIFI_STA); //If not call this line, Module may run 2 mode => Station mode and Access point mode
  WiFi.begin(STA_SSID, STA_PASS);
  WiFi.setAutoReconnect(true);  //Reconnect to an access point in case it is disconnected 
  //Improve from E:\APY Start_21-Mar-2018\Arduino Example\My devolpment\ESP8266_Improve_ST_mode
  int result = -1; unsigned short tO = 0;
  while (result != WL_CONNECTED) {
    result = WiFi.status();
    switch (result) {
      case WL_NO_SSID_AVAIL:
        stErr = "WiFi SSID is unavailable.";
        //DB.println(stErr);
        return false;
      case WL_CONNECT_FAILED:
        stErr = "WiFi Password is incorrect.";
        //DB.println(stErr);
        stationName = "";
        return false;
    }
    //Check timeout
    if (tO > 200) { //40s
      stErr = "WiFi connecting timeout";
      //DB.println(stErr);
      //Get last try
      uint8_t countTry = EEPROM.read(TRY_ADDR);
      //Check limit trying
      if (countTry < 1) {
        //Save try
        saveTryConnectWiFi(countTry + 1);
        //Restart to try again
        softwareReset();
      } else {
        //Clear
        saveTryConnectWiFi(0);
      }
      wifi_station_disconnect();
      return false;
    }
    //Little delay for getting all result
    tO++;
    delay(200);
  }
  stErr = "Connected!";
  //DB.println(stErr);
  //Clear
  saveTryConnectWiFi(0);
  //LAN config
  ip = WiFi.localIP();
  return true;
}

void saveTryConnectWiFi(uint8_t count) {
  writeByte(TRY_ADDR, count);
}

//Ref: https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html
bool networkIsFound(String stationSSID) {
  //First checking
  if (stationSSID == "") {
    return false;
  }
  //Get size
  int n = WiFi.scanNetworks();
  //Find
  for (int i = 0; i < n; i++) {
    if (stationSSID == WiFi.SSID(i)) {
      return true;
    }
  }
  return false;
}

bool writeString(int addr, String text) {
  //Length out of range?
  unsigned int len = text.length();
  if (len > getEEPLength(addr)) {
    return false;
  }
  //Write
  //String t = "["+String(addr)+"] "+text+" ["+String(len)+"]";
  //DB.print("Write: "); //DB.println(t);
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + i, text[i]); //Write one by one with starting address of addr
  }
  return EEPROM.commit();    //Store data to EEPROM
}

//Updated by Somsak Elect 08/10/2019
String readString(int addr) {
  //DB.print("Read address: "); ////DB.print(addr);
  String text = ""; int mL = getEEPLength(addr);
  for (int i = 0; i < mL; i++) {
    byte v = EEPROM.read(addr + i);
    if (v == 0) {
      break;  //Out of loop!
    }
    text += char(v);
  }
  //DB.print(" Value: "); ////DB.println(text);
  return text;
}

bool writeByte(uint16_t addr, uint8_t v){
  if(addr>=EEPROM_SIZE) return false;
  EEPROM.write(addr, v);
  return EEPROM.commit();
}

uint8_t readByte(uint16_t addr){
  if(addr>=EEPROM_SIZE) return 0xFF;
  return EEPROM.read(addr);
}

void clearEEPROM(int start, int f) {
  //DB.println("Clear EEPROM");
  for (int i = start; i < f; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit(); //Must alway call this function when you write to EEPROM finish. if you don't use, writing EEPROM will unsuccess.
}

//After reset, this method good work without hanging! with changing WiFi mode
void softwareReset() {
  //DB.println("Software reset!");
  delay(50);
  ESP.reset();
}

//Avoid WDT reset!: use for delay >= 1000
void delayMillis(int d) {
  int n = d / 10;
  for (int i = 0; i < n; i++) {
    delay(10);
  }
}

void checkClearMode(){
  int currentState = digitalRead(CLEAR_PIN);

  //Check state 
  if(currentState==LOW && lastState==HIGH){
      //Save time
      startPressTime = millis(); 
      action = false;
      //DB.print("Start press time: "); DB.println(startPressTime);
    
  }else if(currentState==LOW && lastState==LOW){
      //DB.print("Press time: "); DB.println(millis()-startPressTime);
      //Do action only one time/press
      if((millis()-startPressTime)>CLEAR_TIME && !action){
        //Serial.print("\nClear WiFi Mode... and wait to restart module!");
        clearEEPROM(0, EEPROM_SIZE);  //Clear all
        //Set flag
        SetLedCom(0);  //OFF LED
        action = true;
      }
//      if((millis()-startPressTime)>RESTART_TIME){
//        action = true;
//      }
      delay(1); //Delay 1ms for recheck again
      
  }else if(currentState==HIGH && lastState==LOW){
    //Restart
    if(action){
      //Serial.println("\n\tRestart ESP...");
      //Restart
      ESP.reset();
      //Clear value
      pressTime = 0; 
      action = false;
    }
    
  }

  //Save last state
  lastState = currentState;
  
}

String sendHTML(String content){
  return webHeader + content + webFooter;
}

void handleForm() {
  //DB.println("Setting from Web...");

  String eSSID = web.arg("ssid");
  String ePass = web.arg("password");
  String saveBy = web.arg("saveBy");  //IP Address of PC
  String br = web.arg("baud");  //Index of BAUD_L[] 
  String df = web.arg("format"); //Index of D_FORMAT_L[]
  String mqttHost = web.arg("mqtt-host");
  String mqttPort = web.arg("mqtt-port");
  String mqttName = web.arg("mqtt-username");
  String mqttPass = web.arg("mqtt-password");

  if(br==""){ br = String(BR_INDEX_DEFAULT+1); } 
  if(df==""){ df = String(DF_INDEX_DEFAULT+1); } 

//  DB.print("SaveBy: "); DB.println(saveBy);
//  DB.print("Baud: "); DB.println(br);
//  DB.print("Format: "); DB.println(df);
//  DB.print("SSID: "); DB.println(eSSID);
//  DB.print("Password: "); DB.println(ePass);

  //Buffer
  String str[] = { eSSID, ePass, saveBy, mqttHost, mqttPort, mqttName, mqttPass, br, df };
  int addr[] = { SSID_ADDR, PASS_ADDR, PC_ADDR, BROKER_HOST_ADDR, BROKER_PORT_ADDR, BROKER_USERNAME_ADDR, BROKER_PASSWORD_ADDR, BR_ADDR, DF_ADDR };
  bool ok = true;
  //Clear and slave
  for (int i = 0; i < 9; i++) {
    //DB.print("Try save: "); DB.println(addr[i]);
    clearEEPROM(addr[i], addr[i]+getEEPLength(addr[i])); //Clear
    if(i>6){
      //DB.print(str[i]); DB.print(" >< "); DB.println((uint8_t)str[i].toInt());
      ok = writeByte(addr[i], (uint8_t)str[i].toInt()); //Save
    }else{
      ok = writeString(addr[i], str[i]);  //Save
    }
    delay(20);
    if (!ok) {
      //DB.println("Failed!");
      goto FORM_OUT;
    }
  }

FORM_OUT:
  web.send(200, "text/html", ok ? sendHTML(bodySavedContent) : sendHTML(bodyFailedContent)); //Send web page
  //Set flag to restart
  if (ok) {
    //DB.println("Saved");
    //Save mode
    if (!runStation) {
      writeString(WIFI_MODE_ADDR, ST_MODE);  //AP to ST
    }
    //Set flag to restart
    startDelaySwitchMode = millis();
    isSwitchMode = true;
  }
}

void handleOnConnect(){
  //Set content
  String con = "";
    //About
    con += "<div id=\"AboutDevice\"><h3>About</h3>";
    con += "<p>Device ID: " + deviceID + "</p>";
    con += "<p>IP Address: " + getStingOfIP(ip) + "</p>";
    con += "<p>MAC Address: " + String(WiFi.macAddress()) + "</p>";
    con += "<p>version: " + softVersion + "</p></div>";
    //Setting
    con += "<form id=\"myForm\" action=\"/save\">";
    //Serial
    con += getSerialHTML();
    //MQTT broker
    con += "<div id=\"MqttData\"><h3>Connect to MQTT broker</h3>";
    con += "<label for=\"mqtt-host\"><span>Host:</span><input type=\"text\" maxlength=\"40\" name=\"mqtt-host\" value=\""+brokerData[0]+"\" required></label>";
    con += "<label for=\"mqtt-port\"><span>Port:</span><input type=\"number\" min=\"1\" max=\"65535\" name=\"mqtt-port\" value=\""+brokerData[1]+"\" required></label>";
    con += "<label for=\"mqtt-username\"><span>Username:</span><input type=\"text\" maxlength=\"40\" name=\"mqtt-username\" value=\""+brokerData[2]+"\"></label>";
    con += "<label for=\"mqtt-password\"><span>Password:</span><input type=\"text\" maxlength=\"40\" name=\"mqtt-password\" value=\""+brokerData[3]+"\"></label></div>";
    //Connect to Wi-Fi
    con += "<div id=\"RounterData\"><h3>Connect to Wi-Fi</h3>";
    con += "<label for=\"ssid\"><span>SSID:</span><input id=\"ssidIn\" type=\"text\" maxlength=\"40\" name=\"ssid\" value=\""+readString(SSID_ADDR)+"\" required><button type=\"button\" class=\"btn scan_btn\" onclick=\"showNetwork(\'block\')\">Scan</button></label>";
    con += "<label for=\"password\"><span>Password:</span><input type=\"text\" name=\"password\" value=\"" + readString(PASS_ADDR) + "\"></label></div>";
    con += "<label><span></span><input type=\"submit\" class=\"btn bright_btn\" value=\"Save\"/></label></form>";    

  //Response
  SetLedCom(0); //Off LED
  web.send(200, "text/html", sendHTML(con));
  SetLedCom(1); //On LED
}

String getSerialHTML(){
  String h = "<div id=\"SerialData\">\
                <h3>Serial</h3>\
                <select id=\"baud\" name=\"baud\">";
  //Baudrate              
  for(uint8_t i=0; i<BAUD_L_LEN; i++){
    h += "<option value=\""+String(i+1)+"\" ";
    if(i == BR_INDEX){ h += "selected"; }
    h += ">"+String(BAUD_L[i])+" Baud</option>";
  }
  h += "</select><select id=\"format\" name=\"format\">";
  //Data format
  for(uint8_t i=0; i<D_FORMAT_L_LEN; i++){
    h += "<option value=\""+String(i+1)+"\" ";
    if(i == DF_INDEX){ h += "selected"; }
    h += ">"+D_FORMAT_N_L[i]+"</option>";
  }
  h += "</select></div>";
  return h;                     
}

String getNetworkTable(){
  //Get size
  int n = WiFi.scanNetworks();  
  if(n==0){ return "<center><h4>Not found Wi-Fi network!</h4></center>"; }
  //Set Table
  String d = "<center><h4>Wi-Fi Network Discovery</h4></center><table><tr><th>SSID</th><th>RSSI</th></tr>";
  String ssid;
  for(int i=0; i<n; i++){
    ssid = WiFi.SSID(i);
    d += "<tr onclick=\"setNetwork(\'"+ssid+"\')\"><td>"+ssid+"</td><td>"+String(WiFi.RSSI(i))+"dBm</td></tr>";
  }
  return d + "</table>";
}

void handleScan(){
  web.send(200, "text/html", getNetworkTable());
}

//Thank: https://forum.arduino.cc/index.php?topic=228884.0
String getStingOfIP(const IPAddress& address){
  return String() + address[0] + "." + address[1] + "." + address[2] + "." + address[3];
}

String getJsonElement(String key, String value){
  //"key":"value"
  return "\""+key+"\":\""+value+"\"";
}

String getJsonOfDevice(String ssid, const IPAddress& ip){
  String key[] = { "ID", "Connected", "IP" }; //Key format should match with VB
  String value[] = { deviceID, ssid, getStingOfIP(ip) };
  String d = "{";
  for(int i=0; i<3; i++){
    if(i>0){ d += ","; }
    d += getJsonElement(key[i], value[i]);  
  }
  return d+"}";
}

void updateDeviceData(){
  //ST mode only
  if(!runStation || ipUpdated || tryTotal>3){ return; }
  //Check flag
  uint8_t updated = EEPROM.read(UPDATED_ADDR);
  //Check IP address
  String pcIP = readString(PC_ADDR);
  //Check allow
  if(pcIP=="" || updated){ ipUpdated = true; return; }
  IPAddress host;
  if (!host.fromString(pcIP)) { return; }
  String clientData = getJsonOfDevice(stationName, ip);
  //Update Client Data
  if((millis()-startRetryTime)>3000){
    //DB.print("Try to update device data to "); DB.println(host);
    WiFiClient mClient;
    if (mClient.connect(host, PORT)){
      //Send device data with Json string format
      mClient.print(clientData);
      mClient.flush();
      //Waiting response
      while (mClient.connected() || mClient.available()){
        if (mClient.available()){
          uint8_t nD = mClient.read();
          //DB.print("Result: "); DB.println(nD); // 1 = Success, 0 = Failed
          ipUpdated = nD>0;
          //DB.print("IP updated: "); DB.println(ipUpdated);
          //Save EEP
          writeByte(UPDATED_ADDR, nD);
          break;
        }
      }
      //Disconnect
      mClient.stop();
    }
    //Save
    startRetryTime = millis();
    tryTotal++;
  }
}

uint16_t CRC16 (const uint8_t *nData, uint16_t offset, uint16_t wLength){
      static const uint16_t wCRCTable[] = { 
            0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
            0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
            0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
            0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
            0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
            0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
            0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
            0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
            0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
            0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
            0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
            0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
            0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
            0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
            0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
            0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
            0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
            0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
            0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
            0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
            0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
            0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
            0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
            0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
            0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
            0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
            0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
            0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
            0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
            0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
            0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
            0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
    };

    byte nTemp;
    uint16_t wCRCWord = 0xFFFF;
    
    while (wLength--){
        nTemp = *(nData+(offset++)) ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }

    return wCRCWord;
    
} 


boolean validateModbusFrame(const uint8_t *buff, uint16_t len, bool rtu)
{   /*
    Serial.println();
    for(int i=0; i<len; i++){
      Serial.print(buff[i], HEX); Serial.print(" ");
    }
    Serial.println();
    */
    //Index of array
    uint8_t id, fn, minLen;
    if(rtu){
      // RTU: minLen (1 x Slave ID, 1 x Function, n x Data, 2 x CRC)
      id = 0; fn = 1; minLen = 4;
    }else{
      // TCP: minLen (2 x Transaction ID, 2 x Protocol ID, 2 x Message Length, 1 x Unit ID, 1 x Function, n x Data)
      id = 6; fn = 7; minLen = 8; 
    }
    //Check length
    if(len<minLen) return false;
    //Check CRC 
    if(rtu){
      uint16_t crc = CRC16(buff, 0, len-2);
      //Serial.println();
      //Serial.print(lowByte(crc), HEX); Serial.print(" "); Serial.println(highByte(crc), HEX);
      if(lowByte(crc)!=buff[len-2] || highByte(crc)!=buff[len-1]) return false;
    }
    //SetLedCom(0);
    //Check Salve/Unit ID 
    if(buff[id]>247) return false;
    //Check function code
    switch (buff[fn] & 0x7F)
    {
    case FC_READ_COILS:             // read coils (digital read)
    case FC_READ_DISCRETE_INPUT:    // read input state (digital read)
    case FC_READ_HOLDING_REGISTERS: // read holding registers (analog read)
    case FC_READ_INPUT_REGISTERS:   // read input registers (analog read)
    case FC_WRITE_COIL:             // write coils (digital write)
    case FC_WRITE_REGISTER:         // write regosters (digital write)
    case FC_WRITE_MULTIPLE_COILS:
    case FC_WRITE_MULTIPLE_REGISTERS:
        return true;
    default:
        return false;
    }
}

//Handle received
void MySerial_callbackReceived(uint8_t *data, uint16_t len){
  //Verify
  if(validateModbusFrame(data, len, true)){
    //Remove CRC
    len -= 2;
    //Set byte count
    head[BYTE_COUNT_INDEX] = highByte(len);
    head[BYTE_COUNT_INDEX+1] = lowByte(len);
    //Insert head[] before first index of *data
    memmove(data+HEAD_LEN, data, len);
    memmove(data, head, HEAD_LEN);
    //ON LED
    SetLedCom(1);
    //Send to Slave
    mqtt.publish(modbusPollResponse.c_str(), data, len+HEAD_LEN);
    //Clear head
    for(int x=0; x<HEAD_LEN; x++){
      head[x] = 0;
    }
  }
}

//Handle send start
void MySerial_callbackSendStart(){
  //Driving Mode
  SetDR(1);
}

//Handle send end
void MySerial_callbackSendEnd(){
  //Receiving Mode
  SetDR(0);
}

void mqtt_Callback(char* topic, byte* payload, unsigned int plength) {
  //DB.print("New request..."); DB.println(length);
  //Verify
  if(validateModbusFrame(payload, plength, false)){
    //Off LED
    SetLedCom(0);
    //Save head
    memmove(head, payload, HEAD_LEN);
    //Set CRC
    uint16_t crc = CRC16(payload, HEAD_LEN, plength-HEAD_LEN);
    payload[plength++] = lowByte(crc);
    payload[plength++] = highByte(crc);
    //Send to Slave without Header
    MySerial.sendBytes(payload, HEAD_LEN, plength-HEAD_LEN);
    //ON
    //SetLedCom(1);
  }
}

void runMQTT(){
  //Allow?
  if(!runStation){ return; }
  //MQTT running
  if(!mqtt.connected()){
    SetLedCom(0);
    //Connect
    //DB.print("MQTT connecting...");
    //ClientID Note: https://www.cloudmqtt.com/blog/2018-11-21-mqtt-what-is-client-id.html
    //DB.println(deviceID.c_str());
    if (mqtt.connect(deviceID.c_str(), brokerData[2].c_str(), brokerData[3].c_str())){
      //DB.println("connected");
      //Subscribe
      mqtt.subscribe(modbusPollRequest.c_str());
      SetLedCom(1);
    }else{
       //DB.println("failed, rc="); DB.print(client.state());
       //Wait before retrying and blink LED 2 sec
//       SetLedCom(0); delay(1000);
//       SetLedCom(1); delay(1000);
      //Toggle LED
      blink_toggle = blink_toggle? 0 : 1;
      SetLedCom(blink_toggle);      
      //delayMillis(1000);
    }
  }else{
    //Handle serial's data incomming
    MySerial.poll();
    //Interval check data incomming
    mqtt.loop();
  }
}

void onWiFiConnected(const WiFiEventStationModeConnected& evt) {
  //DB.print("WiFi connected..."); DB.println(millis());
  //On LED
  SetLedCom(1);
  //Clear
  countWifiLost = 0;
  blink_toggle = 0;
}

void onWiFiDisconnected(const WiFiEventStationModeDisconnected& evt) {
  //Toggle LED
  blink_toggle = blink_toggle? 0 : 1;
  SetLedCom(blink_toggle);
  //Count 3 sec/times
  countWifiLost++;
  //DB.print("WiFi disconnected..."); DB.println(countWifiLost);
  if(countWifiLost>10){
    countWifiLost = 0;
    //Restart board to AP mode
    softwareReset();
  }
}

void setup() {
  //Set pin
  pinMode(CLEAR_PIN, INPUT); //Clear BTN
  pinMode(LED_COM, OUTPUT);  //LED blink for comunication
  pinMode(DR_Pin, OUTPUT);   //RS-485 mode
  SetLedCom(1);              //ON LED     
  SetDR(1);                  //Drive mode

  //Enable serial port by default baudrate
  DB.begin(9600);
  
  //EEPROM
  EEPROM.begin(EEPROM_SIZE);

  //Get WiFi Mode length to Clear EEPROM (Action due to set: Flash All Content)
  String m = readString(WIFI_MODE_ADDR);
  if (m != ST_MODE && m != AP_MODE) { 
    //Clear All
    clearEEPROM(0, EEPROM_SIZE); 
    //Save mode
    writeString(WIFI_MODE_ADDR, AP_MODE);
  }

  //Baudrate for Modbus
  uint8_t sI = readByte(BR_ADDR); delay(5);
  if(sI>0 && sI<=BAUD_L_LEN){ BR_INDEX = sI-1; }
  //Data format for Modbus
  sI = readByte(DF_ADDR); delay(5);
  if(sI>0 && sI<=D_FORMAT_L_LEN){ DF_INDEX = sI-1; }   

  DB.print("MB baud: "); DB.println(BAUD_L[BR_INDEX]);
  DB.print("MB format: "); DB.println(D_FORMAT_N_L[DF_INDEX]);

  //========= Set Device ID =========//
  //Get MAC address, for MAC address of ESP8266 same between AP mode and ST mode
  byte moduleMAC[6]; //6 byte array to hold the MAC address
  WiFi.macAddress(moduleMAC); //ex. mac array in form Hex value = {B4,E6,2D,B2,48,6E} but actual MAC ID = {6E,48,B2,2D,E6,B4} <= reverse
  String macID = "";
  int i;
  for(i=4; i<6; i++){
    String a = String(moduleMAC[i], HEX); //Convert byte to HEX
    if(a.length()<2){ a = "0"+a; } //Keep 2 digit
    macID += a;
  }
  macID.toUpperCase();
  //Update device name
  deviceID += macID;
  //DB.print("\nDevice ID: "); DB.println(deviceID);
  
  //===== Set WiFi =======//
  runStation = runSTMode();
  if (!runStation) {
    //Run by AP mode
    createWiFiAccessPoint();
  }
  DB.print("Local IP: "); DB.println(ip);
  delay(100); DB.end(); //Disable before re-enable by Modbus 

  //=============== MQTT ================//
  //Get MQTT broker
  String nBuf[BROKER_SIZE]; 
  for(i=0; i<BROKER_SIZE; i++){
    nBuf[i] = readString(brokerAddress[i]); 
    delay(10);
  }
  //Check broker
  if(!nBuf[0].isEmpty() && !nBuf[1].isEmpty()){
    for(i=0; i<BROKER_SIZE; i++){
      brokerData[i] = nBuf[i]; 
    }
  }
  //Update topic with Modbus Poll, .../<device ID>
  modbusPollRequest += deviceID;
  modbusPollResponse += deviceID; 
  //Set broker and callback function
  int mPort = brokerData[1].toInt();
  if(mPort<=0){ mPort = 1883; brokerData[1] = "1883"; }
  mqtt.setServer(brokerData[0].c_str(), mPort);
  mqtt.setCallback(mqtt_Callback);

  //======= Serial for Modbus =========//
  MySerial.begin(BAUD_L[BR_INDEX], D_FORMAT_L[DF_INDEX]);
  MySerial.setCallbackReceived(MySerial_callbackReceived);
  MySerial.setCallbackSendStart(MySerial_callbackSendStart);
  MySerial.setCallbackSendEnd(MySerial_callbackSendEnd);
  MySerial.setFrameSize(4); //Min length of RTU frame

  //=========== Web Server ============//
  //Setup the DNS server redirecting all the domains to the apIP
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(DNS_PORT, "*", ip);
  //Server, thank: https://circuits4you.com/2019/03/20/esp8266-receive-post-get-request-data-from-website/
  web.on("/", handleOnConnect);
  web.on("/save", handleForm);
  web.on("/scan", handleScan);
  web.onNotFound(handleOnConnect); //Redirect page
  web.begin(); //Start server
  
  //Save time
  startRetryTime = millis();
  startTime = startRetryTime;
  
}



void loop() {

  //Check clear SW
  checkClearMode();
  if(action){ return; } //Hold off LED

  //Auto Switch Mode
  unsigned long diff = millis() - startTime;
  if(diff>30000){
    //Now running on AP mode, but it should run in ST mode, check network back?
    if(!runStation && networkIsFound(stationName)){
      softwareReset();
    }
    //Save time
    startTime = millis();
  }

  //WiFi Mode changed?
  if (isSwitchMode) {
    //Delay time before reset
    if ((millis() - startDelaySwitchMode) > 3000) {
      isSwitchMode = false;
      ESP.reset();
    }
  }

  //DNS
  dns.processNextRequest(); //Redirect
  //HTTP 
  web.handleClient();

  //Update Client Data
  updateDeviceData();
  
  //Handle MQTT
  runMQTT();
  
}
