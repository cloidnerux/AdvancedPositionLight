/// Advanced Position Light
/// Based upon an arduino pro mini
/// by cloidnerux, 2015


#include <../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h>        // Mavlink interface
#include <../GCS_MAVLink/include/mavlink/v1.0/common/mavlink.h>


#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


// How many WS2812 are attached to the Arduino in one string
#define NUMPIXELS      8

//Which voltage is to considerd good
#define BATTERY_GOOD 4000
//Which voltage is consider accapable
#define BATTERY_OK   3600

//The string of WS2812
Adafruit_NeoPixel strip_1 = Adafruit_NeoPixel(NUMPIXELS, 4, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_2 = Adafruit_NeoPixel(NUMPIXELS, 5, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_3 = Adafruit_NeoPixel(NUMPIXELS, 6, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_4 = Adafruit_NeoPixel(NUMPIXELS, 7, NEO_GRB + NEO_KHZ800);


////////////////////////////////////////////////////////////////////////////////////

uint8_t    ap_fixtype = 3;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;

uint16_t   ap_voltage_battery = 0;    // 1000 = 1V
int16_t    ap_current_battery = 0;    //  10 = 1A
uint8_t    ap_cell_count = 0;

////////////////////////////////////////////////////////////////////////////////////

uint8_t updateSys;
uint8_t updateGps;

////////////////////////////////////////////////////////////////////////////////////
static uint8_t      apm_mav_type;
static uint8_t      apm_mav_system; 
static uint8_t      apm_mav_component;
static uint8_t      base_mode=0;

mavlink_message_t msg; 
uint8_t buf[50];
int system_type = MAV_TYPE_QUADROTOR;
int autopilot_type = MAV_AUTOPILOT_GENERIC;

////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //startup all diferent modules
  strip_1.begin();
  strip_2.begin();
  strip_3.begin();
  strip_4.begin();
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  updateSys = 0;
  updateGps = 0;

  //Initialise the LEDs
  for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip_1.setPixelColor(i, strip_1.Color(0,0,0)); // Moderately bright green color.
    strip_2.setPixelColor(i, strip_2.Color(0,0,0)); // Moderately bright green color.
    strip_3.setPixelColor(i, strip_3.Color(0,0,0)); // Moderately bright green color.
    strip_4.setPixelColor(i, strip_4.Color(0,0,0)); // Moderately bright green color.
  }
  strip_1.show(); // This sends the updated pixel color to the hardware.
  strip_2.show();
  strip_3.show();
  strip_4.show();

  //Wait a moment for the APM/Pixhawk to boot
  delay(3000);
}

void loop() {
  uint8_t red = 0, green = 0, blue = 0;
  uint16_t tmp = 0;
  static unsigned long time = millis();
  static uint8_t state = 0;
  static uint8_t connection = 0;
  uint8_t voltageAlarm;

  //Receive Mavlink packages
  comm_receive();
  //Do we have valid voltage data?
  if(ap_cell_count > 1)
  {
    //Is the battery empty, so we have to give a low voltage alarm indicator
    if((ap_voltage_battery / ap_cell_count) < BATTERY_OK)
      voltageAlarm = 1;
    else
      voltageAlarm = 0;
  }

  //Switch every second
  if((millis() - time) > 1000)  //tick
  {
    time = millis();
    state ^= 1;
    if(state == 0)
    {
      for(int i=0;i<NUMPIXELS;i++){
        strip_3.setPixelColor(i, strip_3.Color(0,0,0)); // Moderately bright green color.
        strip_4.setPixelColor(i, strip_4.Color(0,0,0)); // Moderately bright green color.
        if(voltageAlarm)
          strip_2.setPixelColor(i, 0, 0, 0);
        }
    }
    if(state == 1)
    {
      for(int i=0;i<NUMPIXELS;i++){
        strip_3.setPixelColor(i, strip_3.Color(0,0,255)); // Moderately bright green color.
        strip_4.setPixelColor(i, strip_4.Color(255,0,0)); // Moderately bright green color.
        if(voltageAlarm)
          strip_2.setPixelColor(i, 255, 0, 0);
      }
    }
    if(voltageAlarm)
      strip_2.show();
    strip_3.show();
    strip_4.show();
  }
  //We have new cell voltage data
  if(ap_cell_count > 1 && updateSys)
  {
    updateSys = 0;
    if(!voltageAlarm)
    {
       tmp = (ap_voltage_battery / ap_cell_count) - BATTERY_OK;
       tmp *= 8;    //We want to scale this to a number of LEDs to be green
       tmp /= (4200 - BATTERY_OK);
       for(int i = 0; i < tmp; i++)
       {
         strip_2.setPixelColor(i, 0, 255, 0);
       }
       for(int i = tmp; i < NUMPIXELS; i++)
       {
         strip_2.setPixelColor(i, 0, 0, 255);         
       }
       strip_2.show();
    }
  }
  //There is new GPS data to process and show
  if(updateGps)
  {
    updateGps = 0;
    if(ap_fixtype == 3)
    {
        red = 0;
        blue = 0;
        green = 255;
    }
    else if(ap_fixtype == 2)
    {
        red = 0;
        blue = 255;
        green = 255;
    }
    else
    {
       red = 255;
       blue = 0;
       green = 0;
    }
    for(int i= 0; i < ap_sat_visible; i++)
    {
       strip_1.setPixelColor(i, red, green, blue);
    }
    for(int i = ap_sat_visible; i < NUMPIXELS; i++)
    {
        strip_1.setPixelColor(i, 0, 0, 0);
    }
    strip_1.show();
  }
}

void comm_receive() { 
  mavlink_message_t msg; 
  mavlink_status_t status;
  //receive data over serial 
  while(Serial.available() > 0) 
  { 
    uint8_t c = Serial.read();
    //try to get a new message 
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    { 
      // Handle message
      digitalWrite(13, HIGH);
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
              apm_mav_system    = msg.sysid;
              apm_mav_component = msg.compid;
              apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);              
          }
          break;
        case MAVLINK_MSG_ID_SYS_STATUS:
          digitalWrite(13, LOW);
          ap_voltage_battery = mavlink_msg_sys_status_get_voltage_battery(&msg);  // 1 = 1mV
          ap_current_battery = mavlink_msg_sys_status_get_current_battery(&msg);     // 1=10mA
          
          if(ap_voltage_battery > 21000) ap_cell_count = 6;
          else if (ap_voltage_battery > 16800 && ap_cell_count != 6) ap_cell_count = 5;
          else if(ap_voltage_battery > 12600 && ap_cell_count != 5) ap_cell_count = 4;
          else if(ap_voltage_battery > 8400 && ap_cell_count != 4) ap_cell_count = 3;
          else if(ap_voltage_battery > 4200 && ap_cell_count != 3) ap_cell_count = 2;
          else ap_cell_count = 0;
          updateSys = 1;
          break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:   // 24
          digitalWrite(13, LOW);
          ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
          updateGps = 1;
          break;
        default:
          //Do nothing
        break;
      }
    } 
  }
}

