// This #include statement was automatically added by the Particle IDE.
#include <neopixel.h>
#include <MQTT.h>
#include "math.h"

// Particle OS configuration
SYSTEM_THREAD(ENABLED);         // Enable system thread (https://docs.particle.io/reference/device-os/firmware/photon/#system-thread)
SYSTEM_MODE(SEMI_AUTOMATIC);    // Semi-automatic mode does not automatically connect to the cloud on boot

/**********     Defines     **********/
// Timer intervals
#define LED_UPDATE_INTERVAL 15            // interval at which the LED update function is run, ms
#define SENSOR_READ_INTERVAL 100          // interval at which the function that polls the sensor is run, ms
#define MQTT_CLIENT_INTERVAL 5000         // interval at which the function that sends data over MQTT is run, ms
#define STATE_MANAGER_INTERVAL 100        // interval at which the function that manages system state is run, ms

// Pins
#define PMS5003_SET D6                    // PMS5003 "SET" pin
#define PMS5003_nRST D3                   // PMS5003 "!RST" pin (active low)
#define LED_DATA_PIN D0                   // data out pin for the WS2812 LEDs

#define LED_COUNT 6
#define LED_TYPE WS2812B

// PM thresholds
#define PM10_THRESHOLD_RED 1000           // above this level, PM1.0 status is "red" (ug/m3)
#define PM10_THRESHOLD_YELLOW 750         // above this level, PM1.0 status is "yellow" (ug/m3)
#define PM25_THRESHOLD_RED 1000           // above this level, PM2.5 status is "red" (ug/m3)
#define PM25_THRESHOLD_YELLOW 750         // above this level, PM2.5 status is "yellow" (ug/m3)
#define PM100_THRESHOLD_RED 5000          // above this level, PM10 status is "red" (ug/m3)
#define PM100_THRESHOLD_YELLOW 3750       // above this level, PM10 status is "yellow" (ug/m3)

#define LED_SPINNER_PERIOD 750            // how long does it take the spinner to go around once? (ms)
#define SENSOR_WAKEUP_DELAY 10000         // how long should we wait after powering up the sensor to start collecting data? (ms)
                                          //    NB: the PMS5003 manual says we should wait at least 30 seconds "because of the fan's performance"
#define SENSOR_DATA_MAX_AGE 3000          // what's the oldest reading we can accept from the sensor? (ms)
#define EEPROM_VALIDITY_ADDR 0            // write 0 to this address when the EEPROM topic has been written OK
#define EEPROM_TOPIC_ADDR 4               // write the topic we want to use here (str)

#define HARDCODED_MQTT_TOPIC "livingroom"

/**********     Structs     **********/
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct sys_state {
  bool network_connected;
  bool cloud_connected;
  bool mqtt_connected;
  bool sensor_data_valid;
  bool sensor_initialized;
  uint32_t sensor_init_time_ms;
  uint32_t last_update_time_ms;         // used to track last update time before clock is synced
  uint32_t last_update_time_s;          // used to track last update time after clock is synced
  struct pms5003data sensor_data;
};

/**********     Globals     **********/
struct sys_state state;
String mqtt_topic_prefix;
String mqtt_topic_id;
//MQTT mqtt_conn("mqtt.custer.lan", 1883, mqtt_callback);
MQTT mqtt_conn("10.100.0.6", 1883, mqtt_callback);
Adafruit_NeoPixel leds(LED_COUNT, LED_DATA_PIN, LED_TYPE);

// Timers
Timer led_update_timer( LED_UPDATE_INTERVAL, gremlin );
Timer sensor_read_timer( SENSOR_READ_INTERVAL, sensor_read );
Timer mqtt_client_timer( MQTT_CLIENT_INTERVAL, mqtt_client );
Timer state_manager_timer( STATE_MANAGER_INTERVAL, state_manager );

/**********      Functions     **********/
/* MQTT message receive callback */
void mqtt_callback( char* topic, byte* payload, unsigned int length ){
    // copy received payload to null-terminated string
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

    // do nothing for now
    return;
}

void setup() {
    // Initialize state
    state.network_connected = false;
    state.cloud_connected = false;
    state.mqtt_connected = false;
    state.sensor_data_valid = false;
    state.sensor_initialized = false;
    state.sensor_init_time_ms = 0;
    state.last_update_time_ms = 0;
    state.last_update_time_s = 0;
    
    pinMode(PMS5003_SET, OUTPUT);        // sensor SET
    pinMode(PMS5003_nRST, OUTPUT);        // sensor !RST
    
    digitalWrite(PMS5003_SET, HIGH);
    digitalWrite(PMS5003_nRST, HIGH);
    
    state.sensor_initialized = true;
    state.sensor_init_time_ms = millis();      // Start the clock on the sensor warmup time.
                                            // Don't use now() here because we probably don't have a valid RTC time.
                                            
    // Set up LED
    leds.begin();
    leds.clear();
    leds.show();
    
    Serial.begin(115200);
    Serial1.begin(9600);
    
    // Kick off timers
    led_update_timer.start();
    sensor_read_timer.start();
    state_manager_timer.start();
    mqtt_client_timer.start();
    
    Particle.connect();
    
    // Configure MQTT topic
    mqtt_topic_prefix = String("custer/");
    mqtt_topic_id = get_device_mqtt_topic();
    
    // Connect MQTT 
    while( !mqtt_conn.isConnected() ){
        mqtt_conn.connect( "iaq-sensor-" + System.deviceID() );
        delay(1000);
    }
    
    Particle.function( "setTopic", writeMqttTopic );
    
}

/* Write the MQTT topic to EEPROM.
 *  For now this is just called as a cloud function due to laziness.
 */
int writeMqttTopic( String newTopic ){
    EEPROM.put(EEPROM_TOPIC_ADDR, newTopic.c_str());
    EEPROM.put(EEPROM_VALIDITY_ADDR, 0);
    
    uint8_t validity;
    String eepromTopic;
    
    EEPROM.get(EEPROM_TOPIC_ADDR, eepromTopic);
    EEPROM.get(EEPROM_VALIDITY_ADDR, validity);
    
    if( eepromTopic == newTopic && validity == 0 ){
        return 0;
    } else {
        return -1;
    }
}

/* This function handles the outbound communications over MQTT.
 *  If the sensor data is valid, will send out one message `<mqtt_topic_prefix>iaq_sensor/<mqtt_topic_id>/pm` with PM data.
 *      (Note the slashes - `mqtt_topic_prefix` should end with a / if set, but `mqtt_topic_id` should not.)
 *  If the sensor data is not valid, will send out ????? TODO.
 */
void mqtt_client(){
    String mqtt_topic;
    String mqtt_payload;
    
    if(state.mqtt_connected){
        if( state.sensor_data_valid ){
            // PM10
            mqtt_topic = String::format("%siaq_sensor/%s/pm10", mqtt_topic_prefix.c_str(), mqtt_topic_id.c_str());
            mqtt_payload = String::format("%u", state.sensor_data.pm100_env);
            mqtt_conn.publish( mqtt_topic, mqtt_payload );
            
            //PM2.5
            mqtt_topic = String::format("%siaq_sensor/%s/pm2.5", mqtt_topic_prefix.c_str(), mqtt_topic_id.c_str());
            mqtt_payload = String::format("%u", state.sensor_data.pm25_env);
            mqtt_conn.publish( mqtt_topic, mqtt_payload );
            
            //PM1.0
            mqtt_topic = String::format("%siaq_sensor/%s/pm1.0", mqtt_topic_prefix.c_str(), mqtt_topic_id.c_str());
            mqtt_payload = String::format("%u", state.sensor_data.pm10_env);
            mqtt_conn.publish( mqtt_topic, mqtt_payload );
        }
    }
}

/* This function is called on a regular interval by the `sensor_read_timer` timer.
 *  Its job is to look at the system state struct and update:
 *      - network_connected
 *      - cloud_connected
 *      - mqtt_connected
 *      - sensor_data_valid
 */
void state_manager(){
    state.network_connected = WiFi.ready();
    state.cloud_connected = Particle.connected();
    state.mqtt_connected = mqtt_conn.isConnected();

    Serial.printlnf("%u: net: %d | cloud: %d | mqtt: %d", millis(), state.network_connected, state.cloud_connected, state.mqtt_connected);
    
    if( state.sensor_initialized == true ){
        if( state.sensor_init_time_ms != 0 && (state.sensor_init_time_ms + SENSOR_WAKEUP_DELAY) < millis() ){
            // This is the first loop through state_manager() since we started the sensor
            state.sensor_init_time_ms = 0;
            
            // Reset last update timestamp - next read will be first valid read
            state.last_update_time_s = 0;
            state.last_update_time_ms = 0;
            state.sensor_data_valid = false;
        } else if(state.sensor_init_time_ms == 0){
            // if sensor_init_time == 0 we already passed the warmup time - don't check the warmup delay anymore to avoid overflow errors
            state.sensor_data_valid = is_sensor_data_fresh();
        } else {
            // We are still waiting for the sensor to warm up
            state.sensor_data_valid = false;
        }
    }
}

/* Return a String identifier for the device's MQTT topic.
 *  IAQ data is published to the topic `custer/iaq_sensor/<identifier>/pm`.
 *  If saved in EEPROM, this function will return the identifier from EEPROM.
 *  If not, will return an identifier based on the MAC address of the device.
 */
String get_device_mqtt_topic(){
    
    return String( HARDCODED_MQTT_TOPIC );      // fuck it
    uint8_t validityFlag;
    String topic;
    
    EEPROM.get( EEPROM_VALIDITY_ADDR, validityFlag );
    if( validityFlag == 0 ){
        // load topic from EEPROM
        EEPROM.get( EEPROM_TOPIC_ADDR, topic );
        return topic;
    } else {
        // set topic from device MAC
        byte mac[6];
        waitUntil( WiFi.ready );
        WiFi.macAddress(mac);
        return String::format("%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

/* Check to see if now() - state.last_update_time is more than the data expiry threshold.
 *  Return true if the data is valid, false otherwise.
 * This function attempts to handle RTC validity gracefully by checking UNIX time if available,
 *  and falling back to millis() if not.
 * Hopefully we can set the RTC before we've been powered on for 49 days...
 */
bool is_sensor_data_fresh(){
    // TODO: should lock system state to avoid multi-write bugs
    if( state.last_update_time_ms == 0 ){
        // We have never gotten data from the sensor, so it's not fresh
        return false;
    }
    
    if( Time.isValid() && state.last_update_time_s != 0 ){
        // Time is valid and the update time has been written since we got valid time.
        // OK to compare by RTC time.
        return (Time.now() - state.last_update_time_s) < (SENSOR_DATA_MAX_AGE/1000);
    } else {
        // RTC not valid or we haven't written since the RTC became valid
        return (millis() - state.last_update_time_ms) <  SENSOR_DATA_MAX_AGE;
    }
}

void gremlin(){
    uint32_t base_color, spin_color, ms;
    float spinner_angle;
    
    // If the sensor data is valid, we will display a "base color" from the reading
    if( state.sensor_data_valid ){
        if(     state.sensor_data.pm10_env > PM10_THRESHOLD_RED \
            ||  state.sensor_data.pm25_env > PM25_THRESHOLD_RED \
            || state.sensor_data.pm100_env > PM100_THRESHOLD_RED        ){
            // red zone!!!!
            base_color = leds.Color(255,0,0);
        } else if(      state.sensor_data.pm10_env > PM10_THRESHOLD_YELLOW \
                    ||  state.sensor_data.pm25_env > PM25_THRESHOLD_YELLOW \
                    ||  state.sensor_data.pm100_env > PM100_THRESHOLD_YELLOW ){
            // yellow zone ??
            base_color = leds.Color(255,211,0);
        } else {
            // GREEN ZONE
            base_color = leds.Color(0,255,0);
        }
    } else {
        // no valid sensor data - set base color to nothing
        base_color = leds.Color(0,0,0);
    }
    
    // Set spinner color if appropriate
    if( !state.network_connected || !state.cloud_connected || !state.mqtt_connected ){
        // later set individual colors here for each specific error condition
        spin_color = leds.Color(0,0,255);
        
        ms = millis() % LED_SPINNER_PERIOD;
        spinner_angle = (ms / (float)LED_SPINNER_PERIOD) * (2*3.14159265);
        spin_color = leds.Color(0,0,255);
        
        // Shared spinning (blend with base color)
        for(int i=0; i<6; i++){
            float frac;
            frac = max(0.0, cosf(spinner_angle - (2*3.14159265/6*i)));
            leds.setPixelColor(i, blend(base_color,spin_color,frac));
        }
    } else {
        // no other errors - just display the base color
        for(int i=0; i<6; i++){
            leds.setPixelColor(i, base_color);
        }
    }
    
    leds.show();
}

/* Blend two colors together, with a "bias" parameter `frac`.
 *  `a` and `b` are the colors to blend, in the AdaFruit NeoPixel packed 32-bit color format (RGB)/
 *  `frac` is a float between 0 and 1.
 *      When `frac` == 0.0, blend() returns `a`.
 *      When `frac` == 1.0, blend() returns `b`.
 *      When `frac` == 0.5, blend() returns an equal blend of both colors. 
 *      You get the idea.
 */
uint32_t blend( uint32_t a, uint32_t b, float frac){
    // bounds limit on blend fraction
    frac = min(1, max(0, frac));
    
    uint8_t a_r, a_g, a_b, b_r, b_g, b_b, o_r, o_g, o_b;
    float a_rf, a_gf, a_bf, b_rf, b_gf, b_bf;
    
    // Unpack uint32_t values
    a_r = (uint8_t)(a >> 16);
    a_g = (uint8_t)(a >> 8);
    a_b = (uint8_t)a;
    
    b_r = (uint8_t)(b >> 16);
    b_g = (uint8_t)(b >> 8);
    b_b = (uint8_t)b;
    
    a_rf = (float)a_r/255.0;
    a_gf = (float)a_g/255.0;
    a_bf = (float)a_b/255.0;
    
    b_rf = (float)b_r/255.0;
    b_gf = (float)b_g/255.0;
    b_bf = (float)b_b/255.0;
    
    o_r = (uint8_t)(255 * (a_rf * (1.0-frac) + b_rf * frac));
    o_g = (uint8_t)(255 * (a_gf * (1.0-frac) + b_gf * frac));
    o_b = (uint8_t)(255 * (a_bf * (1.0-frac) + b_bf * frac));
    
    return ((uint32_t)o_r << 16) | ((uint32_t)o_g <<  8) | o_b;
}

void loop() {

}

/* Wrapper function - called by the timer `sensor_read_timer` to check for new data */
void sensor_read(){
    readPMSdata(&Serial1);
}

bool readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if ( s->peek() != 0x42) {
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  char buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }

  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&(state.sensor_data), (void *)buffer_u16, 30);

  if (sum != state.sensor_data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  
  if(Time.isValid()){
      state.last_update_time_s = Time.now();
  }
  state.last_update_time_ms = millis();
  return true;
}


