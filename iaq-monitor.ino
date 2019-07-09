// This #include statement was automatically added by the Particle IDE.
#include <neopixel.h>
#include <MQTT.h>

// Particle OS configuration


/**********     Defines     **********/
// Timer intervals
#define LED_UPDATE_INTERVAL 15            // interval at which the LED update function is run, ms
#define SENSOR_READ_INTERVAL 100          // interval at which the function that polls the sensor is run, ms
#define MQTT_CLIENT_INTERVAL 1000         // interval at which the function that sends data over MQTT is run, ms
#define STATE_MANAGER_INTERVAL 100        // interval at which the function that manages system state is run, ms

// Pins
#define PMS5003_SET D6                    // PMS5003 "SET" pin
#define PMS5003_nRST D3                   // PMS5003 "!RST" pin (active low)

// PM thresholds
#define PM10_THRESHOLD_RED 1000           // above this level, PM1.0 status is "red" (ug/m3)
#define PM10_THRESHOLD_YELLOW 750         // above this level, PM1.0 status is "yellow" (ug/m3)
#define PM25_THRESHOLD_RED 1000           // above this level, PM2.5 status is "red" (ug/m3)
#define PM25_THRESHOLD_YELLOW 750         // above this level, PM2.5 status is "yellow" (ug/m3)
#define PM100_THRESHOLD_RED 5000          // above this level, PM10 status is "red" (ug/m3)
#define PM100_THRESHOLD_YELLOW 3750       // above this level, PM10 status is "yellow" (ug/m3)

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
  int last_update_time;
  pms5003_data sensor_data;
};


/**********     Globals     **********/
struct sys_state state;
MQTT mqtt_client("10.100.0.6", 1883, mqtt_callback);

// Timers
Timer led_update_timer( LED_UPDATE_INTERVAL, led_update );
Timer sensor_read_timer( SENSOR_READ_INTERVAL, sensor_read );
Timer mqtt_client_timer( MQTT_CLIENT_INTERVAL, mqtt_client );
Timer state_manager_timer( STATE_MANAGER_INTERVAL, state_manager );


/**********      Functions     **********/
/* MQTT message receive callback */
void mqtt_callback( char* topic, byte* payload, unsigned int length ){
    // do nothing for now
    return;
}

void setup() {
    pinMode(PMS5003_SET, OUTPUT);        // sensor SET
    pinMode(PMS5003_nRST, OUTPUT);        // sensor !RST
    
    digitalWrite(PMS5003_SET, HIGH);
    digitalWrite(PMS5003_nRST, HIGH);
    
    // TODO - need to pause to allow sensor to initialize
    
    Serial.begin(115200);
    Serial1.begin(9600);
    
    // Initialize state
    state.network_connected = false;
    state.cloud_connected = false;
    state.mqtt_connected = false;
    state.sensor_data_valid = false;
    state.last_update_time = 0;
    
    // Kick off timers
    led_update_timer.start();
    
//    Serial.println("I am alive");
    
     /* Connect MQTT */
//    mqtt_client.connect( "iaq-sensor-" + System.deviceID() );
    
}

void loop() {
    last_read_attempt_time = Time.now();
    if( readPMSdata(&Serial1) ){
        // yay data read success
        last_successful_read_time = Time.now();
        pm10_std = data.pm10_standard;
        pm25_std = data.pm25_standard;
        pm100_std = data.pm100_standard;
        
        Serial.println();
        Serial.println("---------------------------------------");
        Serial.println("Concentration Units (standard)");
        Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
        Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
        Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
        Serial.println("---------------------------------------");
        Serial.println("Concentration Units (environmental)");
        Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
        Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
        Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
        Serial.println("---------------------------------------");
        Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
        Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
        Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
        Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
        Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
        Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
        Serial.println("---------------------------------------");
        
        if( mqtt_client.isConnected() ){
            mqtt_client.publish( "custer/iaq/pm10", String( data.pm10_standard, DEC) );
            mqtt_client.publish( "custer/iaq/pm25", String( data.pm25_standard, DEC) );
            mqtt_client.publish( "custer/iaq/pm100", String( data.pm100_standard, DEC) );
        }
        
        
    }
    
   // delay(1000);

}

bool readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if ( s->peek() != 0x42) {
    return false;
  }

    Serial.println("Found the 0x42 start byte");

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
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}