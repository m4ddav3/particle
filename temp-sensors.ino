#define WITH_DALLAS
#define NO_ONE_WIRE_DEBUG

#ifdef WITH_DALLAS
#include "OneWire/OneWire.h"
#endif

#include "SparkFun-Spark-Phant/SparkFun-Spark-Phant.h"
#include "Adafruit_DHT/Adafruit_DHT.h"
#include "Adafruit_BME280/Adafruit_BME280.h"

SYSTEM_MODE(AUTOMATIC);

#define SAMPLE_DELAY_MINS 2

#define DHTPIN  D6
#define DHTTYPE DHT22
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BME280 bme;

#ifdef WITH_DALLAS
#define DALLAS_PIN D2
OneWire ds = OneWire(DALLAS_PIN);

typedef byte OneWireAddress[8];

OneWireAddress ds_sensors[4] = {
    { 0x28, 0x61, 0x1b, 0x1f, 0x03, 0x00, 0x00, 0xa0 },
    { 0x28, 0x9e, 0xf0, 0x1e, 0x03, 0x00, 0x00, 0xd3 },
    { 0x28, 0xb8, 0xf3, 0x1e, 0x03, 0x00, 0x00, 0x99 },
    { 0x28, 0xd7, 0x0b, 0x1f, 0x03, 0x00, 0x00, 0x69 },
};

void collect_ds_temps();
#endif

const char server[] = "192.168.1.88";
#define PORT 8080
const char publicKey[]  = "db8bQQj2eAFDgK1w9NQJIj2Popr";
const char privateKey[] = "eeWeyy0MgLHnDOvPGqBYImyjnl9";
const char deleteKey[]  = "wNPNddRkx0hGe8o3lEQWHzaygYX";
Phant phant(server, publicKey, privateKey);

double temperature;
double humidity;
int analog0;

double bme_temp;
double bme_hum;
double bme_pres;

double wire_28611b1f300a0;
double wire_289ef01e300d3;
double wire_28b8f31e30099;
double wire_28d7b1f30069;

String one_wire_sensors;
OneWireAddress one_wire_sensor_list[MIN_SENSOR_LIST_SIZE];
uint8_t num_one_wire_sensors = 0;

int one_wire_detect() {
    // Search the bus and add found devices to one_wire_sensor_list
}

void setup() {
    // Set the oboard RGB LED to 25% brightness
    RGB.control(true);
    RGB.brightness(64);
    RGB.control(false);
    
#ifdef WITH_DALLAS
    ds.reset_search();
    // TODO Probably want to issue a reset to all the 1-wire sensors?
#endif
    
    Particle.variable("temperature", temperature);
    Particle.variable("humidity"   , humidity);
    Particle.variable("analog0"    , analog0);
    Particle.variable("bme-temp"   , bme_temp);
    Particle.variable("bme-hum"    , bme_hum);
    Particle.variable("bme-pres"   , bme_pres);
    
    Particle.variable("one-wire-sensors", one_wire_sensors, STRING);
    Particle.function("one-wire-detect", one_wire_detect);
    
    //IPAddress myNetMask = WiFi.subnetMask();
    //Particle.publish("device-ip", String(myIP) + "/" + String(myNetMask));
    
    dht.begin();
    
    if (!bme.begin(0x76)) {
        Particle.publish("No bme at 0x76");
        while (1) delay(1000);
    };
}

void pause_loop();
void send_to_phant();

int publish_minute = -1;

void loop() {
    if (publish_minute == -1 || publish_minute == Time.minute()) {
        // Moved the sample reading into here to try to improve the reading
        humidity    = (double) dht.getHumidity();
        temperature = (double) dht.getTempCelcius();
        
        bme_temp = (double) bme.readTemperature();
        bme_hum  = (double) bme.readHumidity();
        bme_pres = (double) (bme.readPressure() / 100.0F);
        
        analog0 = analogRead(A0);
        analog0 += analogRead(A0);
        analog0 /= 2;
        
        publish_minute = (Time.minute() + SAMPLE_DELAY_MINS) % 60;
        
#ifdef WITH_DALLAS
        collect_ds_temps();
#endif
        send_to_phant();
    }
    
    delay(5000);
}

void send_to_phant() {
    phant.add("dht_temperature", temperature);
    phant.add("dht_humidity", humidity);
    phant.add("analog0", analog0);
    phant.add("bme_temperature", bme_temp);
    phant.add("bme_humidity", bme_hum);
    phant.add("bme_pressure", bme_pres);

    phant.add("28611b1f300a0", wire_28611b1f300a0);
    phant.add("289ef01e300d3", wire_289ef01e300d3);
    phant.add("28b8f31e30099", wire_28b8f31e30099);
    phant.add("28d7b1f30069", wire_28d7b1f30069);

    TCPClient client;
    uint8_t attempts = 0;
    bool success = false;
    
    String phant_request = phant.post();
    
    for (int i = 0; i < 5; i++) {
        if (client.connect(server, PORT)) {
            byte written = client.print(phant_request);
            
            String tries = "";
            if (attempts > 0) {
                tries = String::format(", attempted %d times", attempts);
            }
            
            success = true;
            
            Particle.publish("send-to-phant", String::format("Wrote %d bytes%s", written, tries.c_str()));
            
            break;
        }
        else {
            attempts++;
        }
        delay(1000);
    }
    
    if (! success) {
        Particle.publish("send-to-phant", "Failed to send");
    }
    
    phant_request = "";
}


#ifdef WITH_DALLAS

void read_ds_sensor(byte addr[], byte data[], byte type_s) {
    uint8_t i = 0;
    uint8_t attempts = 0;
    bool crc_okay = false;
    
    for (attempts = 0; attempts < 5; attempts++){ 
        ds.reset();
        
        ds.select(addr);
        ds.write(0xBE,0);                    // Read Scratchpad
        if (type_s == 2) ds.write(0x00,0);   // The DS2438 needs a page# to read
        
        for ( i = 0; i < 9; i++) {           // we need 9 bytes
            data[i] = ds.read();
        }

        uint8_t crc = OneWire::crc8(data, 8);
        
        if (crc != data[8]) {
            String foo = String("Bad CRC: Got ");
            foo.concat(String(data[8], HEX));
            foo.concat(", wanted ");
            foo.concat(String(crc, HEX));
            Particle.publish("one-wire", foo);
        }
        else {
            break;
        }
    }
}

void collect_ds_temps() {
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    float celsius;
    
    uint8_t sensor = 0;
    
    wire_28611b1f300a0 = 0.0f;
    wire_289ef01e300d3 = 0.0f;
    wire_28b8f31e30099 = 0.0f;
    wire_28d7b1f30069  = 0.0f;

    byte sensor_index = 0;
    
    ds.reset();               // first clear the 1-wire bus
    ds.skip();                // don't select any address, i.e. broadcast
    ds.write(0x44, 0);        // or start conversion in powered mode (bus finishes low)

    delay(800);
    
    for (sensor_index = 0; sensor_index < 4; sensor_index++) {
        memcpy(addr, ds_sensors[sensor_index], sizeof(OneWireAddress));
        
#ifdef ONE_WIRE_DEBUG
        String device_info = "";
        for (byte i = 0; i < 8; i++) {
            if (addr[i] < 16) device_info.concat('0');
            device_info.concat(String(addr[i], HEX));
        }
        Particle.publish("one-wire-debug", device_info);
#endif
        
        // Get temp
        read_ds_sensor(addr, data, 0);
        // Convert temp
        int16_t raw = (data[1] << 8) | data[0];
        if (type_s == 2) raw = (data[2] << 8) | data[1];
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7; //  9 bit res,  93.75 ms
        if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        // default is 12 bit resolution, 750 ms conversion time
        celsius = (float)raw * 0.0625;
        
        // Save temp
        switch (sensor_index) {
            case 0: wire_28611b1f300a0 = celsius; break;
            case 1: wire_289ef01e300d3 = celsius; break;
            case 2: wire_28b8f31e30099 = celsius; break;
            case 3: wire_28d7b1f30069  = celsius; break;
        }
    }
}
#endif
