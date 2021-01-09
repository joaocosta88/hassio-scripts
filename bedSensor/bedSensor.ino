#include <WiFi.h>
#include <PubSubClient.h>

const char * ssid = "MEO-FEEE10";
const char * password = "56c75ccd2d";

const char * mqtt_server = "192.168.1.87";
int          mqtt_port = 1883;
const char * mqtt_username = "mqtt";
const char * mqtt_password = "mqtt";

const char * mqtt_topic_bed_left = "sensor/bed/left";
int          bed_left_gpio_pin = 34;
const char * mqtt_topic_bed_right = "sensor/bed/right";
int          bed_right_gpio_pin = 35;


WiFiClient espClient;
PubSubClient client(espClient);


void setup()
{
  Serial.begin(115200);

  // Setup WiFi
  connectWifi();
  connectMqtt();

  Serial.println("Setup done");
}

void loop()
{
  int gpio_left_value = analogRead(bed_left_gpio_pin);
  int gpio_right_value = analogRead(bed_right_gpio_pin);

  double leftWeight = convertToNewtons(gpio_left_value);
  double rightWeight = convertToNewtons(gpio_right_value);

  publishValueToMqtt(leftWeight, mqtt_topic_bed_left);
  publishValueToMqtt(rightWeight, mqtt_topic_bed_right);

  delay(3000);
}

void connectWifi() {
  Serial.println("Connecting to Wifi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());
}

void connectMqtt() {
  Serial.println("Connecting to MQTT.");

  client.setServer(mqtt_server, mqtt_port);

  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("BedSensor", mqtt_username, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
}

double convertToNewtons(int value) {

  if (value == 4095)
  {
    return 0;
  }
  else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 3.3V
    // so FSR = ((Vcc - V) * R) / V
    unsigned long  fsrResistance = value;     // fsrVoltage is in millivolts so 3.3V = 3300mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= (4095 - value);
    Serial.print("FSR resistance in ohms = ");
    Serial.println(fsrResistance);

    double fsrConductance = 1000000;           // we measure in micromhos so
    fsrConductance /= fsrResistance;
    Serial.print("Conductance in microMhos: ");
    Serial.println(fsrConductance);

    double fsrForce = 0;
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);
    }

    return fsrForce;
  }
}

void publishValueToMqtt(double value, const char* topic) {

  char buffer[10];
  dtostrf(value, 0, 0, buffer);

  client.publish(topic, buffer);

}
