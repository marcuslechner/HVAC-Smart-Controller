#include <dht11.h>
dht11 DHT;
const int dht_data = 12;
int temp = 0;
int humid = 0;

void setup() 
{
  Serial.begin(9600);

}

void loop() 
{
  DHT.read(dht_data);
  temp=DHT.temperature;
  humid=DHT.humidity;
  Serial.print(temp);
  Serial.print("\t");
  Serial.println(humid);
  delay(500);
}
