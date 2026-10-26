// Requires Adafruit NeoPixel and ArduinoJson 7 libraries.

#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
const uint16_t LED_COUNT=22;
const uint8_t LED_PIN=6;
Adafruit_NeoPixel ring(LED_COUNT,LED_PIN,NEO_GRB+NEO_KHZ800);
char buffer[4096]; size_t used=0; bool dropping=false;
unsigned long lastFrame=0;
void fault() { ring.fill(ring.Color(255,0,0)); ring.show(); }
void setup() { Serial.begin(115200); ring.begin(); ring.setBrightness(40); fault(); }
void loop() {
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='\n') {
      if(!dropping) {
        buffer[used]=0; JsonDocument doc;
        if(!deserializeJson(doc,buffer)) {
          JsonArray pixels=doc["pixels"].as<JsonArray>();
          bool seen[LED_COUNT]={false}; bool valid=pixels.size()==LED_COUNT;
          for(JsonArray p:pixels) {
            if(p.size()!=4) { valid=false; break; }
            int id=p[0].as<int>();
            if(id<0 || id>=LED_COUNT || seen[id]) { valid=false; break; }
            seen[id]=true;
            for(int k=1;k<4;k++) if(!p[k].is<int>() || p[k].as<int>()<0 || p[k].as<int>()>255) valid=false;
          }
          if(valid) {
            for(JsonArray p:pixels) ring.setPixelColor(p[0].as<int>(),ring.Color(p[1].as<int>(),p[2].as<int>(),p[3].as<int>()));
            ring.show(); lastFrame=millis();
          }
        }
      }
      used=0; dropping=false;
    } else if(!dropping) {
      if(used<sizeof(buffer)-1) buffer[used++]=c;
      else { dropping=true; used=0; }
    }
  }
  if(millis()-lastFrame>500) fault();
}
