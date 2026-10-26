/* Teensy 4.1, six VL53L7CX, native Ethernet UDP.
 * Requires stm32duino VL53L7CX and QNEthernet libraries.
 * Matches hrc_ros2/hrc_interfaces and the uploaded Python publisher.
 * Compile and measure on actual hardware before relying on timing.
 */
#include <Arduino.h>
#include <Wire.h>
#include <QNEthernet.h>
#include <vl53l7cx_class.h>
#include "config.h"
#include "packet_format.h"
using namespace qindesign::network;

// Two sensors per bus. IDs are logical S1..S6, not physical ring order.
VL53L7CX s1(&Wire, LPN_PINS[0]);
VL53L7CX s2(&Wire, LPN_PINS[1]);
VL53L7CX s3(&Wire1,LPN_PINS[2]);
VL53L7CX s4(&Wire1,LPN_PINS[3]);
VL53L7CX s5(&Wire2,LPN_PINS[4]);
VL53L7CX s6(&Wire2,LPN_PINS[5]);
VL53L7CX *sensors[6]={&s1,&s2,&s3,&s4,&s5,&s6};
// ST API takes 8-bit addresses (7-bit Wire addresses are 0x2A,0x2B).
constexpr uint8_t ADDRESSES[6]={0x54,0x56,0x54,0x56,0x54,0x56};
tof::SensorFrame frames[6];
VL53L7CX_ResultsData results; // Sequential reads: one shared scratch buffer.
EthernetUDP udp;
uint8_t packet[tof::PacketSize];
uint32_t packetSeq=0,lastPacketMs=0,txOK=0,txFailed=0;
uint32_t reads[6]={};
uint8_t stream[6]={};
bool haveStream[6]={};
bool netReady=false;

bool startSensor(size_t i) {
  auto &s=*sensors[i]; auto &f=frames[i];
  int rc=s.init_sensor(ADDRESSES[i]);
  if(rc==0) { f.init_ok=1; rc=s.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8); }
  if(rc==0) rc=s.vl53l7cx_set_ranging_mode(VL53L7CX_RANGING_MODE_CONTINUOUS);
  if(rc==0) rc=s.vl53l7cx_set_ranging_frequency_hz(RANGING_HZ);
  if(rc==0) rc=s.vl53l7cx_start_ranging();
  f.range_ok=(rc==0);
  if(rc!=0) {
    f.data_ok=0; s.vl53l7cx_off(); // Remove failed device from the bus.
  }
  Serial.printf("S%u: init=%u ranging=%u rc=%d\n",unsigned(i+1),f.init_ok,f.range_ok,rc);
  return rc==0;
}

void pollSensor(size_t i) {
  auto &s=*sensors[i]; auto &f=frames[i];
  if(!f.init_ok || !f.range_ok) return;
  uint8_t ready=0;
  uint8_t rc=s.vl53l7cx_check_data_ready(&ready);
  if(rc!=0) { f.data_ok=0; f.fresh=0; return; }
  if(!ready) return;
  rc=s.vl53l7cx_get_ranging_data(&results);
  if(rc!=0) { f.data_ok=0; f.fresh=0; return; }
  const uint8_t count=s.get_stream_count();
  if(haveStream[i] && count==stream[i]) return;
  haveStream[i]=true; stream[i]=count;
  for(size_t z=0;z<tof::ZoneCount;++z) {
    // First target per zone, preserving native zone order and raw ST status.
    const size_t k=z*VL53L7CX_NB_TARGET_PER_ZONE;
    const int32_t d=results.distance_mm[k];
    f.distance_mm[z]=(d>0 && d<=65535)?uint16_t(d):0;
    f.status[z]=results.target_status[k];
  }
  ++f.sequence; ++reads[i]; f.timestamp_ms=millis(); f.data_ok=1; f.fresh=1;
}

void sendPacket(uint32_t now) {
  // Stale/error metadata must not masquerade as fresh measurements.
  for(auto &f:frames) {
    if(!f.data_ok || uint32_t(now-f.timestamp_ms)>FRAME_TIMEOUT_MS) {
      f.fresh=0; f.data_ok=0;
    }
  }
  tof::encode(packet,packetSeq++,now,frames);
  bool ok=false;
  if(netReady && Ethernet.linkState() && udp.beginPacket(PC_IP,DEST_PORT)) {
    const size_t written=udp.write(packet,sizeof(packet));
    const bool sent=udp.endPacket();
    ok=written==sizeof(packet) && sent;
  }
  if(ok) ++txOK; else ++txFailed;
  // Do not replay freshness after a failed send. Next measured frame will retry.
  for(auto &f:frames) f.fresh=0;
  lastPacketMs=now;
}

void setup() {
  Serial.begin(115200);
  const uint32_t start=millis();
  while(!Serial && uint32_t(millis()-start)<1500) yield();
  // Keep every LPn low until its address has been assigned individually.
  for(size_t i=0;i<6;++i) {
    sensors[i]->begin(); frames[i].id=i+1;
    for(auto &v:frames[i].status) v=255;
  }
  Wire.begin(); Wire1.begin(); Wire2.begin();
  Wire.setClock(I2C_HZ); Wire1.setClock(I2C_HZ); Wire2.setClock(I2C_HZ);
  delay(20);
  for(size_t i=0;i<6;++i) startSensor(i);
  netReady=Ethernet.begin(TEENSY_IP,NETMASK,GATEWAY);
  if(netReady) netReady=udp.begin(LOCAL_PORT);
  Serial.printf("UDP initialized=%u, packet bytes=%u\n",netReady,unsigned(sizeof(packet)));
  lastPacketMs=millis();
}

void loop() {
  for(size_t i=0;i<6;++i) { pollSensor(i); yield(); }
  const uint32_t now=millis();
  bool allFresh=true;
  for(const auto &f:frames) if(!f.fresh || !f.data_ok) allFresh=false;
  // A full cycle normally yields one packet. A failed sensor must not stop telemetry.
  if(allFresh || uint32_t(now-lastPacketMs)>=HEARTBEAT_MS) sendPacket(now);
  static uint32_t logMs=0;
  if(uint32_t(now-logMs)>=1000) {
    if(Serial && Serial.availableForWrite()>160) {
      Serial.printf("tx=%lu fail=%lu | fresh reads S1..S6: %lu %lu %lu %lu %lu %lu\n",
        (unsigned long)txOK,(unsigned long)txFailed,
        (unsigned long)reads[0],(unsigned long)reads[1],(unsigned long)reads[2],
        (unsigned long)reads[3],(unsigned long)reads[4],(unsigned long)reads[5]);
    }
    txOK=txFailed=0; for(auto &n:reads) n=0;
    logMs=now;
  }
  yield();
}
