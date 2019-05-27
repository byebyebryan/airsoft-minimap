#include "MiniMap.h"

void prepareMsg(uint8_t* buf, uint8_t type, uint8_t id, uint16_t ref_time, uint16_t heading, float lat, float lng) {
  *(uint32_t*)(buf) =  packInt(type, id, ref_time, heading);
  *(float*)(buf+4) = lat;
  *(float*)(buf+8) = lng;
}

Msg parseMsg(uint8_t* buf, uint8_t len) {
  Msg res;
  res.recv_at = millis();
  uint32_t packed = *(uint32_t*)(buf);
  unpackInt(packed, res.type, res.id, res.ref_time, res.hdg);
  res.lat = *(float*)(buf+4);
  res.lng = *(float*)(buf+8);
  return res;
}

uint32_t packInt(uint8_t type, uint8_t id, uint16_t ref_time, uint16_t heading) {
  uint32_t res = (uint32_t) type;
  res |= ((uint32_t)id << 4);
  res |= ((uint32_t)ref_time << 8);
  res |= ((uint32_t)heading << 20);
  return res;
}

void unpackInt(uint32_t packed, uint8_t& type, uint8_t& id, uint16_t& ref_time, uint16_t& heading) {
  type = packed & 0xf;
  id = (packed >> 4) & 0xf;
  ref_time = (packed >> 8) & 0xfff;
  heading = (packed >> 20) & 0xfff;
}

void updateNode(Node& node, const Msg& msg) {
  node.ref_time = msg.ref_time;
  node.hdg = msg.hdg;
  node.lat = msg.lat;
  node.lng = msg.lng;
  node.last_recv = msg.recv_at;
}