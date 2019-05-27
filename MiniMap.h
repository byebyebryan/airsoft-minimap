#ifndef __MiniMap_h
#define __MiniMap_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

struct Msg {
  uint8_t type = 0;
  uint8_t id = 0;
  uint16_t ref_time = 0;
  uint16_t hdg = 0;
  float lat = 0.f;
  float lng = 0.f;
  uint32_t recv_at = 0;
};

struct Node {
  uint8_t id = 0;
  uint16_t ref_time = 0;
  uint16_t hdg = 0;
  float lat = 0.f;
  float lng = 0.f;
  uint32_t last_recv = 0;
};

void prepareMsg(uint8_t* buf, uint8_t type, uint8_t id, uint16_t ref_time, uint16_t heading, float lat, float lng);
Msg parseMsg(uint8_t* buf, uint8_t len);
uint32_t packInt(uint8_t type, uint8_t id, uint16_t ref_time, uint16_t heading);
void unpackInt(uint32_t packed, uint8_t& type, uint8_t& id, uint16_t& ref_time, uint16_t& heading);

void updateNode(Node& node, const Msg& msg);

#endif