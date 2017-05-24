#ifndef OFXMODBUSORIENTAL_BUFFER_H
#define OFXMODBUSORIENTAL_BUFFER_H

#include "Query.h"

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_BEGIN

class ConcurrentValue : public QueryImpl<255>
{
    const uint8_t max_drive_no_size = 59;
    const uint8_t reg_size = 4;
    
public:
    
    ConcurrentValue()  {}
    ~ConcurrentValue() {}
    
    void set(uint8_t id, uint32_t val) { setValue32(val_offset + reg_size * id, val); }
    size_t getDriveNoSize() { return max_drive_no_size + 1; }
};


class ConcurrentPosition : public ConcurrentValue
{
public:
    ConcurrentPosition()
    {
        setID(0x00);
        setFunc(0x10);
        setAddr(0x0400);
        setRegSize(0x7B);
        setRegBytes(0x7B * 2);
    }
};

class ConcurrentVelocity : public ConcurrentValue
{
public:
    ConcurrentVelocity()
    {
        setID(0x00);
        setFunc(0x10);
        setAddr(0x0480);
        setRegSize(0x7B);
        setRegBytes(0x7B * 2);
    }
};

class ConcurrentMode : public ConcurrentValue
{
public:
    ConcurrentMode()
    {
        setID(0x00);
        setFunc(0x10);
        setAddr(0x0500);
        setRegSize(0x7B);
        setRegBytes(0x7B * 2);
    }
};

class ConcurrentAcceleration : public ConcurrentValue
{
public:
    ConcurrentAcceleration()
    {
        setID(0x00);
        setFunc(0x10);
        setAddr(0x0600);
        setRegSize(0x7B);
        setRegBytes(0x7B * 2);
    }
};

class ConcurrentDeacceleration : public ConcurrentValue
{
public:
    ConcurrentDeacceleration()
    {
        setID(0x00);
        setFunc(0x10);
        setAddr(0x0680);
        setRegSize(0x7B);
        setRegBytes(0x7B * 2);
    }
};

class ConcurrentCurrent : public ConcurrentValue
{
public:
    ConcurrentCurrent()
    {
        setID(0x00);
        setFunc(0x10);
        setAddr(0x0700);
        setRegSize(0x7B);
        setRegBytes(0x7B * 2);
    }
};


class Buffer
{
	std::shared_ptr<ConcurrentPosition> pos;
	std::shared_ptr<ConcurrentVelocity> vel;
	std::shared_ptr<ConcurrentMode> mode;
	std::shared_ptr<ConcurrentAcceleration> acc;
	std::shared_ptr<ConcurrentDeacceleration> dec;
	std::shared_ptr<ConcurrentCurrent> crnt;
	
    enum class State { Idle, Pos, Vel, Start, Clear };
    State state { State::Idle };

public:
    
    Buffer()
    {
        pos = std::make_shared<ConcurrentPosition>();
        vel = std::make_shared<ConcurrentVelocity>();
        mode = std::make_shared<ConcurrentMode>();
        acc = std::make_shared<ConcurrentAcceleration>();
        dec = std::make_shared<ConcurrentDeacceleration>();
        crnt = std::make_shared<ConcurrentCurrent>();
    }
    
    using DataRef = std::shared_ptr<Query>;
    
    void setPosition(uint8_t id, int32_t p) { pos->set(id, (uint32_t)p); }
    void setVelocity(uint8_t id, int32_t v) { vel->set(id, (uint32_t)v); }
    void setMode(uint8_t id, uint8_t m) { mode->set(id, m); }
    void setAcceleration(uint8_t id, uint32_t a) { acc->set(id, a); }
    void setDeacceleration(uint8_t id, uint32_t d) { dec->set(id, d); }
    void setCurrent(uint8_t id, uint32_t c) { crnt->set(id, c); }
    
    DataRef getPositionRef() { return pos; }
	DataRef getVelocityRef() { return vel; }
	DataRef getModeRef() { return mode; }
	DataRef getAccelerationRef() { return acc; }
	DataRef getDeaccelerationRef() { return dec; }
	DataRef getCurrentRef() { return crnt; }
    
    int32_t getPosition(uint8_t id) { return (int32_t)pos->at(id); }
	int32_t getVelocity(uint8_t id) { return (int32_t)vel->at(id); }
	uint8_t getMode(uint8_t id) { return (uint8_t)mode->at(id); }
	uint32_t getAcceleration(uint8_t id) { return acc->at(id); }
	uint32_t getDeacceleration(uint8_t id) { return dec->at(id); }
	uint32_t getCurrent(uint8_t id) { return crnt->at(id); }
    
    size_t size() { return pos->getDriveNoSize(); }
	
};


OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_END

#endif /* OFXMODBUSORIENTAL_BUFFER_H */
