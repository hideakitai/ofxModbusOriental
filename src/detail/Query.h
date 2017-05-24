#ifndef OFXMODBUSORIENTAL_QUERY_H
#define OFXMODBUSORIENTAL_QUERY_H

#include <cstdint>
#include <array>
#include "Utils.h"

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_BEGIN

enum class CmdType
{
    // native modbus commands (0x007D)
    Start, Home, Stop, Free, Reset, JogFwd, JogBwd,
    // native modbus commands
    Select, Direct, Status, Diag,
    // broadcast commands
    StartAll, SetAll,
    // individual commands
    SetPos, SetVel, SetAcc, SetDec, SetFunc, SetCur,
    // N/A
    Clear, NA
};


class Query
{
public:
	virtual uint8_t* data() = 0;
	virtual size_t size() = 0;
    virtual uint32_t at(uint8_t id) = 0;
	virtual uint32_t operator[](uint8_t id) = 0;
    virtual void setID(uint8_t id) = 0;
};

template <size_t Size>
class QueryImpl : public Query
{
public:
	
    virtual uint8_t* data() override { setCrc(); return query.data(); }
    virtual size_t size() override { return query.size(); }
    virtual uint32_t at(uint8_t id) override
    {
        uint32_t data = 0;
        size_t offset = id * sizeof(uint32_t) + val_offset;
        data |= (uint32_t)((query.data()[offset + 0] << 24) & 0xFF000000);
        data |= (uint32_t)((query.data()[offset + 1] << 16) & 0x00FF0000);
        data |= (uint32_t)((query.data()[offset + 2] <<  8) & 0x0000FF00);
        data |= (uint32_t)((query.data()[offset + 3] <<  0) & 0x000000FF);
        return data;
    }
    virtual uint32_t operator[](uint8_t id) override { return at(id); }
	
    virtual void setID(uint8_t id) override { query[0] = id; }
	
    void setFunc(uint8_t func) { query[1] = func; }
    
    void setAddr(uint16_t addr)
    {
        query[2] = (addr >> 8) & 0xFF;
        query[3] = (addr >> 0) & 0xFF;
    }
    
    void setRegSize(uint8_t size)
    {
        query[4] = 0x00;
        query[5] = size;
    }

    void setRegBytes(uint8_t size)
    {
        query[6] = size;
    }
    
    void setValue32(uint8_t offset, uint32_t val)
    {
        query[offset + 0] = (val >> 24) & 0xFF;
        query[offset + 1] = (val >> 16) & 0xFF;
        query[offset + 2] = (val >>  8) & 0xFF;
        query[offset + 3] = (val >>  0) & 0xFF;
    }
    
    void setValue16(uint8_t offset, uint16_t val)
    {
        query[offset + 0] = 0;
        query[offset + 1] = 0;
        query[offset + 2] = (val >>  8) & 0xFF;
        query[offset + 3] = (val >>  0) & 0xFF;
    }
    
    void setValue8(uint8_t offset, uint8_t val)
    {
        query[offset + 0] = 0;
        query[offset + 1] = 0;
        query[offset + 2] = 0;
        query[offset + 3] = val;
    }

    void setCrc()
    {
        uint16_t crc16 = crc.get(query.data(), query.size() - 2);
        auto it = std::end(query);
        *(--it) = crc16 >> 8;
        *(--it) = crc16 & 0xFF;
    }
    
    
public:

    std::array<uint8_t, Size> query;
    CrcGenerator crc;
    const uint8_t val_offset = 7;
    
};

class RemoteIOs : public QueryImpl<13>
{
    std::unordered_map<CmdType, uint16_t, EnumClassHash> remote_ios_val
    {
        {CmdType::Start,  0x0008},
        {CmdType::Home,   0x0010},
        {CmdType::Stop,   0x0020},
        {CmdType::Free,   0x0040},
        {CmdType::Reset,  0x0080},
        {CmdType::JogFwd, 0x1000},
        {CmdType::JogBwd, 0x2000},
        {CmdType::Clear,  0x0000},
    };
    
    
public:
    
    RemoteIOs(CmdType cmd, uint8_t id = 0)
    {
        setID(id);
        setFunc(0x10);
        setAddr(0x007C);
        setRegSize(2);
        setRegBytes(4);
        setCommand(cmd);
    }
    
    void setCommand(CmdType cmd) { setValue32(val_offset, remote_ios_val[cmd]); }
};



class NetSelect : public QueryImpl<13>
{
public:
    
    NetSelect(uint8_t no, uint8_t id = 0)
    {
        setID(id);
        setFunc(0x10);
        setAddr(0x007A);
        setRegSize(2);
        setRegBytes(4);
        setValue8(val_offset, no);
    }
    
};


class JogSteps : public QueryImpl<13>
{
public:
    
    JogSteps(uint32_t steps, uint8_t id = 0)
    {
        setID(id);
        setFunc(0x10);
        setAddr(0x02A0);
        setRegSize(2);
        setRegBytes(4);
        setValue32(val_offset, steps);
    }
};


class JogSpeed : public QueryImpl<13>
{
public:
    
    JogSpeed(uint32_t speed, uint8_t id = 0)
    {
        setID(id);
        setFunc(0x10);
        setAddr(0x02A2);
        setRegSize(2);
        setRegBytes(4);
        setValue32(val_offset, speed);
    }
};


class Origin : public QueryImpl<13>
{
public:
    
    Origin(int32_t pos, uint8_t id = 0)
    {
        setID(id);
        setFunc(0x10);
        setAddr(0x038C);
        setRegSize(2);
        setRegBytes(4);
        setValue32(val_offset, pos);
    }
};


class DirectDrive : public QueryImpl<41>
{
    const uint8_t no_offset = 7;
    const uint8_t mode_offset = 11;
    const uint8_t pos_offset = 15;
    const uint8_t vel_offset = 19;
    const uint8_t acc_offset = 23;
    const uint8_t dec_offset = 27;
    const uint8_t crnt_offset = 31;
    const uint8_t trig_offset = 35;
    
public:
    
    DirectDrive(uint8_t id = 0)
    {
        setID(id);
        setFunc(0x10);
        setAddr(0x58);
        setRegSize(0x10);
        setRegBytes(0x20);
        setDriveNo(0xFF);
        setDriveMode(0x01);
        setPosition(0);
        setVelocity(0);
        setAcceleration(0);
        setDeacceleration(0);
        setCurrent(0x03E8);
        setTrigger(1);
    }
    
    void setDriveNo(uint8_t no) { setValue8(no_offset, no); }
    void setDriveMode(uint8_t mode) { setValue8(mode_offset, mode); }
    void setPosition(uint32_t pos) { setValue32(pos_offset, pos); }
    void setVelocity(uint32_t vel) { setValue32(vel_offset, vel); }
    void setAcceleration(uint32_t acc) { setValue32(acc_offset, acc); }
    void setDeacceleration(uint32_t dec) { setValue32(dec_offset, dec); }
    void setCurrent(uint32_t crnt) { setValue32(crnt_offset, crnt); }
    void setTrigger(char trig) { setValue8(trig_offset, (uint8_t)trig); }
};


OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_END

#endif /* OFXMODBUSORIENTAL_QUERY_H */
