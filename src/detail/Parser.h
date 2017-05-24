#ifndef OFXMODBUSORIENTAL_PARSER_H
#define OFXMODBUSORIENTAL_PARSER_H

#include <queue>
#include "Utils.h"

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_BEGIN

class Parser
{
public:
    
    enum class State { Addr, Func, Size, Data, Crc };
    
    struct Response
    {
        uint8_t addr;
        uint8_t func;
        uint8_t size;
        vector<uint8_t> data;
        uint16_t crc;
    };
    
    size_t available() { return _readBuffer.size(); }
    
    void clear() { reset(); }
    
    void pop() { _readBuffer.pop(); }
    
    const Response& front() const { return _readBuffer.front(); }
    
    void feed(const uint8_t* const data, const size_t size)
    {
        for (size_t i = 0; i < size; ++i) feed(data[i]);
    }
    
    void feed(uint8_t data)
    {
        switch(state)
        {
            case State::Addr:
            {
                reset();
                r_buffer.addr = data;
                crc.push(data);
                state = State::Func;
                break;
            }
            case State::Func:
            {
                r_buffer.func = data;
                crc.push(data);
                state = State::Size;
                break;
            }
            case State::Size:
            {
                r_buffer.size = data;
                crc.push(data);
                state = State::Data;
                break;
            }
            case State::Data:
            {
                r_buffer.data.push_back(data);
                crc.push(data);
                if (++count >= r_buffer.size) state = State::Crc;
                break;
            }
            case State::Crc:
            {
                if      (crc_count == 0) r_buffer.crc = ((uint16_t)data & 0x00FF);
                else if (crc_count == 1) r_buffer.crc |= (data << 8) & 0xFF00;
                
                if (++crc_count >= 2)
                {
                    if (r_buffer.crc == crc.get()) _readBuffer.push(r_buffer);
                    else ofLogError("invalid checksum") << (int)r_buffer.crc << " & " << (int)crc.get() << endl;
                    reset();
                }
                break;
            }
            default:
            {
                reset();
                break;
            }
        }
    }
    
    
private:
    
    void reset()
    {
        r_buffer.addr = r_buffer.func = r_buffer.size = 0;
        r_buffer.data.clear();
        crc.clear();
        count = crc_count = 0;
        state = State::Addr;
    }
    
    queue<Response> _readBuffer;
    
    Response r_buffer;
    CrcGenerator crc;
    uint8_t count {0};
    uint8_t crc_count {0};
    State state = State::Addr;
    
};


OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_END

#endif /* OFXMODBUSORIENTAL_PARSER_H */
