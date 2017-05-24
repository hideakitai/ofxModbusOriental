#ifndef OFXMODBUSORIENTAL_UTILS_H
#define OFXMODBUSORIENTAL_UTILS_H

#include <cstdint>
#include <vector>

class Ticker
{
public:
    
    Ticker(float interval) : curr(0.f), prev(0.f), interval(interval) { }
    
    bool tick()
    {
        curr = ofGetElapsedTimef();
        if (curr - prev < interval) return false;
        while (curr - prev >= interval) prev += interval;
        return true;
    }
    
    void reset() { curr = prev = ofGetElapsedTimef(); }
    
    float now() { return curr - prev;}
    
    void setInterval(float interval) { this->interval = interval; }
    
private:
    
    float curr;
    float prev;
    float interval;
    
};

class CrcGenerator
{
    vector<uint8_t> elements;
    
public:
    
    void push(uint8_t v) { elements.push_back(v); }
    
    void clear() { elements.clear(); }
    
    uint16_t get()
    {
        uint16_t result = 0xFFFF;
        for (auto& e : elements)
        {
            result ^= e;
            for (size_t j = 0; j < 8; ++j)
            {
                if (result & 0x01) result = (result >> 1) ^ 0xA001;
                else result >>= 1;
            }
        }
        return result;
    }
    
    uint16_t get(uint8_t* data, size_t size)
    {
        uint16_t result = 0xFFFF;
        for (size_t i = 0; i < size; ++i)
        {
            result ^= data[i];
            for (size_t j = 0; j < 8; ++j)
            {
                if (result & 0x01) result = (result >> 1) ^ 0xA001;
                else result >>= 1;
            }
        }
        return result;
    }
};


#endif /* OFXMODBUSORIENTAL_UTILS_H */
