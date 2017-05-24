#ifndef OFXMODBUSORIENTAL_REQUEST_H
#define OFXMODBUSORIENTAL_REQUEST_H

#include <cstdint>
#include <unordered_map>

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_BEGIN

enum class RequestType { Status, Position };

class Request : public QueryImpl<8>
{
	RequestType key;
	uint32_t response {0};
	bool b_requested {false};
	bool b_received {false};
	size_t tick_count {0};
	size_t tick_timeout {3};
	
    unordered_map<RequestType, uint16_t, EnumClassHash> request_reg_map
    {
        {RequestType::Status, 0x007E},
        {RequestType::Position, 0x0120}
    };
	
public:
    
    Request(uint8_t id, RequestType req)
    {
        setID(id);
        setFunc(0x03);
        setAddr(request_reg_map[req]);
        setRegSize(2);
		key = req;
    }
	
	bool isRequested() { return b_requested; }
	bool isReceived() { return b_received; }
	
	uint8_t getID() { return query[0]; }
	RequestType getKey() { return key; }
	uint32_t getResponse() { return response; }
	
    void setResponse(uint32_t r) { response = r; b_received = true; }
	void requested() { b_requested = true; }
	
	bool timeout() { return (++tick_count >= tick_timeout) ? true : false; }
	void setTimeout(size_t count) { tick_timeout = count; }
	
};

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_END

#endif /* OFXMODBUSORIENTAL_REQUEST_H */
