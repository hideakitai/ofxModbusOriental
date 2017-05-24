#ifndef OFXMODBUSORIENTAL_STREAM_H
#define OFXMODBUSORIENTAL_STREAM_H

#include <memory>
#include <deque>
#include "ofxSerial.h"
#include "Utils.h"
#include "Query.h"
#include "Request.h"
#include "Parser.h"

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_BEGIN

class Stream : public ofxSerial
{
public:
	
	bool begin(
        size_t id,
        size_t baud,
        data_bits d = OFXSERIAL_DATABIT_8,
        parity p = OFXSERIAL_PARITY_EVEN,
        stop_bits s = OFXSERIAL_STOPBIT_2
	){
		ofxSerial::setup(id, baud);
		ofxSerial::setDataBits(d);
		ofxSerial::setParity(p);
		ofxSerial::setStopBits(s);
        ticker.reset();
	}
	
	bool begin(
        string name,
        size_t baud,
        data_bits d = OFXSERIAL_DATABIT_8,
        parity p = OFXSERIAL_PARITY_EVEN,
        stop_bits s = OFXSERIAL_STOPBIT_2
	){
		ofxSerial::setup(name.c_str(), baud);
		ofxSerial::setDataBits(d);
		ofxSerial::setParity(p);
		ofxSerial::setStopBits(s);
        ticker.reset();
	}
	
	void update()
	{
		if (!ofxSerial::isInitialized()) return;
		
        if (ticker.tick())
        {
            if (requests.size())
            {
                auto req = requests.front();
                if (!req->isRequested())
                {
                    write(req->data(), req->size());
                    req->requested();
                }
				else if (req->timeout())
                {
                    ofLogError("Response Timeout!!") << req->getID();
                    requests.pop_front();
                }
            }
            else if (queries.size())
            {
                write(queries.front()->data(), queries.front()->size());
                queries.pop_front();
            }
        }
		
		while (ofxSerial::available()) parser.feed(ofxSerial::readByte());
		while (parser.available()) handleInput(parser.front());
	}

	void request(RequestType r, uint8_t id)
	{
		parser.clear();
		if (ofxSerial::isInitialized())
		{
			std::shared_ptr<Request> req = std::make_shared<Request>(id, r);
			requests.push_back(req);
		}
	}

	void push_back(std::shared_ptr<Query> q) { if(ofxSerial::isInitialized()) queries.push_back(q); }
    
	void push_front(std::shared_ptr<Query> q) { if(ofxSerial::isInitialized()) queries.push_front(q); }
    
	void pop() { queries.pop_front(); }
	
	void archiveResponse() { requests.pop_front(); }
	
	void setInterval(float sec) { ticker.setInterval(sec); }
	
    bool available() { return (requests.size()) ? requests.front()->isReceived() : false; }
    size_t query_size() { return queries.size(); }
    size_t request_size() { return requests.size(); }
    
	std::shared_ptr<Request> getResponse() { return requests.front(); }
	
    
private:
	
    void write(uint8_t* data, size_t size) { ofxSerial::writeBytes(data, size); }
    
	void handleInput(const Parser::Response& res)
	{
        if (!requests.size())
        {
            parser.pop();
            return;
        }
        
		auto& req = requests.front();
		uint32_t p = 0;
		p |= (res.data[0] << 24) & 0xFF000000;
		p |= (res.data[1] << 16) & 0x00FF0000;
		p |= (res.data[2] <<  8) & 0x0000FF00;
		p |= (res.data[3] <<  0) & 0x000000FF;
		req->setResponse(p);
		parser.pop();
	}
	
    Parser parser;
	Ticker ticker {0.1};
	
	std::deque<std::shared_ptr<Query>> queries;
	std::deque<std::shared_ptr<Request>> requests;
	
	size_t timeout_tick {3};
};

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_END

#endif /* OFXMODBUSORIENTAL_STREAM_H */
