#ifndef OFXMODBUSORIENTAL_CONTROLLER_H
#define OFXMODBUSORIENTAL_CONTROLLER_H

#include "detail/Stream.h"
#include "detail/Buffer.h"


OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_BEGIN

template <size_t Size>
class Controller
{
	struct Status
	{
        bool tlc {true};
        bool move {true};
        bool busy {true};
        bool alarm {true};
        bool ready {false};
	};

public:

    bool begin(
        size_t id,
        size_t baud,
	    float interval,
        data_bits d = OFXSERIAL_DATABIT_8,
        parity p = OFXSERIAL_PARITY_EVEN,
        stop_bits s = OFXSERIAL_STOPBIT_2
    ){
		serial.begin(id, baud, d, p, s);
        serial.setInterval(interval); // sometimes drops in 0.05 sec
    }

    bool begin(
		string name,
        size_t baud,
	    float interval,
        data_bits d = OFXSERIAL_DATABIT_8,
        parity p = OFXSERIAL_PARITY_EVEN,
        stop_bits s = OFXSERIAL_STOPBIT_2
    ){
		serial.begin(name, baud, d, p, s);
        serial.setInterval(interval); // sometimes drops in 0.05 sec
    }
	
    void update()
    {
		serial.update();

		while (serial.available())
		{
			auto req = serial.getResponse();
            switch(req->getKey())
            {
                case RequestType::Status:
                {
					uint32_t data = req->getResponse();
    				status[req->getID()].tlc = ((data >> 8) & 0x80);
    				status[req->getID()].move = ((data >> 8) & 0x20);
    				status[req->getID()].busy = ((data >> 8) & 0x01);
    				status[req->getID()].alarm = ((data >> 0) & 0x80);
    				status[req->getID()].ready = ((data >> 0) & 0x20);
//                    cout << "read status : " << hex << data << dec << endl;
    				break;
    			}
    			case RequestType::Position:
    			{
    				int32_t p = (int32_t)req->getResponse();
                    read_pos[req->getID()] = p;
                    wrote_pos[req->getID()] = p;
//					cout << "read pos : " << (int)req->getID() << ", " << (int)p << endl;
                    break;
                }
                default:
                {
                    // TODO: not broadcast motion response
                    cout << "response [invalid]" << endl;
                    break;
                }
            }
			serial.archiveResponse();
		}
    }
    
    void draw(float x, float y)
    {
        ofPushStyle();
        ofSetColor(255);
        ofDrawBitmapString("r", x, y + 8);
        ofDrawBitmapString("m_id", x + 40, y + 8);
        for (size_t i = 1; i <= getNumMotors(); ++i)
        {
            ofColor c = (ready(i)) ? ofColor::green : ofColor::red ;
            ofSetColor(c);
            ofDrawRectangle(x, y + 20 * i, 10, 10);
            ofDrawBitmapString(ofToString(i), x + 40, y + 8 + 20 * i);
        }
        ofPopStyle();
    }

	void request(RequestType r, uint8_t id)
    {
        if (id == 0) ofLogError("id == 0 is broadcast addr, invalid request");
        else serial.request(r, id);
    }

	void stop(uint8_t id)
	{
		std::shared_ptr<RemoteIOs> ios = std::make_shared<RemoteIOs>(CmdType::Stop, id);
		serial.push_front(std::static_pointer_cast<Query>(ios));
	}
    
//	void home(uint8_t id)
//	{
//		directDrive(id, offsets[id], 10000, 3000, 3000);
//	}
    
	void free(uint8_t id)
	{
		std::shared_ptr<RemoteIOs> ios = std::make_shared<RemoteIOs>(CmdType::Free, id);
		serial.push_back(std::static_pointer_cast<Query>(ios));
	}
    
    void reset(uint8_t id)
	{
		std::shared_ptr<RemoteIOs> ios = std::make_shared<RemoteIOs>(CmdType::Reset, id);
		serial.push_back(std::static_pointer_cast<Query>(ios));
	}

    void data_no(uint8_t no, uint8_t id)
	{
		std::shared_ptr<NetSelect> sel = std::make_shared<NetSelect>(no, id);
		serial.push_back(std::static_pointer_cast<Query>(sel));
	}

    void start(uint8_t id)
	{
		std::shared_ptr<RemoteIOs> ios = std::make_shared<RemoteIOs>(CmdType::Start, id);
		serial.push_back(std::static_pointer_cast<Query>(ios));
	}

    void clear(uint8_t id)
	{
		std::shared_ptr<RemoteIOs> ios = std::make_shared<RemoteIOs>(CmdType::Clear, id);
		serial.push_back(std::static_pointer_cast<Query>(ios));
	}

    void forward(uint8_t id)
    {
		std::shared_ptr<RemoteIOs> ios = std::make_shared<RemoteIOs>(CmdType::JogFwd, id);
		serial.push_back(std::static_pointer_cast<Query>(ios));
    }

    void backward(uint8_t id)
    {
		std::shared_ptr<RemoteIOs> ios = std::make_shared<RemoteIOs>(CmdType::JogBwd, id);
		serial.push_back(std::static_pointer_cast<Query>(ios));
    }

    void direct
    (
        uint8_t id, uint32_t abs_pos, uint32_t vel, uint32_t acc, uint32_t dec,
        uint8_t mode = 0x01, uint16_t crnt = 0x03E8, char trig = 1, uint8_t data_no = 0xFF
    ){
		std::shared_ptr<DirectDrive> drive = std::make_shared<DirectDrive>(id);
        drive->setDriveNo(data_no);
        drive->setDriveMode(mode);
        drive->setPosition(abs_pos);
        drive->setVelocity(vel);
        drive->setAcceleration(acc);
        drive->setDeceleration(dec);
        drive->setCurrent(crnt);
        drive->setTrigger(trig);
		serial.push_back(std::static_pointer_cast<Query>(drive));
    }

    void setJogSteps(uint8_t id, uint32_t steps)
    {
		std::shared_ptr<JogSteps> step = std::make_shared<JogSteps>(steps, id);
		serial.push_back(std::static_pointer_cast<Query>(step));
    }


    void setPosition(uint8_t id, int32_t pos)
    {
        if (id == 0) for (size_t i = 1; i <= getNumMotors(); ++i)
            buffer.setPosition(i, pos);
        else
            buffer.setPosition(id, pos);
    }
    void setVelocity(uint8_t id, int32_t vel)
    {
        if (id == 0) for (size_t i = 1; i <= getNumMotors(); ++i)
            buffer.setVelocity(i, vel);
        else
            buffer.setVelocity(id, vel);
    }
    void setMode(uint8_t id, uint8_t mode)
    {
        if (id == 0) for (size_t i = 1; i <= getNumMotors(); ++i)
            buffer.setMode(i, mode);
        else
            buffer.setMode(id, mode);
    }
    void setAcceleration(uint8_t id, uint32_t acc)
    {
        if (id == 0) for (size_t i = 1; i <= getNumMotors(); ++i)
            buffer.setAcceleration(i, acc);
        else
            buffer.setAcceleration(id, acc);
    }
    void setDeceleration(uint8_t id, uint32_t dec)
    {
        if (id == 0) for (size_t i = 1; i <= getNumMotors(); ++i)
            buffer.setDeceleration(i, dec);
        else
            buffer.setDeceleration(id, dec);
    }
    void setCurrent(uint8_t id, uint32_t crnt)
    {
        if (id == 0) for (size_t i = 1; i <= getNumMotors(); ++i)
            buffer.setCurrent(i, crnt);
        else
            buffer.setCurrent(id, crnt);
    }
    
    void write(uint8_t id)
    {
        writePosition(id);
        writeVelocity(id);
        writeAcceleration(id);
        writeDeceleration(id);
    }

	void writePosition(uint8_t id)
	{
        for (size_t i = 0; i < wrote_pos.size(); ++i)
            wrote_pos[i] = buffer.getPosition(i);
        
		Buffer::DataRef query = buffer.getPositionRef();
        query->setID(id);
		serial.push_back(query);
	}

	void writeVelocity(uint8_t id)
	{
		Buffer::DataRef query = buffer.getVelocityRef();
        query->setID(id);
		serial.push_back(query);
	}

	void writeMode(uint8_t id)
	{
		Buffer::DataRef query = buffer.getModeRef();
        query->setID(id);
		serial.push_back(query);
	}

	void writeAcceleration(uint8_t id)
	{
		Buffer::DataRef query = buffer.getAccelerationRef();
        query->setID(id);
		serial.push_back(query);
	}

	void writeDeceleration(uint8_t id)
	{
		Buffer::DataRef query = buffer.getDecelerationRef();
        query->setID(id);
		serial.push_back(query);
	}

	void writeCurrent(uint8_t id)
	{
		Buffer::DataRef query = buffer.getCurrentRef();
        query->setID(id);
		serial.push_back(query);
	}

	
    void setInterval(float sec) { serial.setInterval(sec); }
    
    void setVelocityLimit(int32_t v) { max_vel = v; }
	
	void setMotionTriangle(uint8_t id, int32_t pos, float time)
	{
        if (id == 0) for (size_t i = 1; i <= getNumMotors(); ++i)
            setMotionTriangleImpl(i, pos, time);
        else
            setMotionTriangleImpl(id, pos, time);
	}
    
	void setMotionTrapezoid(uint8_t id, int32_t target_pos, uint32_t target_acc, float time)
	{
        if (id == 0) for (size_t i = 1; i <= getNumMotors(); ++i)
            setMotionTrapezoidImpl(i, target_pos, target_acc, time);
        else
            setMotionTrapezoidImpl(id, target_pos, target_acc, time);
	}
    
    bool isOpen() { return serial.isInitialized(); }
    
    bool empty() { return (query_size() || request_size()) ? false : true; }

    bool ready(uint8_t id = 0)
    {
        bool b_ready = true;
        if (id == 0)
        {
            for (size_t i = 1; i <= getNumMotors(); ++i)
            {
                b_ready &= !status[i].tlc;
                b_ready &= !status[i].move;
                b_ready &= !status[i].busy;
                b_ready &= !status[i].alarm;
                b_ready &= status[i].ready;
            }
        }
        else
        {
            b_ready &= !status[id].tlc;
            b_ready &= !status[id].move;
            b_ready &= !status[id].busy;
            b_ready &= !status[id].alarm;
            b_ready &= status[id].ready;
        }
        return b_ready;
    }
    
    bool isTrqLimit(uint8_t id) { return status[id].tlc; }
    bool isMoving(uint8_t id) { return status[id].move; }
    bool isBusy(uint8_t id) { return status[id].busy; }
    bool hasAlarm(uint8_t id) { return status[id].alarm; }
    bool isReady(uint8_t id) { return status[id].ready; }
    
    size_t query_size() { return serial.query_size(); }
    size_t request_size() { return serial.request_size(); }

	size_t getNumMotors() { return Size; }
    Status getStatus(uint8_t id) { return status[id]; }
	int32_t getPosition(uint8_t id) { return read_pos[id]; }
	int32_t getPositionMax() { return pos_limit_max; }
	int32_t getPositionMin() { return pos_limit_min; }
	int32_t getVelocityMax() { return vel_limit_max; }
	int32_t getVelocityMin() { return vel_limit_min; }
	uint32_t getAccelerationMax() { return acc_limit; }
	uint32_t getCurrentMax() { return crnt_limit; }
    
	int32_t getPositionBuffer(uint8_t id) { return wrote_pos[id]; }
	
private:

    void setMotionTriangleImpl(uint8_t id, int32_t pos, float time)
    {
        float diff_pos = (float)((float)pos - (float)wrote_pos[id]);
        float avg_vel = diff_pos / time;
        float vel = 0.f;
        float acc = 0.f;
		assert(abs(avg_vel) < max_vel);
		
        if (std::abs(avg_vel) <= 500.f)
        {
            vel = avg_vel;
            acc = acc_limit;
            ofLogWarning("low speed : constant speed operation");
        }
        else
        {
            vel = vel_limit_max; // 2.f * avg_vel
            acc = 4.f * avg_vel / time;
        }
        buffer.setAcceleration(id, (uint32_t)std::abs(acc));
        buffer.setDeceleration(id, (uint32_t)std::abs(acc));
        buffer.setVelocity(id, (int32_t)vel); // if needed
        buffer.setPosition(id, pos);
    }
	
	void setMotionTrapezoidImpl(uint8_t id, int32_t target_pos, uint32_t target_acc, float time)
	{
//        float diff_pos = (float)(target_pos - wrote_pos[id]);
//		float avg_vel = diff_pos / time;
//		float vel = 0.f;
//		float acc = 0.f;
//        
//        cout << dec;
//        cout << "prev pos = " << wrote_pos[id] << endl;
//        cout << "next pos = " << target_pos << endl;
//        cout << "time     = " << time << endl;
//        cout << "avg  vel = " << avg_vel << endl;
//        
//        // TODO: if acc is too fast, upper line is vanished....., so limit acc
//        // TODO: if target_acc ( time < 4.f * avg_vel, vel = nan......
//        
//        if (std::abs(avg_vel) <= 500.f)
//        {
//            cout << "constant vel drive" << dec << endl;
//			vel = avg_vel;
//		}
//		else
//		{
//            cout << "trapezoid vel drive" << dec << endl;
//			vel = (target_acc * time + sqrt(target_acc * time * (target_acc * time - 4.f * avg_vel))) / 2.f;
////			float time_const_vel = time - 2.f * vel / target_acc;
//		}
//        buffer.setAcceleration(id, target_acc); // if needed
//        buffer.setDeceleration(id, target_acc); // if needed
//		buffer.setVelocity(id, (int32_t)vel);
//		buffer.setPosition(id, target_pos);
//        cout << "pos = " << target_pos << endl;
//        cout << "vel = " << vel << endl;
//        cout << "acc = " << target_acc << endl;
	}
	
    ofxOriental::Stream serial;
    ofxOriental::Buffer buffer;

    std::array<int32_t, Size + 1> read_pos;
    std::array<int32_t, Size + 1> wrote_pos;
	std::array<Status, Size + 1> status;
	
	const int32_t pos_limit_max = std::numeric_limits<int32_t>::max(); // -2,147,483,648 - 2,147,483,647 step
	const int32_t pos_limit_min = std::numeric_limits<int32_t>::min(); // -2,147,483,648 - 2,147,483,647 step
	const int32_t vel_limit_max { 4000000}; // -4,000,000 - 4,000,000 Hz
	const int32_t vel_limit_min {-4000000}; // -4,000,000 - 4,000,000 Hz
	const uint32_t acc_limit {1000000000};  // 1 - 1,000,000,000 (1=0.001kHz/s, 1=0.001s, or 1=0.001ms/kHz)
	const uint32_t crnt_limit {0x3E8};  // 0 - 1000 (0.1% per 1)
	const int32_t max_vel {20000};
};

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_END

#endif /* OFXMODBUSORIENTAL_CONTROLLER_H */
