# ofxModbusOriental
modbus protocol wrapper for oriental motor



## Dependencies

- [ofxSerial](https://github.com/hideakitai/ofxSerial) (for further setting of ofSerial)
- USB to RS485 converter. Please check the pin assign for yourself.




## Limitations

currently motor ids  should be set as starts from 1 and line up in order to Nth motor.

for example, the ids of N motors are : 1, 2, …, N-1, N.



## Usage



### Modbus Setting

``` c++
#include "ofxModbusOriental.h"

ofxOriental::Controller<num_motors> modbus;

void setup()
{
    // start modbus communication (via usb serial)
    modbus.begin(serial_name_or_id, modbus_baud, modbus_interval);
}

void update()
{
    // send, receive and parse data
    modbus.update();
}
```



### Motor IDs

If you pass the motor id = 0, it means broadcast and all motor receive the same command and does not reply.



### Basic Motion Functions

#### Motion

``` c++
modbus.stop(id);
modbus.free(id);
modbus.reset(id);
modbus.direct(id, pos, vel, acc, dec);
modbus.forward(id);
modbus.backward(id);
modbus.setJogSteps(id, steps);
```



#### Request

```c++
modbus.request(ofxOriental::RequestType::Status, id);
modbus.request(ofxOriental::RequestType::Position, id);
```



### Control Motion with Drive Data Number

with this feature, you can control motors flexibly like :

- control multiple motors with different motion rapidly and concurently
- buffer next positions and other parameters in advance, and execute them by just trigger ```start()```

``` c++
// first, pair motor id and drive data number
modbus.data_no(id, drive_data_no);

// set parameters to buffer
modbus.setMode(id, drive_mode);
modbus.setCurrent(id, crnt);
modbus.setPosition(id, pos);
modbus.setVelocity(id, vel);
modbus.setAcceleration(id, acc);
modbus.setDeceleration(id, dec);

// write buffered data to driver
modbus.writeMode(id);
modbus.writeCurrent(id);
modbus.writePosition(id);
modbus.writeVelocity(id);
modbus.writeAcceleration(id);
modbus.writeDeceleration(id);

// write trigger to start motion
modbus.start(id);

// clear trigger for next motion
modbus.clear(id);
```



or for instance, you can use wrapped methods like

```c++
// first, pair motor id and drive data number
modbus.data_no(id, drive_data_no);

// set mode and current
modbus.setMode(id, drive_mode);
modbus.setCurrent(id, crnt);

// moves to next_pos in duration
// this method sets position, velocity, acceleration and deceleration
modbus.setMotionTriangle(id, next_pos, duration);

// write position, velocity, acceleration and deceleration
modbus.write(id);

// write trigger to start motion
modbus.start(id);

// clear trigger for next motion
modbus.clear(id);
```



for more detail, check the example and source codes.



## LICENSE

MIT