#include "ofApp.h"

const size_t num_motors = 1;
const size_t modbus_baud = 230400;
float modbus_interval = 0.05;
const uint8_t modbus_init_mode = 1;
const uint32_t modbus_init_acc = 5000;
const int32_t modbus_init_vel = 5000;
const int32_t modbus_init_crnt = 1000;
const uint32_t modbus_init_jog_steps = 300;


ofxOriental::Controller<num_motors> modbus;

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetFrameRate(60);
    ofSetBackgroundColor(0);
    
    cout << "begin modbus communication" << endl;
    modbus.begin(0, modbus_baud, modbus_interval);
    
    cout << "read current motor position" << endl;
    for (size_t i = 1; i <= modbus.getNumMotors(); ++i)
        modbus.request(ofxOriental::RequestType::Position, i);
    
    cout << "write all queries & request, and wait for the reply..." << endl;
    while (!modbus.empty()) modbus.update();
    
    cout << "set initial settings to buffers" << endl;
    // 0: broadcast, 1-N: motor id
    for (size_t i = 1; i <= modbus.getNumMotors(); ++i)
    {
        modbus.data_no(i, i);
        modbus.setPosition(i, modbus.getPosition(i));
    }
    modbus.setMode(0, modbus_init_mode);
    modbus.setAcceleration(0, modbus_init_acc);
    modbus.setDeceleration(0, modbus_init_acc);
    modbus.setCurrent(0, modbus_init_crnt);
    modbus.setVelocity(0, modbus.getVelocityMax());
    modbus.setJogSteps(0, modbus_init_jog_steps);
    
    cout << "write initial settings to motors" << endl;
    modbus.writeMode(0);
    modbus.writeCurrent(0);
    modbus.write(0);
    
    cout << "send status request" << endl;
    for (size_t i = 1; i <= modbus.getNumMotors(); ++i)
        modbus.request(ofxOriental::RequestType::Status, i);
    
    cout << "write all queries & request, and wait for the reply..." << endl;
    while (!modbus.empty()) modbus.update();
    
    cout << "check if motor status is ready " << endl;
    if (!modbus.ready()) ofLogError("motor is NOT ready");
}

//--------------------------------------------------------------
void ofApp::update(){
    modbus.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    stringstream ss;
    
    ss << "[test commands]" << endl;
    ss << "space    : stop motors" << endl;
    ss << "f        : free motors" << endl;
    ss << "r        : reset motors" << endl;
    ss << "d        : direct motion (fwd)" << endl;
    ss << "D        : direct motion (bwd)" << endl;
    ss << "j        : jog (fwd)" << endl;
    ss << "J        : jog (bwd)" << endl;
    ss << "KEY_UP   : set jog steps bigger" << endl;
    ss << "KEY_DOWN : set jog steps smaller" << endl;
    ss << "e        : go to position using buffers (fwd)" << endl;
    ss << "E        : go to position using buffers (bwd)" << endl;
    ss << "s        : read status" << endl;
    ss << "p        : read position" << endl;
    ss << endl;
    for (size_t i = 1; i <= modbus.getNumMotors(); ++i)
    {
        ss << "[motor status " << i << "]" << endl;
        ss << boolalpha;
        ss << "is torque limit : " << modbus.isTrqLimit(i) << endl;
        ss << "is moving       : " << modbus.isMoving(i) << endl;
        ss << "is busy         : " << modbus.isBusy(i) << endl;
        ss << "has alarm       : " << modbus.hasAlarm(i) << endl;
        ss << "is ready        : " << modbus.isReady(i) << endl;
        ss << endl;
    }
    ss << endl;
    ss << "[actual read position]" << endl;
    for (size_t i = 1; i <= modbus.getNumMotors(); ++i)
        ss << "position " << i << " : " << modbus.getPosition(i) << endl;
    
    ofSetColor(255);
    ofDrawBitmapString(ss.str(), 20, 20);
    modbus.draw(20, 500);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    
    switch (key) {
            
        case ' ':
        {
            // stop motors right now
            modbus.stop(0);
            break;
        }
        case 'f':
        {
            // free motors
            modbus.free(0);
            break;
        }
        case 'r':
        {
            // reset motors
            modbus.reset(0);
            break;
        }
        case 'd':
        {
            // direct motion
            int32_t next_pos = 5000;
            modbus.direct(0, next_pos, modbus_init_vel, modbus_init_acc, modbus_init_acc);
            break;
        }
        case 'D':
        {
            // direct motion
            int32_t next_pos = 0;
            modbus.direct(0, next_pos, modbus_init_vel, modbus_init_acc, modbus_init_acc);
            break;
        }
        case 'j':
        {
            // jog forward
            modbus.forward(0);
            break;
        }
        case 'J':
        {
            // jog backward
            modbus.backward(0);
            break;
        }
        case OF_KEY_UP:
        {
            modbus.setJogSteps(0, 1000);
            break;
        }
        case OF_KEY_DOWN:
        {
            modbus.setJogSteps(0, 200);
            break;
        }
        case 'e':
        {
            // by using pos/vel/acc/dec buffers,
            // you can set different parameters to multiple motors
            // and run them concurrently or any timing you need
            // first, set pos/vel/acc/dec to buffers,
            // second, write those buffered data to motor driver
            // finally, trigger the start flag
            // after that, you need to clear start flag for next motion
            
            int32_t next_pos = 5000;
            float duration = 5.f;
            // set next position and duration
            // this automatically set pos, vel, acc, dec to the buffer
            modbus.setMotionTriangle(0, next_pos, duration);
            // or set them to buffer manually
    //        modbus.setPosition(0, next_pos);
    //        modbus.setVelocity(0, modbus_init_vel);
    //        modbus.setAcceleration(0, modbus_init_acc);
    //        modbus.setDeceleration(0, modbus_init_acc);
            
            // write buffered values to buffer
            // position, velocity, acceleration, Deceleration
            modbus.write(0);
            
            // or write them manually
    //        modbus.writePosition(0);
    //        modbus.writeVelocity(0);
    //        modbus.writeAcceleration(0);
    //        modbus.writeDeceleration(0);
            
            // start motion depending on the wrote values (pos, vel, acc, dec)
            modbus.start(0);
            // clear start flag (you should do this before next start())
            modbus.clear(0);
            
            break;
        }
        case 'E':
        {
            int32_t next_pos = 0;
            float duration = 5.f;
            modbus.setMotionTriangle(0, next_pos, duration);
            modbus.write(0);
            modbus.start(0);
            modbus.clear(0);
            break;
        }
        case 's':
        {
            for (size_t i = 1; i <= modbus.getNumMotors(); ++i)
                modbus.request(ofxOriental::RequestType::Status, i);
            break;
        }
        case 'p':
        {
            for (size_t i = 1; i <= modbus.getNumMotors(); ++i)
                modbus.request(ofxOriental::RequestType::Position, i);
            break;
        }
        
        default:
        {
            break;
        }
    }
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
