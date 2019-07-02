// -----------------------------------------------------------------------------
// OOKDecoderSensor
// Copyright (C) 2017-2018 by Oscarsan
// -----------------------------------------------------------------------------

#if SENSOR_SUPPORT && OOK_DECODER_SUPPORT 

#pragma once

#include "Arduino.h"
#include "BaseSensor.h"
#include <vector>

#define OOK_SENSOR_THN132N               0xEA4C      // Temp
#define OOK_SENSOR_THGR228N              0x1A2D      // Temp. HR
#define OOK_SENSOR_RTGR328N              0xACC       // 0x*ACC Temp. HR
#define OOK_SENSOR_THGR328N              0xCA2C      // Temp. HR
#define OOK_SENSOR_THGR810               0xFA28      // Temp. HR
#define OOK_SENSOR_WTGR800               0xFAB8      // Temp. HR


class DecodeOOK {
protected:
    byte total_bits, bits, flip, state, pos, data[25];

    virtual byte decode (uint32_t width) =0;
    virtual void done_decoder(void) = 0;
public:

    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

    DecodeOOK () { resetDecoder(); }

    bool nextPulse (uint32_t width) {
        if (state != DONE)
            switch (decode(width)) {
                case 0: resetDecoder(); break;
                case 1:  done(); break;
                case 2:  break;
            }
        return isDone();
    }

    bool isDone () const { return state == DONE; }

    const byte* getData (byte& count) const {
        count = pos;
        return data;
    }

    void resetDecoder () {
        total_bits = bits = pos = flip = 0;
        state = UNKNOWN;
    }

    // add one bit to the packet data buffer

    virtual void gotBit (byte value) {
        total_bits++;
        byte *ptr = data + pos;
        *ptr = (*ptr >> 1) | (value << 7);

        if (++bits >= 8) {
            bits = 0;
            if (++pos >= sizeof data) {
                resetDecoder();
                return;
            }
        }
        state = OK;
    }

    // store a bit using Manchester encoding
    void manchester (byte value) {
        flip ^= value; // manchester code, long pulse flips the bit
        gotBit(flip);
    }

    // move bits to the front so that all the bits are aligned to the end
    void alignTail (byte maxim =0) {
        // align bits
        if (bits != 0) {
            data[pos] >>= 8 - bits;
            for (byte i = 0; i < pos; ++i)
                data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
            bits = 0;
        }
        // optionally shift bytes down if there are too many of 'em
        if (maxim > 0 && pos > maxim) {
            byte n = pos - maxim;
            pos = maxim;
            for (byte i = 0; i < pos; ++i)
                data[i] = data[i+n];
        }
    }

    void reverseBits () {
        for (byte i = 0; i < pos; ++i) {
            byte b = data[i];
            for (byte j = 0; j < 8; ++j) {
                data[i] = (data[i] << 1) | (b & 1);
                b >>= 1;
            }
        }
    }

    void reverseNibbles () {
        for (byte i = 0; i < pos; ++i)
            data[i] = (data[i] << 4) | (data[i] >> 4);
    }

    void done () {
        while (bits)
            DecodeOOK::gotBit(0); // padding
        done_decoder();
        //state = DONE;
    }
};

class OregonDecoderV2 : public DecodeOOK {
protected:
    byte _previous_data[25];
public:
    OregonDecoderV2() {}

    // add one bit to the packet data buffer
   virtual void gotBit (byte value) {
        if(!(total_bits & 0x01)){
            data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
        }
        total_bits++;
        pos = total_bits >> 4;
        if (pos >= sizeof data) {
            resetDecoder();
            return;
        }
        state = OK;
    }

    virtual byte decode (uint32_t width) {
        if ((200 <= width) && (width < 1200)) {
            byte w = width >= 700;
            switch (state) {
                case UNKNOWN:
                    if (w != 0) ++flip; // Long pulse, count 24
                    else if ((w == 0) && (24 <= flip)) { // First Short pulse, start bit
                        flip = 0;
                        state = T0;
                    } 
                    else return 0;                       // Reset decoder
                    break;
                case OK:
                    if (w == 0) state = T0;   // Short pulse
                    else manchester(1); // Long pulse
                    break;
                case T0:
                    if (w == 0) manchester(0); // Second short pulse, start bit
                    else return 0;                         // Reset decoder
                    break;
            }
        } 
        else if (width >= 2500  && pos >= 8) {
                return 1;
        } else {
            return 0;
        }
        return total_bits == 160 ? 1: 2;
    }

    int temperature() {
      //int sign = (data[6]&0x8) ? -1 : 1;
      //float temp = ((data[5]&0xF0) >> 4)*10 + (data[5]&0xF) + (float)(((data[4]&0xF0) >> 4) / 10.0);
      //return sign * temp;
      int celsius = ((_previous_data[5] >> 4) * 100)  + ((_previous_data[5] & 0x0F) * 10) + ((_previous_data[4] >> 4));
      if ((_previous_data[6] & 0x0F) >= 8) celsius = -celsius;
      return celsius;
    }
    uint8_t humidity() {
      if (_previous_data[0] == 0xEA && _previous_data[1] == 0x4C)    //  THN132N,... no humidity sensor
        return 0;
      else 
        return (_previous_data[7] & 0xF) * 10 + ((_previous_data[6] & 0xF0) >> 4);; 
    }

    
    uint8_t os_channel() {
      uint8_t os_channel;
      switch (_previous_data[2]) {
        case 0x10: os_channel = 1; break;
        case 0x20: os_channel = 2; break;
        case 0x40: os_channel = 3; break;
      }
      return os_channel;
    }
    
   
    byte battery() {
      return (_previous_data[4] & 0x4) ? 1 : 9;
    }
    
    uint32_t model(){ //Model
      return ((_previous_data[0]&0xff)<<8) + _previous_data[1];
    }

    uint8_t id(){   //ID. This value change every battery replacement of the sensor
      return _previous_data[3];
    }

    int Sum(byte count) {
      int s = 0;
      for (byte i = 0; i < count; i++)  {
        s += (data[i] & 0xF0) >> 4;
        s += (data[i] & 0xF);
      }
      if (int(count) != count) s += (data[count] & 0xF0) >> 4;
      return s;
    }
    
    void done_decoder(void){
        if (data[0] == 0xEA && data[1] == 0x4C) {   //  THN132N,...
          int c = ((data[6] & 0xF0) >> 4) + ((data[7] & 0xF) << 4);
          int s = ((Sum(6) + (data[6] & 0xF) - 0xa) & 0xff);
          if (s != c){                              //checksum error
            resetDecoder(); 
            return;
          }
        }
        if (data[0] == 0x1A && data[1] == 0x2D) {   // THGR228N,...
          if (data[8] != ((Sum(8) - 0xa) & 0xff)){     //cheksum error
            resetDecoder(); 
            return;
          }
        }
        if (memcmp(_previous_data, data, pos)!=0){   // non duplicated value
          memcpy(_previous_data, data, pos); 
          state = DONE;   
        }
        else                                         // duplicated value. Discarded
          resetDecoder();
    }
};

class OOKDecoderSensor : public BaseSensor, OregonDecoderV2 {

    public:

        // ---------------------------------------------------------------------
        // Public
        // ---------------------------------------------------------------------

       OOKDecoderSensor(): BaseSensor() {
            _count = 1;
            _sensor_id = SENSOR_EVENTS_ID;
        }

        ~OOKDecoderSensor() {
            _enableInterrupts(false);
        }

        // ---------------------------------------------------------------------

        void setGPIO(unsigned char gpio) {
            _gpio = gpio;
        }

        void setTrigger(bool trigger) {
            _trigger = trigger;
        }

        void setPinMode(unsigned char pin_mode) {
            _pin_mode = pin_mode;
        }

        void setInterruptMode(unsigned char interrupt_mode) {
            _interrupt_mode = interrupt_mode;
        }

        // ---------------------------------------------------------------------

        unsigned char getGPIO() {
            return _gpio;
        }

        bool getTrigger() {
            return _trigger;
        }

        unsigned char getPinMode() {
            return _pin_mode;
        }

        unsigned char getInterruptMode() {
            return _interrupt_mode;
        }


        // ---------------------------------------------------------------------
        // Sensors API
        // ---------------------------------------------------------------------

        // Initialization method, must be idempotent
        // Defined outside the class body
        void begin() {
            uint32_t addrs[] = OOK_DECODER_SENSORS_ADDR;
            if (!_dirty) {
                if (millis() - _autodiscover_start_time > (OOK_DECODER_AUTODISCOVER_WAIT_TIME*1000)) // finish discover time
                    _ready=true;
                else if (_count >= OOK_DECODER_SENSORS_MAX_NUMBER)
                    _ready=true;
                return;
            }
            for (int i=0; i < (sizeof(addrs) / sizeof(addrs[0])); i++){
                ook_device_magnitudes_t device_mag;
                if (has_temperature(addrs[i])){
                    device_mag.address=addrs[i];
                    device_mag.value=0.0;
                    device_mag.type=MAGNITUDE_TEMPERATURE;
                    _dev_mag.push_back(device_mag);
                }
                if (has_humidity(addrs[i])){
                    device_mag.address=addrs[i];
                    device_mag.value=0.0;
                    device_mag.type=MAGNITUDE_HUMIDITY;
                    _dev_mag.push_back(device_mag);
                }
            }

            _count = _dev_mag.size();
            _enableInterrupts(true);
            if (!OOK_DECODER_AUTODISCOVER_ADDR)
                _ready = true;
            else 
                _autodiscover_start_time=millis();
            _dirty = false;
        }

        // Loop-like method, call it in your main loop
        void tick() {

            if (isDone()){  // finished sensor message
/*              for (byte i = 0; i < pos; ++i) {
                    Serial.print(data[i] >> 4, HEX); Serial.print(data[i] & 0x0F, HEX);
                    DEBUG_MSG_P("%X", data[i]);
                }*/
                uint16_t new_model = model();
                bool found = false;
                if ( (new_model & 0x0fff) == OOK_SENSOR_RTGR328N) 
                   new_model = new_model & 0x0fff; 
                uint32_t new_address = new_model<<16 | id()<<8 | os_channel();
                DEBUG_MSG_P("[OKKSensor] Model:0x%X ID:0x%X Ch:%u Bat:%u T:%u HR:%u Address:0x%X\n", 
                    model(), id(), os_channel(),battery(),temperature(), humidity(), new_address);

                for (int index=0; index < _count; index++){
                    if (_dev_mag[index].address==new_address){ //device found
                        if  (has_temperature(new_address))
                            _dev_mag[index].value= (double) temperature()/10;
                        if  (has_humidity(new_address)){
                            index++;
                            _dev_mag[index].value= (double) humidity();
                        }
                        found=true;
                        break; // exit loop
                     }
                }

                if (!_ready){ // look for a new detected device              
                    if (!found && OOK_DECODER_AUTODISCOVER_ADDR && _count<OOK_DECODER_SENSORS_MAX_NUMBER){  // add new sensor
                        ook_device_magnitudes_t device_mag;
                        if  (has_temperature(new_address)){
                            device_mag.address=new_address;
                            device_mag.value=(double) temperature()/10;
                            device_mag.type=MAGNITUDE_TEMPERATURE;
                            _dev_mag.push_back(device_mag);
                            _count++;
                            DEBUG_MSG_P("[OKKSensor] New device Temperature: #:%d Addr:%X value:%s\n", 
                                _count, device_mag.address, String(device_mag.value).c_str());
                        }
                        if  (has_humidity(new_address)){
                            device_mag.address=new_address;
                            device_mag.value= (double) humidity();
                            device_mag.type=MAGNITUDE_HUMIDITY;
                            _dev_mag.push_back(device_mag);
                            _count++;
                            DEBUG_MSG_P("[OKKSensor] New device Humidity. #:%d Addr:%X value:%s\n", 
                                _count, device_mag.address, String(device_mag.value).c_str());
                        }
                    }
                }
                else if (!found){
                    DEBUG_MSG_P("[OKKSensor] Unregistered Sensor\n");
                }
                resetDecoder();
            }
        }

        // Descriptive name of the sensor
        String description() {
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "OKK_Sensor @ GPIO%d", _gpio);
            return String(buffer);
        }

        // Descriptive name of the slot # index

        String slot(unsigned char index) {
            if (index < _count) {
                char buffer[40];
                snprintf(buffer, sizeof(buffer), "%s (%X) @ GPIO%d",
                    chipAsString(index).c_str(),
                    _dev_mag[index].address,_gpio);
                return String(buffer);
            }
            return String();
        }

        // Address of the device
        String address(unsigned char index) {
            char buffer[20] = {0};
            if (index < _count) {
                snprintf(buffer, sizeof(buffer), "%X",_dev_mag[index].address);
            }
            return String(buffer);
        }


        // Type for slot # index
        unsigned char type(unsigned char index) {
            if (index < _count) {
                return _dev_mag[index].type;
            }
            return MAGNITUDE_NONE;
        }

        // Current value for slot # index
        double value(unsigned char index) {
            if (index < _count) {
                return _dev_mag[index].value;
            }
            return 0;
        }

        // Handle interrupt calls
        void handleInterrupt(unsigned char gpio) {
            (void) gpio;
            static unsigned long last = 0;
            // determine the pulse length in microseconds, for either polarity
            unsigned long pulse = micros() - last;
            last += pulse;
            nextPulse(pulse);
         /*    static unsigned long last = 0;
            if (millis() - last > _debounce) {
                last = millis();
                _events = _events + 1;
                if (_trigger) {
                    if (_callback) _callback(MAGNITUDE_EVENT, digitalRead(gpio));
                }
            }*/
        }

    protected:

        // ---------------------------------------------------------------------
        // Interrupt management
        // ---------------------------------------------------------------------

        void _attach(OOKDecoderSensor * instance, unsigned char gpio, unsigned char mode);
        void _detach(unsigned char gpio);

        void _enableInterrupts(bool value) {

            static unsigned char _interrupt_gpio = GPIO_NONE;

            if (value) {
                if (_interrupt_gpio != GPIO_NONE) _detach(_interrupt_gpio);
                _attach(this, _gpio, _interrupt_mode);
                _interrupt_gpio = _gpio;
            } else if (_interrupt_gpio != GPIO_NONE) {
                _detach(_interrupt_gpio);
                _interrupt_gpio = GPIO_NONE;
            }

        }

        // ---------------------------------------------------------------------
        // Protected
        // ---------------------------------------------------------------------
        String chipAsString(unsigned char index) {
            uint16_t model = (_dev_mag[index].address)>>16;
            if (model == OOK_SENSOR_THN132N) return String("THN132N");
            if (model == OOK_SENSOR_THGR228N) return String("THGR228N");
            if (model == OOK_SENSOR_THGR328N) return String("THGR328N"); 
            if (model == OOK_SENSOR_THGR328N) return String("THGR328N"); 
            if (model == OOK_SENSOR_WTGR800) return String("WTGR800");   
            else{
                if ((model & 0x0fff) == OOK_SENSOR_RTGR328N) return String("RTGR328N"); 
                else return String("Unkown");
            } 
        }
        
        bool has_temperature(uint32_t addr){
            switch (addr>>16){
            case OOK_SENSOR_THN132N:  
            case OOK_SENSOR_THGR228N:
            case OOK_SENSOR_RTGR328N:
            case OOK_SENSOR_THGR328N:
            case OOK_SENSOR_THGR810:
            case OOK_SENSOR_WTGR800:
                return true;
                break;
            default:
                return false;
                break;
            }
        }

        bool has_humidity(uint32_t addr){
            switch (addr>>16){
            case OOK_SENSOR_THN132N:  //  THN132N,... no humidity sensor
                return false;
                break;
            case OOK_SENSOR_THGR228N:
            case OOK_SENSOR_RTGR328N:
            case OOK_SENSOR_THGR328N:
            case OOK_SENSOR_THGR810:
            case OOK_SENSOR_WTGR800:
                return true;
                break;
            default:
                return false;
                break;
            }
        }

        typedef struct {
            uint32_t address; // 0xMMMMIICC M->model I->ID C->Channel
            double value = 0;
            unsigned char type;
        } ook_device_magnitudes_t;
        std::vector<ook_device_magnitudes_t> _dev_mag;

        unsigned char _gpio = GPIO_NONE;
        bool _trigger = false;
        unsigned char _pin_mode = INPUT;
        unsigned char _interrupt_mode = RISING;
        unsigned long _autodiscover_start_time;

};



// -----------------------------------------------------------------------------
// Interrupt helpers
// -----------------------------------------------------------------------------

OOKDecoderSensor * _event_sensor_instance[10] = {NULL};

void ICACHE_RAM_ATTR _event_sensor_isr(unsigned char gpio) {
    unsigned char index = gpio > 5 ? gpio-6 : gpio;
    if (_event_sensor_instance[index]) {
        _event_sensor_instance[index]->handleInterrupt(gpio);
    }
}

void ICACHE_RAM_ATTR _event_sensor_isr_0() { _event_sensor_isr(0); }
void ICACHE_RAM_ATTR _event_sensor_isr_1() { _event_sensor_isr(1); }
void ICACHE_RAM_ATTR _event_sensor_isr_2() { _event_sensor_isr(2); }
void ICACHE_RAM_ATTR _event_sensor_isr_3() { _event_sensor_isr(3); }
void ICACHE_RAM_ATTR _event_sensor_isr_4() { _event_sensor_isr(4); }
void ICACHE_RAM_ATTR _event_sensor_isr_5() { _event_sensor_isr(5); }
void ICACHE_RAM_ATTR _event_sensor_isr_12() { _event_sensor_isr(12); }
void ICACHE_RAM_ATTR _event_sensor_isr_13() { _event_sensor_isr(13); }
void ICACHE_RAM_ATTR _event_sensor_isr_14() { _event_sensor_isr(14); }
void ICACHE_RAM_ATTR _event_sensor_isr_15() { _event_sensor_isr(15); }

static void (*_event_sensor_isr_list[10])() = {
    _event_sensor_isr_0, _event_sensor_isr_1, _event_sensor_isr_2,
    _event_sensor_isr_3, _event_sensor_isr_4, _event_sensor_isr_5,
    _event_sensor_isr_12, _event_sensor_isr_13, _event_sensor_isr_14,
    _event_sensor_isr_15
};

void OOKDecoderSensor::_attach(OOKDecoderSensor * instance, unsigned char gpio, unsigned char mode) {
    if (!gpioValid(gpio)) return;
    _detach(gpio);
    unsigned char index = gpio > 5 ? gpio-6 : gpio;
    _event_sensor_instance[index] = instance;
    attachInterrupt(gpio, _event_sensor_isr_list[index], mode);
    #if SENSOR_DEBUG
        DEBUG_MSG_P(PSTR("[SENSOR] GPIO%d interrupt attached to %s\n"), gpio, instance->description().c_str());
    #endif
}

void OOKDecoderSensor::_detach(unsigned char gpio) {
    if (!gpioValid(gpio)) return;
    unsigned char index = gpio > 5 ? gpio-6 : gpio;
    if (_event_sensor_instance[index]) {
        detachInterrupt(gpio);
        #if SENSOR_DEBUG
            DEBUG_MSG_P(PSTR("[SENSOR] GPIO%d interrupt detached from %s\n"), gpio, _event_sensor_instance[index]->description().c_str());
        #endif
        _event_sensor_instance[index] = NULL;
    }
}

#endif // SENSOR_SUPPORT && OOK_SUPPORT
