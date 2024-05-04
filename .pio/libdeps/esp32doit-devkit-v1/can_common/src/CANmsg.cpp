#include "Arduino.h"
#include "CANmsg.h"

CAN_FRAME Tx_Rx_message;

CANmsg::CANmsg(gpio_num_t rx_id, gpio_num_t tx_id, uint32_t Baudrate) : msg(Tx_Rx_message)
{
    __rx_id = rx_id;
    __tx_id = tx_id;
    _baudrate = Baudrate;
};

CANmsg::~CANmsg()
{
    if(this->f)
    { 
        CAN.removeCallback();
        CAN.disable(); 
        this->f = false;
    }
};

bool CANmsg::init(callback isr)
{
    return this->init(isr, 0, 0);
}

bool CANmsg::init(callback isr, uint32_t Id)
{
    return this->init(isr, Id, 0);
} 

bool CANmsg::init(callback isr, uint32_t Id, uint32_t Mask)
{
    bool can_flag_init = false;
    CAN.setCANPins(__rx_id, __tx_id);

    if(CAN.begin(_baudrate))
    {
        can_flag_init = true;
        f = true;

        if(Id==0 && Mask==0)
            CAN.watchFor();
        if(Id!=0 && Mask==0)
            CAN.watchFor(Id);
        if(Id!=0 && Mask!=0)
            CAN.watchFor(Id, Mask);
        else 
            CAN.watchFor();

        CAN.setCallback(0, isr);
        if(Id != 0)  _ext = Id > 0x7FF ? 1 : 0;
        this->setup();
    }
    return can_flag_init;
} 

void CANmsg::setup()
{
    msg.id = 0x00;
    msg.length = 8;
    msg.extended = _ext; 
    msg.rtr = 0;
}

int CANmsg::Set_Filter(uint32_t Id, uint32_t Mask, bool Extended)
{
    return CAN.setRXFilter(Id, Mask, Extended);
}

void CANmsg::clear(uint32_t new_id, bool ext, uint8_t length)
{
    len = 0;
    msg.id = new_id;
    msg.length = length;
    msg.extended = ext & 0x01; 
    msg.rtr = 0;
    memset(msg.data.uint8, 0, 8);
}

bool CANmsg::write()
{
    return CAN.sendFrame(msg);
}

bool CANmsg::SendMsgBuffer(uint32_t Id, bool ext, uint8_t length, unsigned char* data)
{
    this->clear(Id, ext, length);
    memcpy(&msg.data.uint8, &data, length);
    return this->write();
}

void CANmsg::Set_Debug_Mode(bool md)
{
    CAN.setDebuggingMode(md);
}
