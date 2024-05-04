
#ifndef CANMSG_H
#define CANMSG_H

#include "CAN/can_common.h"
#include "CAN/esp32_can_builtin.h"
#include "CAN/esp32_can.h"

CAN_FRAME Tx_Rx_message;
#define CAN CAN0
#define MAX_MESSAGE_LENGTH 8
typedef void (*callback)(CAN_FRAME*);

class CANmsg
{
    private:
      uint8_t _ext = 0;
    protected:
        CAN_FRAME msg;
        int len = 0;
        bool f = false;

        void setup()
        {
            msg.id = 0x00;
            msg.length = 8;
            msg.extended = _ext; 
            msg.rtr = 0;
        }
    public:
        CANmsg(gpio_num_t rx_id, gpio_num_t tx_id, uint32_t Baudrate) : msg(Tx_Rx_message)
        {
            CAN.setCANPins(rx_id, tx_id);
            CAN.begin(Baudrate);
        }

        ~CANmsg() 
        { 
            if(f)
            { 
                CAN.removeCallback();
                CAN.disable(); 
                f = false;
            }
        };

        void init(callback isr) 
        {
            this->init(isr, 0, 0);
        } 

        void init(callback isr, uint32_t Id)
        {
            this->init(isr, Id, 0);
        }

        void init(callback isr, uint32_t Id, uint32_t Mask)
        {
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

        int Set_Filter(uint32_t Id, uint32_t Mask, bool Extended)
        {
            return CAN.setRXFilter(Id, Mask, Extended);
        }

        void clear(uint32_t new_id, bool ext = false, uint8_t length = MAX_MESSAGE_LENGTH)
        {
            len = 0;
            msg.id = new_id;
            msg.length = length;
            msg.extended = ext & 0x01; 
            msg.rtr = 0;
            memset(msg.data.uint8, 0, 8);
        }

        bool write()
        {
            return CAN.sendFrame(msg);
        }

        void Set_Debug_Mode(bool md)    
        {
            CAN.setDebuggingMode(md);
        }

        bool SendMsgBuffer(uint32_t Id, bool ext, uint8_t length, unsigned char* data)
        {
            this->clear(Id, ext, length);
            memcpy(&msg.data.uint8, &data, length);
            return this->write();
        }

        template<class T>
        CANmsg &operator<<(const T value)
        {
            //if(len + sizeof(T) > 8) return NULL
            memcpy(&msg.data.uint8[len], (uint8_t *)&value, sizeof(T));
            len += sizeof(T);
            return *this;
        }
};


#endif
