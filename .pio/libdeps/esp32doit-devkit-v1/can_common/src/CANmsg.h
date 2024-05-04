#ifndef CANMSG_H
#define CANMSG_H

#include "can_common.h"
#include "esp32_can_builtin.h"
#include "esp32_can.h"

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

        void setup();
    public:
       CANmsg(gpio_num_t rx_id, gpio_num_t tx_id, uint32_t Baudrate);

        ~CANmsg(); 

        void init(callback isr); 

        void init(callback isr, uint32_t Id);

        void init(callback isr, uint32_t Id, uint32_t Mask);

        int Set_Filter(uint32_t Id, uint32_t Mask, bool Extended);

        void clear(uint32_t new_id, bool ext = false, uint8_t length = MAX_MESSAGE_LENGTH);

        bool write();

        bool SendMsgBuffer(uint32_t Id, bool ext, uint8_t length, unsigned char* data);

        void Set_Debug_Mode(bool md);    

        template<class T>
        CANmsg &operator<<(const T value)
        {
            //if(len + sizeof(T) > 8) return NULL
            memcpy(&msg.data.uint8[len], (uint8_t*)&value, sizeof(T));
            len += sizeof(T);
            return *this;
        }

        template<class Q>
        CANmsg &operator=(Q& val)
        {
            val = *reinterpret_cast<Q*>(&msg.data.uint8[0]);
            len -= sizeof(Q);
            memcpy(msg.data.uint8, msg.data.uint8 + sizeof(Q), len);
            return *this;
        }   
};

#endif