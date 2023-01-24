#include <RP2040.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/uart.h>
#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>
#include <hardware/irq.h>
#include <core_cm0plus.h>
#include "hardware/clocks.h"

#include <hardware/regs/intctrl.h>

#include "can2040.h"
// #include "can_frames.h"
// CAN_frames CAN1;

uint16_t curr_pos, target_pos;
typedef struct can2040 CANHandle;
typedef struct can2040_msg CANMsg;
struct can2040_msg tx_msg, rx_msg;

static CANHandle cbus;
volatile bool cb_called = false;
volatile uint32_t cb_notify;

char *msg_to_str(struct can2040_msg *msg) {

    static char buf[64], buf2[8];

    sprintf(buf, "[ %x ] [ %x ] [ ", msg->id, msg->dlc);
    for (uint32_t i = 0; i < msg->dlc && i < 8; i++) {
    sprintf(buf2, "%x ", msg->data[i]);
    strcat(buf, buf2);
    }
    strcat(buf, " ] ");

    if (msg->id & CAN2040_ID_RTR) {
    strcat(buf, "R");
    }

    if (msg->id & CAN2040_ID_EFF) {
    strcat(buf, "X");
    }
    
    return buf;
}

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg){
    // Add message processing code here...
    cb_called = true;
    cb_notify = notify;
    if (notify == CAN2040_NOTIFY_RX) {
    rx_msg = *msg;
    }
}

static void PIOx_IRQHandler(void){
    can2040_pio_irq_handler(&cbus);
}
#define CAN_RX      1 
#define CAN_TX      0

void canbus_setup(void){
    uint32_t pio_num = 1;
    uint32_t bitrate = 1000000;
    uint32_t gpio_rx = CAN_RX, gpio_tx = CAN_TX;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO1_IRQ_0_IRQn, PIOx_IRQHandler);
    NVIC_SetPriority(PIO1_IRQ_0_IRQn, 1);

    // Start canbus
    can2040_start(&cbus, clock_get_hz(clk_sys), bitrate, gpio_rx, gpio_tx);
    NVIC_EnableIRQ(PIO1_IRQ_0_IRQn);
}

union {
    uint8_t axis_val[4]; float axis_fval;
}position_union;


float parse_numeric_value(){
    int buffer = 0, decimal_power = 0;
    float var = 0;
    bool neg_data = false, first_byte = true, decimal_data = false;
    while (true)
    {   
        buffer = getchar_timeout_us(0);
        if(first_byte && buffer == '-'){
            neg_data = true;
            first_byte = false;
        }
        if(buffer>='0' && buffer<='9'){    
            var = var*10 + (buffer - 48);
            if (decimal_data){
                decimal_power++;
            }
        }
        else if(buffer == '.'){
            decimal_data = true;
        }
        else if(buffer == '\n' || buffer == ' ' || buffer == ','){
            break;
        }    
        else if (buffer <= 0 || buffer == '-'){
            return PICO_ERROR_TIMEOUT;

        }
    }
    if (neg_data == true) var = -var;
    if(decimal_data)    var = var/(pow(10, decimal_power));
    return var;
}

///////////////////////////////////////////////////////////////////////////////
typedef enum{
    default_address,
    set_stepper_home,
    set_stepper_pos,
    set_dxl1_pos,   //velocity, position
    set_dxl2_pos,   //velocity, position
    set_dxl_torque, //dxl0, dxl1
    get_stepper_pos,
    get_dxl_pos,
    get_dxl_curr    //dxl0, dxl1

} CAN_IDs;

int main(){

    stdio_init_all();

    sleep_ms(10);

    printf("Starting Initialization of CAN\n");
    canbus_setup();
    printf("Initialized CAN\n");

    sleep_ms(1000);
    while (true) {
        CANMsg msg = {0};
        uint8_t command = getchar_timeout_us(0);
        if(command == 'u' || command == 'U'){
            uint8_t sub_command = getchar_timeout_us(0);
            if(sub_command == '1'){
                // msg.data[0] = 0x02;
                msg.id = set_stepper_home;
                msg.data[0] = 0xCC;
                // msg.data[1] = 0xDD;
                printf("Homing\n");
                for(int i = 0; i <=7;i++){
                    printf("%x\n",msg.data[i]);
                }
                msg.dlc = 1;
            }
            else if(sub_command == '2'){
                float position = parse_numeric_value();
                // msg.data[0] = 0x01;
                msg.id = set_stepper_pos;
                position_union.axis_fval = position;
                msg.data[0] = position_union.axis_val[0];
                msg.data[1] = position_union.axis_val[1];
                msg.data[2] = position_union.axis_val[2];
                msg.data[3] = position_union.axis_val[3];
                msg.dlc = 4; 
                printf("Moved\n");
                for(int i = 0; i <=7;i++){
                    printf("%x\n",msg.data[i]);
                }        
            }    
            else if(sub_command == '3'){
                msg.id = set_dxl1_pos;
                float velocity = parse_numeric_value();
                float position = parse_numeric_value();
                
                position_union.axis_fval = velocity;
                for(int i=0; i<4; i++){
                    msg.data[i] = position_union.axis_val[i];
                }

                position_union.axis_fval = position;
                for(int i=4; i<8; i++){
                    msg.data[i] = position_union.axis_val[i-4];
                }

                msg.dlc = 8; 
            }
            else if(sub_command == '4'){
                msg.id = set_dxl2_pos;
                float velocity = parse_numeric_value();
                float position = parse_numeric_value();
                
                position_union.axis_fval = velocity;
                for(int i=0; i<4; i++){
                    msg.data[i] = position_union.axis_val[i];
                }

                position_union.axis_fval = position;
                for(int i=4; i<8; i++){
                    msg.data[i] = position_union.axis_val[i-4];
                }
                msg.dlc = 8;
            }
            else if(sub_command == '5'){
                // msg.data[0] = 0x02;
                msg.id = set_dxl_torque;
                msg.dlc = 2;

                float torque_dxl0 = parse_numeric_value();
                float torque_dxl1 = parse_numeric_value();
                msg.data[0] = (int)torque_dxl0 & 0xFF;
                msg.data[1] = (int)torque_dxl1 & 0xFF;

            }
            else if(sub_command == '6'){
                
                msg.id = get_stepper_pos | CAN2040_ID_RTR;
                msg.dlc = 4;

            }
            else if(sub_command == '7'){
                
                msg.id = get_dxl_pos | CAN2040_ID_RTR;
                msg.dlc = 4;

            }
            else if(sub_command == '8'){
                
                msg.id = get_dxl_curr | CAN2040_ID_RTR;
                msg.dlc = 4;

            }
            if(can2040_check_transmit(&cbus)){
                int res = can2040_transmit(&cbus, &msg);
                if (res) printf("Sent request on %X\n", msg.id);
            }
            else{
                printf("bus busy\n");
            }
        }
        else if(command == 'X'){
            printf("CAN2040\n");
            printf("ok\n");
        }
        
        if(cb_called){
            switch (cb_notify) {
                // received message
                case CAN2040_NOTIFY_RX:
                    // printf("cb: received ID %x \n", rx_msg.id);
                         
                    switch(rx_msg.id){
                        case 0x01:
                            printf("cb: received msg: %s\n", msg_to_str(&rx_msg));
                        break;
                        case get_dxl_curr:
                        case get_dxl_pos:
                        case get_stepper_pos:
                            
                            for(int i=0; i<4; i++){
                                position_union.axis_val[i] = rx_msg.data[i];
                            }

                            printf("data received: %f\n", position_union.axis_fval);
                        
                    }
                    break;

                // message sent ok
                case CAN2040_NOTIFY_TX:
                    // printf("cb: message sent ok\n");
                    break;

                // error
                case CAN2040_NOTIFY_ERROR:
                    printf("cb: an error occurred\n");
                    break;

                // unknown
                default:
                    printf("cb: unknown notification = %lu\n", cb_notify);
            }
            cb_called = 0;
        }
        
        sleep_ms(200);
    }
}
