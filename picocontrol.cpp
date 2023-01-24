#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "encoders.pio.h"
// #include "pico/unique_id.h"

#include "Dynamixel2Pico.h"
#include "project_config.h"
#include "pins_picofeeder_board.h"
#include "encoder_config.h"
#include "functions.h"
#include "I2C_comm.h"
#include "vcnl4040.h"
#include "servo.h"
#include "flash.h"
#include "Trapezoidal_Profile.h"
#include "dynamixel_functions.h"
#include "can_functions.h"
#include "can2040.h"
#include "can_frames.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/pwm.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include <RP2040.h>
#include "hardware/clocks.h"

typedef struct can2040 CANHandle;
typedef struct can2040_msg CANMsg;

// pico_unique_board_id_t id_out;

PIO pio;
struct repeating_timer timer1, timer2;

// vars
int LED_intensity, servo_angle, servo_current_limit;

int directions[num_of_axis] = {axis_dir};
const uint encoder_pin[num_of_axis] = {ENC1_A};
uint number_of_encoders = num_of_axis;
uint encoder_resoultion[num_of_axis] = {encoder_PPR*4};

float encoder_mm_per_revolution[num_of_axis] = {(Y_BELT_PITCH*Y_PULLEY_TEETH)};
volatile float encoder_mm_per_click[num_of_axis];
volatile float axis_offset[num_of_axis] = {0}, added_offset[num_of_axis] = {0};
float homing_position[num_of_axis] = {2};               // the absolute position of the loader when it is homed
float bed_size[num_of_axis] = {300};
volatile int32_t capture_buf[num_of_axis] = {0};
float Accel= 450, Decel= 450;

volatile int print_done=0;


using namespace ControlTableItem;
uint8_t turn_on = 1;
uint8_t turn_off = 0;
uint8_t DXL_ID = 1;
const uint8_t DXL_1_ID = 1;
const uint8_t DXL_2_ID = 2;
const uint8_t DXL_3_ID = 3;
const uint8_t numOfMotors = 3;
const uint8_t DXL_IDS[numOfMotors] = {DXL_1_ID, DXL_2_ID, DXL_3_ID};
const float DXL_PROTOCOL_VERSION = 2.0;
volatile int16_t goal_position;
uint8_t operatingMode = POSITION_CONTROL_MODE;
Dynamixel2Pico dxl(DXL_SERIAL, DXL_DIR_PIN, UART_RX_PIN, UART_TX_PIN);

uint32_t userInput = 0;
int command = 0;
bool ret = false;

double servo_current_consumption = 0, servo_current_failure, temp_current_consumption = 0, servo_current_integrate_counter = 0, servo_counter;
int windowsize = 1000;
// int servo_adc_readings[windowsize];
bool servo_flag = true, enable_servo = true, last_enable_servo = false;
uint16_t CURRENT_LIMIT_count = 0; 

void dynamixel_loop();

void GPIO_init(void){
    gpio_init(LED_SIG);
    gpio_set_dir(LED_SIG, GPIO_OUT);
    gpio_put(LED_SIG, LOW);

    gpio_init(LED_STAT);
    gpio_set_dir(LED_STAT, GPIO_OUT);
    gpio_put(LED_STAT, LOW);

    gpio_init(BOOST_EN);
    gpio_set_dir(BOOST_EN, GPIO_OUT);
    gpio_put(BOOST_EN, LOW);
    
    gpio_init(LED_DRV_PIN);
    gpio_set_function(LED_DRV_PIN, GPIO_FUNC_PWM);

    gpio_init(MOTOR_DIR);
    gpio_set_dir(MOTOR_DIR, GPIO_OUT);
    gpio_put(MOTOR_DIR, LOW);

    gpio_init(MOTOR_STEP);
    gpio_set_dir(MOTOR_STEP,GPIO_OUT);
    gpio_set_function(MOTOR_STEP, GPIO_FUNC_PWM);

    gpio_init(MOTOR_ENA);
    gpio_set_dir(MOTOR_ENA, GPIO_OUT);
    gpio_put(MOTOR_ENA, LOW);

    gpio_init(LS1);
    gpio_set_dir(LS1,GPIO_IN);
}

void dma_handler() {
    uint i = 1;
    int interrupt_channel = 0; 
    while ((i & dma_hw->ints0) == 0) {  
        i = i << 1; 
        ++interrupt_channel; 
    } 
    dma_hw->ints0 = 1u << interrupt_channel;
    dma_channel_set_read_addr(interrupt_channel, &pio->rxf[interrupt_channel], true);
}

inline void current_calc(){
    if(servo_current_integrate_counter < windowsize){
            busy_wait_us(15);
            temp_current_consumption += adc_read();
            servo_current_integrate_counter++;
            // printf("consumption: %d  angle: %d current_counter: %d\n", servo_current_consumption, servo_angle, servo_current_integrate);
        
        }
        else{    
            servo_current_consumption = temp_current_consumption / servo_current_integrate_counter;
            printf_debug("consumption: %f current_counter: %f\n", servo_current_consumption, servo_current_integrate_counter);
            temp_current_consumption = 0;
            servo_current_integrate_counter = 1;
        }
        // servo_current_consumption = ((servo_current_consumption * (windowsize -1)) + adc_read()) / windowsize;
        // printf("serposi %d\n", servo_current_consumption);
        if (servo_counter > windowsize) servo_flag = true;
        else   servo_counter++;

        if(servo_flag){ 
            if (servo_current_consumption > CURRENT_LIMIT){ //checked dynamixel in nymeria max allowable
                if(CURRENT_LIMIT_count > windowsize+1){
                    // printf("servo error %f\n", servo_current_consumption);
                    servo_current_failure = servo_current_consumption;
                    gpio_put(BOOST_EN, false);
                    enable_servo = false;
                    CURRENT_LIMIT_count = 0;
                }
                else{
                    busy_wait_us(15);
                    CURRENT_LIMIT_count++;
                }
                servo_flag = false;
                servo_counter = 0;
            }
        }
}

#define BUFFER_LENGTH   20
uint8_t buffer[BUFFER_LENGTH];
uint16_t buffer_index= 0, EOF_index = 0;
bool SOF_flag = false, EOF_flag = false;
bool get_block(struct repeating_timer *t) {
    // buffer_index = 0;
    // while(true){
        int c = getchar_timeout_us(0);
        // printf("%c\n", c);
        if (c != PICO_ERROR_TIMEOUT && buffer_index < BUFFER_LENGTH && !EOF_flag) {
            if(c == '<'){
                SOF_flag = true;
                buffer_index = 0;
            }
            else if(c == '>'){
                EOF_flag = true;
                EOF_index = buffer_index;
                // break;
            }
            if(SOF_flag && !EOF_flag){
                buffer[buffer_index++] = (c & 0xFF);
            }
            
        } else {
            // break;
        }
    // }
    return true;
}

CANMsg msg = {0}; 
       
union {
    uint8_t axis_val[4]; float axis_fval;
}position_union;

inline int can_loop(){       
    if(cb_called){
        switch (cb_notify) {
            // received message
            case CAN2040_NOTIFY_RX:
                printf("cb: received ID %x \n", rx_msg.id);
                if(rx_msg.id & CAN2040_ID_RTR){
                    // msg.id = rx_msg.id & 0xFF;
                    switch (rx_msg.id&0xFF){
                        case get_stepper_pos:
                            msg.id = rx_msg.id & 0xFF;
                            msg.dlc = 4;
                            position_union.axis_fval = (homing_position[0]+ added_offset[0] + (pow(-1,!directions[0]))*(capture_buf[0]-axis_offset[0])*encoder_mm_per_click[0]);
                            for (int i=0; i<4; i++){
                                msg.data[i] = position_union.axis_val[i];
                            }
                            printf("stepper pos: %f \n", position_union.axis_fval);
                            break;
                        case get_dxl_pos:
                            msg.id = rx_msg.id & 0xFF;
                            msg.dlc = 8;
                            position_union.axis_fval = getPosition(dxl, DXL_1_ID);
                            for (int i=0; i<4; i++){
                                msg.data[i] = position_union.axis_val[i];
                            }
                            printf("dxl1 pos: %f \n", position_union.axis_fval);
                            position_union.axis_fval = getPosition(dxl, DXL_2_ID);
                            for (int i=0; i<4; i++){
                                msg.data[i] = position_union.axis_val[i];
                            }
                            printf("dxl2 pos: %f \n", position_union.axis_fval);
                            break;
                        case get_dxl_curr:
                            msg.id = rx_msg.id & 0xFF;
                            msg.dlc = 8;

                        default:
                            printf("wrong ID: %X\n", msg.id);
                        
                            break;
                    }
                    // msg.id = rx_msg.id && ~(CAN2040_ID_RTR) ;
                    if(can2040_check_transmit(&cbus)){
                        int res = can2040_transmit(&cbus, &msg);
                        printf("ack request for %X\n success: %d\n", msg.id, res);
                    }
                    else{
                        printf("bus busy\n");
                    }
                    printf("msg ID: %X\n", msg.id);                    
                }
                else{ 
                    switch(rx_msg.id){
                        case set_stepper_pos:
                            for(int i=0; i<4; i++){
                                position_union.axis_val[i] = rx_msg.data[i];
                            }
                            printf("position received: %f\n", position_union.axis_fval);
                            Trap_target(position_union.axis_fval);
                            
                            printf("cb: received msg: %s\n", msg_to_str(&rx_msg));
                        break;
                        case set_stepper_home:
                            Trap_home();
                        break;
                        case set_dxl1_pos:
                            for(int i=0; i<4; i++){
                                position_union.axis_val[i] = rx_msg.data[i];
                            }
                            dxl.torqueOff(DXL_1_ID);
                            setVelocity(dxl, DXL_1_ID, (int)(position_union.axis_fval));
                            dxl.torqueOn(DXL_1_ID);
                            for(int i=4; i<8; i++){
                                position_union.axis_val[i-4] = rx_msg.data[i];
                            }
                            moveTo(dxl, DXL_1_ID, position_union.axis_fval);
                            printf("cb: received msg: %s\n", msg_to_str(&rx_msg));
                        break;
                        case set_dxl2_pos:
                            for(int i=0; i<4; i++){
                                position_union.axis_val[i] = rx_msg.data[i];
                            }
                            dxl.torqueOff(DXL_2_ID);
                            setVelocity(dxl, DXL_2_ID, (int)(position_union.axis_fval));
                            dxl.torqueOn(DXL_2_ID);
                            for(int i=4; i<8; i++){
                                position_union.axis_val[i-4] = rx_msg.data[i];
                            }
                            moveTo(dxl, DXL_2_ID, position_union.axis_fval);
                            printf("cb: received msg: %s\n", msg_to_str(&rx_msg));
                        break;
                        
                        
                    }
                }
                break;

            // message sent ok
            case CAN2040_NOTIFY_TX:
                printf("cb: message sent ok\n");
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
    return true;  
}


int main() {
    stdio_init_all();
    pwm_int();

    // Set up the state machine.
    pio = ENC_PIO;
    
    for(int i=0;i<num_of_axis;i++){
    	encoder_mm_per_click[i] = encoder_mm_per_revolution[i] / encoder_resoultion[i];
    }
    
    uint offset = pio_add_program(pio, &encoders_program);    
    for (uint i = 0; i < number_of_encoders; i++){
        encoders_program_init(pio, i, offset, encoder_pin[i], false);
         
        dma_channel_config c = dma_channel_get_default_config(i);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio, i, false));
        dma_channel_configure(i, &c,
            &capture_buf[i],    // Destinatinon pointer
            &pio->rxf[i],       // Source pointer
            0xffff,             // Number of transfers
            true                // Start immediately
        );
        irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        dma_channel_set_irq0_enabled(i, true);
    }
    
    // int16_t proxValue;
    axis_offset[0] = 0;
    axis_offset[1] = 0;
    axis_offset[2] = 0;

    // i2c_begin(i2c0, I2C_Baudrate, i2c0_scl_pin, i2c0_sda_pin);
    // i2c_begin(i2c1, I2C_Baudrate, i2c1_scl_pin, i2c1_sda_pin);
    
    GPIO_init();
    
    adc_init();
    adc_gpio_init(26);// Make sure GPIO is high-impedance, no pullups etc
    adc_select_input(0); //Or Select ADC input 1 (GPIO27)
    adc_set_clkdiv(25600);

    sleep_ms(3000);
    canbus_setup();
    printf("Status: %d\n", add_repeating_timer_us(-10*TICK_RATE, get_block, NULL, &timer1));

    // watchdog_enable(5000, 1);
    uint16_t timeout_power = 0;
    printf("Status: %d\n", add_repeating_timer_us(-TICK_RATE, repeating_timer_callback, NULL, &timer2));
    control_status = STATE_DISABLED;

    dxl.begin(57600);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    sleep_ms(50);
    dynamixelSetup(dxl, DXL_2_ID);
    sleep_ms(50);
    dynamixelSetup(dxl, DXL_1_ID);
    sleep_ms(50);

    // while(!stdio_usb_connected());
    printf("Hello\n");
    gpio_put(BOOST_EN, HIGH);
    while (true){ 
        can_loop();
        // if(stdio_usb_connected()){
        //     if(gpio_get_out_level(LED_STAT)==0) gpio_put(LED_STAT, true);   //For indication error that USB comm isnt happening
        //     if(last_enable_servo == false){
        //         if(gpio_get_out_level(BOOST_EN)==0) gpio_put(BOOST_EN, true);
        //         busy_wait_ms(500);
        //         last_enable_servo = true;
        //     }
        //     if(enable_servo){
        //         if(gpio_get_out_level(BOOST_EN)==false){
        //             gpio_put(BOOST_EN, true);  //Disable Boost circuit if USB is disconnected from PC
        //             busy_wait_ms(500);
        //         }
        //     }
        // }
        // else if(timeout_power > 50000){
        //     gpio_put(LED_STAT, false);   //For indication error that USB comm isnt happening
        //     gpio_put(BOOST_EN, false);  //Disable Boost circuit if USB is disconnected from PC
        //     timeout_power = 0;
        //     sleep_ms(250);
        //     gpio_put(LED_STAT, true);   //For indication error that USB comm isnt happening
        //     sleep_ms(250);
        // }
        // else{
        //     timeout_power++;
        // }

        // current_calc();
        if(EOF_flag == false){

        }
        else{
            gpio_put(LED_SIG, true);   //For indication error that USB comm isnt happening
            sleep_ms(1);
            gpio_put(LED_SIG, false);   //For indication error that USB comm isnt happening
            sleep_ms(1);
            float value;
                
            switch(buffer[1]){
            case 'b':
                // int intensity = parse_numeric_value();
                // if(intensity >= 0){
                //     pwm_set_freq_duty(LED_DRV_PIN, 2000, intensity);
                //     printf("%c %d\n",command, intensity);
                // }
                msg.id = 0x01;
                msg.data[0] = 0xAA;
                msg.data[1] = 0xBB;
                msg.dlc = 2;
                if(can2040_check_transmit(&cbus)){
                    printf("bus free\n");
                    int res = can2040_transmit(&cbus, &msg);
                    if (res) printf("Sent request on %X\n", msg.id);
                    else    printf("failed\n");
                }
                else{
                    printf("bus busy\n");
                }
                break;
            case 'a':
                
                if(buffer[2] == 'v'){
                    value=0;
                    for(int i=3; i< EOF_index; i++){
                        if(buffer[i]>='0' && buffer[i]<='9'){    
                            value = value*10 + (buffer[i] - 48);
                        }
                    }
                    print_done = 1;
                    bool status = Trap_target(value);
                    if(!status){
                        printf("busy!\n");
                    }
                    else{
                        printf("position: %f\n", value);
                    }
                }
                else if(buffer[2] == 'p'){
                    set_percentage = parse_numeric_value();
                    printf("percentage: %f\n", set_percentage);
                }
                else if(buffer[2] == 'c'){
                    value=0;
                    for(int i=3; i< EOF_index; i++){
                        if(buffer[i]>='0' && buffer[i]<='9'){    
                            value = value*10 + (buffer[i] - 48);
                        }
                    }
                    if(value){
                        Accel = value;
                        printf("Acceleration: %f\n", Accel);
                    }
                    else{
                        printf("accel #\n");
                    }
                }
                else if(buffer[2] == 'd'){
                    value=0;
                    for(int i=3; i< EOF_index; i++){
                        if(buffer[i]>='0' && buffer[i]<='9'){    
                            value = value*10 + (buffer[i] - 48);
                        }
                    }
                    if(value){
                        Decel = value;
                        printf("Deceleration: %f\n", Decel);
                    }
                    else{
                        printf("decel #\n");
                    }
                }
                break;

            case 'I':
                printf("value dxloop: %d\n", (int)(buffer[2]-48));
                value = buffer[2]-48;
                if(value > 0 && value < 9){
                    DXL_ID = value;
                    dynamixel_loop();
                }
                else if (value == 0){
                    for(int id = 0; id < DXL_BROADCAST_ID; id++) {
                        //iterate until all ID in each buadrate is scanned.
                        if(dxl.ping(id)) {
                            printf("ID : ");
                            printf(" %d ",id);
                            printf(", Model Number: ");
                            printf(" %d \n", dxl.getModelNumber(id));
                        }
                    }
                }
                break;
            case 'e':
                value=0;
                for(int i=2; i< EOF_index; i++){
                    if(buffer[i]>='0' && buffer[i]<='9'){    
                        value = value*10 + (buffer[i] - 48);
                    }
                }
                set_speed = value;
                printf("Target speed: %f\n", set_speed);
                break;
            case 's':   
                value=0;
                for(int i=2; i< EOF_index; i++){
                    if(buffer[i]>='0' && buffer[i]<='9'){    
                        value = value*10 + (buffer[i] - 48);
                    }
                }
                if(value >= 0){
                    
                    sleep_ms(100);
                }
                else{
                    printf("#\n");
                }
                break;
            case 'z':
                printf("%d %f\n", servo_angle, ((servo_current_consumption*(3.3/4095)/3.7)/0.5)*1000);
                break;

            // to print the name of the device
            case 'u':
                if(buffer[2] == '2'){
                    printf(BOARD_TYPE "\n");
                    printf("ok\n");
                }
                break;
            // to set the home to the current encoder position
            case 'h':
                
                switch(buffer[2]){
                    case 'x':
                        Trap_home();
                        // axis_offset[0] = capture_buf[0];
                        // added_offset[0] = 0;
                    break;
                    case 'b':
                        
                        
                    break;
                    case 'a':
                    
                    break;
                }
                break;
                
                // printf("%c\n", command);
            case 'd':
                
                switch(buffer[2]){
                    case 'x':
                        added_offset[0] = (value-homing_position[0]);
                        axis_offset[0] = capture_buf[0];
                    break;
                    case 'b':
                    
                    break;
                }
                break;
                
            case '9':
                
                sleep_ms(1);
                break;

            case 'x':
                printf("%c\n",command);
                break;
                
            // to print the current encoder position minus the home position
            case 'p':
                for (uint i = 0; i < number_of_encoders; i++){
                    printf("%f ", homing_position[i]+ added_offset[i] + (pow(-1,!directions[i]))*(capture_buf[i]-axis_offset[i])*encoder_mm_per_click[i]);
                    printf("add offdet: %f, dir: %f, cap: %f, axisoff:%f", added_offset[0], (pow(-1,!directions[0])), capture_buf[0], axis_offset[0]);
                    // printf("%d ", capture_buf[i]-axis_offset[i]);
                }
                printf("\n");
                // printf("%f, %f, %f, %f, %f, %f \n", homing_position[1], added_offset[1], directions[1], capture_buf[1], axis_offset[1], encoder_mm_per_click[1]);
                sleep_ms(1);
                break;
            case '?':
                
                if (buffer[2] == 'I'){     
                    if(buffer[3] == 'D'){
                        printf(BOARD_TYPE "-" HW_VERSION " " FW_TYPE "-" FW_VERSION "\n");
                        printf("AUTHOR: " AUTHOR "\n");
                        // pico_get_unique_board_id(&id_out);
                        // printf("Board ID: ");
                        // for(int i=0; i<PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++){
                        //     printf("%X", id_out.id[i]);
                        // }
                        printf("\n");
                    }
                }
                else if(buffer[2] == 'r'){
                    enable_servo = true;
                    last_enable_servo = false;
                    // servo_set_position(SERVO_PIN, 10);
                    servo_current_failure = 0;
                    busy_wait_ms(50);

                }
                else if(buffer[2] == 's'){
                    printf("error: ");
                    if(enable_servo==false){
                        printf("Servo disabled");
                        printf(" current: %f", servo_current_failure);
                    }
                    printf("\n");

                }
                else if (buffer[2] == '\n'){
                    print_commandlist();
                }
                break;
            case '#':
                
                if (buffer[2] == 'F'){
                    if(buffer[3] == 'P'){
                        if(buffer[4] == 'C'){
                            value=0;
                            for(int i=5; i< EOF_index; i++){
                                if(buffer[i]>='0' && buffer[i]<='9'){    
                                    value = value*10 + (buffer[i] - 48);
                                }
                            }
                            if (value > 0){
                                servo_current_limit = value;
                                update_flash_data();
                                printf("servo current limit updated to %d \n", servo_current_limit);
                            }
                        }
                        else if (buffer[4] == 'A'){
                            value=0;
                            for(int i=5; i< EOF_index; i++){
                                if(buffer[i]>='0' && buffer[i]<='9'){    
                                    value = value*10 + (buffer[i] - 48);
                                }
                            }
                            if (value > 0){
                                servo_angle = value;
                                update_flash_data();
                                printf("servo_angle updated to %d \n", servo_angle);
                            }   
                        }
                    }
                }
                else if(buffer[2] == 'R'){
                    if(buffer[3] == 'P'){
                        value=0;
                        for(int i=4; i< EOF_index; i++){
                            if(buffer[i]>='0' && buffer[i]<='9'){    
                                value = value*10 + (buffer[i] - 48);
                            }
                        }
                        if(value == SECRET_CODE){
                            reset_usb_boot(0, 0);
                        }
                    }
                }
                break;
            }
            buffer_index = 0;
            EOF_index = 0;
            SOF_flag = false;
            EOF_flag = false;
        }
    }
}

void dynamixel_loop(){
    float value;
    float current_position, current_velocity, current_angle;
    printf("buffer 3: %c\n", buffer[3]);             
    switch (buffer[4]){ 
        case 'P':              
            if(buffer[5] == '?'){
                current_position = getPosition(dxl, DXL_ID);
                printf("Present position: %f\n", current_position);
            }
            else if(buffer[5] == '='){
                printf("buffer 4: %c\n", buffer[4]);
                value=0;
                for(int i=6; i< EOF_index; i++){
                    if(buffer[i]>='0' && buffer[i]<='9'){    
                        value = value*10 + (buffer[i] - 48);
                    }
                }
                if(value >= 0){ 
                    goal_position = value; // Set Goal Position
                    printf("Goal Position : %" PRIu32 "\n", goal_position);
                    moveTo(dxl, DXL_ID, goal_position);
                }
            }
            break;

        case 'Q':
            printf("%f\n", dxl.getPresentCurrent(DXL_ID)); // present current value
            break;

        case 'Z':
            value=0;
            for(int i=5; i< EOF_index; i++){
                if(buffer[i]>='0' && buffer[i]<='9'){    
                    value = value*10 + (buffer[i] - 48);
                }
            }
            if(value){
                dxl.torqueOn(DXL_ID);
            }
            else{
                dxl.torqueOff(DXL_ID);
            }
            break;

        case 'I':
            value=0;
            for(int i=5; i< EOF_index; i++){
                if(buffer[i]>='0' && buffer[i]<='9'){    
                    value = value*10 + (buffer[i] - 48);
                }
            }
            if (value > 0 && value <200){
                printf("writing new ID %d to %d\n", value, DXL_ID);
                dxl.torqueOff(DXL_ID);
                printf(dxl.setID(DXL_ID, value)?"Success":"failed");
                dxl.torqueOn(value);
                printf("\n");
            }
            break;

        case 'A':
            
            if(buffer[5] == '?'){
                current_angle = getCurrentAngle(dxl, DXL_ID);
                printf("Present angle: %f\n", current_angle);
            }
            else if(buffer[5] == '='){
                value=0;
                for(int i=6; i< EOF_index; i++){
                    if(buffer[i]>='0' && buffer[i]<='9'){    
                        value = value*10 + (buffer[i] - 48);
                    }
                }
                if(value >= 0 && value <= 360){
                    //   printf("Goal angle : %" PRIu32 "\n", userInput);
                    printf("REACHED AT ANGLE\n");
                    moveToAngle(dxl, DXL_ID, value); // Set Goal Angle
                }
            } 
            break;

        case 'v':
            printf("buffer 4: %c\n", buffer[4]);
            if(buffer[5] == '?'){
                current_velocity = getCurrentVelocity(DXL_ID, UNIT_RAW);
                printf("Current Velocity: %f", current_velocity);
            }
            else if (buffer[5] == '='){
                value=0;
                for(int i=6; i< EOF_index; i++){
                    if(buffer[i]>='0' && buffer[i]<='9'){    
                        value = value*10 + (buffer[i] - 48);
                    }
                }
                printf("velocity value: %d", (int)(value));
                dxl.torqueOff(DXL_ID);
                setVelocity(dxl, DXL_ID, (int)(value));
                dxl.torqueOn(DXL_ID);
            }
            break;

        case 'l':
            printf("Blinking Dynamixel LED\n");
            dxl.ledOn(DXL_ID);  // Turn on the LED on DYNAMIXEL
            sleep_ms(200);
            dxl.ledOff(DXL_ID); // Turn off the LED on DYNAMIXEL
            sleep_ms(200);
            break;

        case 'k':
            if(buffer[5] == '?'){
                uint16_t p = dxl.readControlTableItem(POSITION_P_GAIN, DXL_ID);
                uint16_t i = dxl.readControlTableItem(POSITION_I_GAIN, DXL_ID);
                uint16_t d = dxl.readControlTableItem(POSITION_D_GAIN, DXL_ID);
                printf("pid values: %d %d %d\n", p, i, d);
            }
            else if(buffer[5] == 'p'){
                value=0;
                for(int i=6; i< EOF_index; i++){
                    if(buffer[i]>='0' && buffer[i]<='9'){    
                        value = value*10 + (buffer[i] - 48);
                    }
                }
                dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, value);
            }
            else if(buffer[5] == 'i'){
                value=0;
                for(int i=6; i< EOF_index; i++){
                    if(buffer[i]>='0' && buffer[i]<='9'){    
                        value = value*10 + (buffer[i] - 48);
                    }
                }
                dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, value); 
            }
            else if(buffer[5] == 'd'){
                value=0;
                for(int i=6; i< EOF_index; i++){
                    if(buffer[i]>='0' && buffer[i]<='9'){    
                        value = value*10 + (buffer[i] - 48);
                    }
                }
                dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, value);
            }
            break;

        case 'r':
            printf("Command: %c\n",command);
            ret = dxl.factoryReset(DXL_ID, 0x02, DXL_TIMEOUT);

            if(ret) {
                printf("Reset successfull !!!\n");
                sleep_ms(1000);
                dynamixelSetup(dxl, 1);
            }
            else {
                printf("Reset failed!!!\n");
            }
            break;

        case 'T':            
            if(buffer[5] == '0'){
                dxl.torqueOff(DXL_ID);
                //   printf("Torque off \n");
            }
            else if(buffer[5] == '1'){
                userInput = parse_numeric_value();
                if(userInput >= 0){ 
                    dxl.torqueOn(DXL_ID);
                    //   printf("Torque on \n");
                }
            }
            break;
    }
}
