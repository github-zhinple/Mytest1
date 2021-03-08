/*
 * single_fire_switch_app.h
 *
 *  Created on: 2020年12月6日
 *      Author: zhinple
 */

#ifndef SINGLE_FIRE_SWITCH_APP_H_
#define SINGLE_FIRE_SWITCH_APP_H_

#include <stdint.h>
#include <stdbool.h>
#include "hal/micro/unix/compiler/gcc.h"
#include "stack/include/ember-types.h"
#include "event_control/event.h"
#include "hal/micro/led.h"
#include "em_cmu.h"



#define MODEL_ID  "sg-fire-sw-3"
#define SOFT_VER "v1_0_0"

#define SW_CFG_TOKEN_DATA_ADDR 0x0001

#define KEYS_INTERVAL  200  //

#define  NETWORK_LINK_PERIOD   40 //40*250ms= 10s
#define  NETWORK_LINK_TIMEOUT_MAX_CNT   18 //18*10s = 180s = 3min


#define  NETWORK_LEAVE_PERIOD  40

#define HEARTBEAT_PRIOD 180 //S

#define SW_POWER_UP_STATE false
#define DEFAULT_SLEEP_STATE false


#define ZERO_CROSS_SIGNAL_PORT                      (gpioPortD)
#define ZERO_CROSS_SIGNAL_PIN                        (2U)
#define ZERO_CROSS_SIGNAL_GPIO_DOUT                  (HAL_GPIO_DOUT_LOW)
#define ZERO_CROSS_SIGNAL_GPIO_MODE                  (HAL_GPIO_MODE_INPUT)



#define SW0_LED_ON()      halSetLed(BOARDLED3)
#define SW0_LED_OFF()     halClearLed(BOARDLED3)
#define SW0_LED_TOGGLE()  halToggleLed(BOARDLED3)

#define SW1_LED_ON()      halSetLed(BOARDLED2)
#define SW1_LED_OFF()     halClearLed(BOARDLED2)
#define SW1_LED_TOGGLE()  halToggleLed(BOARDLED2)

#define SW2_LED_ON()      halSetLed(BOARDLED1)
#define SW2_LED_OFF()     halClearLed(BOARDLED1)
#define SW2_LED_TOGGLE()  halToggleLed(BOARDLED1)






enum ERR_LIST
{
    NO_ERR         = 0,
    PARA_POINT_ERR = -1,
    CMD_BYTES_LEN_ERR = -2,
    CLI_CMD_ERR = -3,

};



enum
{
    RALAY_ON1 = 0,
    RALAY_OFF1,
    RALAY_ON2,
    RALAY_OFF2,
    RALAY_ON3,
    RALAY_OFF3,
    RALAY_MAX,
};



#define SW0_RALAY_ON()    hal_set_ralay(RALAY_ON1)
#define SW0_RALAY_OFF()   hal_set_ralay(RALAY_OFF1)
#define SW0_RALAY_END()   hal_clear_ralay(RALAY_ON1);hal_clear_ralay(RALAY_OFF1);


#define SW1_RALAY_ON()   hal_set_ralay(RALAY_ON2)
#define SW1_RALAY_OFF()  hal_set_ralay(RALAY_OFF2)
#define SW1_RALAY_END()  hal_clear_ralay(RALAY_ON2);hal_clear_ralay(RALAY_OFF2);


#define SW2_RALAY_ON()    hal_set_ralay(RALAY_ON3)
#define SW2_RALAY_OFF()   hal_set_ralay(RALAY_OFF3)
#define SW2_RALAY_END()   hal_clear_ralay(RALAY_ON3);hal_clear_ralay(RALAY_OFF3);

#define SW_RALAY_END() hal_clear_ralay(RALAY_ON1);hal_clear_ralay(RALAY_OFF1);\
                       hal_clear_ralay(RALAY_ON2);hal_clear_ralay(RALAY_OFF2);\
                       hal_clear_ralay(RALAY_ON3);hal_clear_ralay(RALAY_OFF3);



#define ON  true
#define OFF false


#define SW0_DEFAULT_ENDPOINT 1
#define SW1_DEFAULT_ENDPOINT 2



typedef struct {
  GPIO_Port_TypeDef   port;
  unsigned int        pin;
} RALAY_ARRAY;

#define BSP_RALAY_PRESENT                         (1)


#define BSP_RALAY_ON1_PIN                         (0U)
#define BSP_RALAY_ON1_PORT                        (gpioPortA)

#define BSP_RALAY_OFF1_PIN                        (0U)
#define BSP_RALAY_OFF1_PORT                       (gpioPortD)


#define BSP_RALAY_ON2_PIN                         (4U)
#define BSP_RALAY_ON2_PORT                        (gpioPortD)

#define BSP_RALAY_OFF2_PIN                        (1U)
#define BSP_RALAY_OFF2_PORT                       (gpioPortD)

#define BSP_RALAY_ON3_PIN                         (3U)
#define BSP_RALAY_ON3_PORT                         (gpioPortD)

#define BSP_RALAY_OFF3_PIN                        (0U)
#define BSP_RALAY_OFF3_PORT                       (gpioPortC)

#define BSP_RALAY_COUNT                           (6U)
#define BSP_RALAY_INIT                            { { BSP_RALAY_ON1_PORT, BSP_RALAY_ON1_PIN }, { BSP_RALAY_OFF1_PORT, BSP_RALAY_OFF1_PIN }, { BSP_RALAY_ON2_PORT, BSP_RALAY_ON2_PIN }, { BSP_RALAY_OFF2_PORT, BSP_RALAY_OFF2_PIN }, { BSP_RALAY_ON3_PORT, BSP_RALAY_ON3_PIN }, { BSP_RALAY_OFF3_PORT, BSP_RALAY_OFF3_PIN } }
#define HAL_RALAY_COUNT                           (6U)
#define HAL_RALAY_ENABLE                          { 0, 1, 2, 3, 4, 5 }
#define BSP_RALAY_POLARITY                        (1)

#define RALAY_CONTROL_PROID_MS 6

enum SW_SERIRS_LIST
{
   SW_0 = 0,//!< SW_0
   SW_1,    //!< SW_1
   SW_2,    //!< SW_2
   SW_NUM,
};


enum POWER_UP_CFG_UPLOAD_STEP
{
   STEP_MODEL_ID_0 = 0,//!< SW_0
   STEP_MODEL_ID_1,    //!< SW_1
   STEP_SOFT_VER_1,    //!< SW_2
   STEP_SOFT_VER_2,    //!< SW_2

   STEP_SW0,
   STEP_SW1,
   STEP_SW2,
};



enum key_process_step
{
   STEP_KEYS_EVENT_TEST = 0,
   STEP_KEYS_INTERVAL,
   STEP_ZERO_CROSS_TEST,
   STEP_SW_ON_OFF,

};

typedef enum
{
   START_JOIN_NETWORK = 0,
   LEAVE_NETWORK,
   SW0_SHORT_PRESS,
   SW1_SHORT_PRESS,
   SW2_SHORT_PRESS,
   SW0_PRESS,
   SW1_PRESS,
   SW2_PRESS,
   LOST_PRAENT_REJOIN_NETWORK,

}KEY_PRESS_EVENT_LIST;

typedef enum
{
  CMD_LIST = 0,
  GET_CFG,
  SET_HEARTBEAT_PERIOD,
  SET_SWITCH_INIT_STATE,
  SET_SLEEP_ENABLE_STATE,
  CLI_CMD_MAX_INDEX,

}CLI_CMD_INDEX;







typedef struct none
{
  bool sw0_state;
  bool sw1_state;
  bool sw2_state;
  uint8_t network_link_cnt;
  uint16_t key_press_event_flag;
  uint8_t network_link_time;
  uint8_t network_leave_time;
  bool switch_power_up_state;
  uint16_t heartbeat_period;

  void (*sw_ctl_callb)(uint8_t sw_num,bool state);
  void (*key_led_ctl)(uint16_t* key_flag);
  void (*key_led_on_off)(uint8_t sw_num,bool state);
  bool (*get_sleep_eable_flag)(void);
  uint16_t (*get_last_short_press_sys_time)(void);
  void (*set_key_press_event_flag)(KEY_PRESS_EVENT_LIST key_event);
  bool (*get_key_press_event_flag)(KEY_PRESS_EVENT_LIST key_event);
  void (*clr_key_press_event_flag)(KEY_PRESS_EVENT_LIST key_event);

  void (*set_sleep_enable_state)(bool state);
  void (*zero_cross_siganl_gpio_interrupt_set)(bool state);
  bool (*get_zero_cross_flag)(void);
  void (*set_zero_cross_flag)(bool state);
//void (*save_sw_cfg_to_flash)(void);
//void (*read_sw_cfg_from_flash)(void);

}SW_CTL_INFO;


extern SW_CTL_INFO  g_sw_ctl_info;
extern uint8_t g_sw0_state;
extern uint32_t enetr_zero_cross_interrupt_t1;
extern uint32_t test_zero_cross_flag_t2;
extern uint32_t before_ralay_control_t3;
extern uint32_t finish_ralay_control_t4;

void hal_set_ralay(uint8_t ralay_num);
void hal_clear_ralay(uint8_t ralay_num);
void hal_Toggle_ralay(uint8_t ralay_num);
void app_init(void);
void app_switch_on_or_off(uint8_t sw_num,bool state);

int8_t user_defined_cli_cmd_handler(uint8_t *cmd);
bool string_cmp(uint8_t* src_str,uint8_t* des_str,uint8_t str_len);
#endif /* SINGLE_FIRE_SWITCH_APP_H_ */
