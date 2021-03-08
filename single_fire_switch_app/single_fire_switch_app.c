/*
 * single_fire_switch_app.c
 *
 *  Created on: 2020年12月6日
 *      Author: zhinple
 */



#include "single_fire_switch_app.h"
#include "debug-printing.h"
#include "user_config.h"

SW_CTL_INFO  g_sw_ctl_info;

//EmberEventControl network_state_led_flash_event_control;

EmberEventControl sw0_relay_event_control;
EmberEventControl sw1_relay_event_control;
EmberEventControl sw2_relay_event_control;

static bool sleep_enable_state = false;
static uint16_t m_key_press_event_flag = 0;
static uint16_t m_last_key_press_time = 0;
static bool m_zero_cross_flag = false;
static const RALAY_ARRAY ralay_array[HAL_RALAY_COUNT] = BSP_RALAY_INIT;

static char *user_defined_cli_cmd_list[] =
{
  "cmd list",
  "get cfg",
  "set heartbeat period",
  "set switch power up state",
  "set sleep enable state",
};



void renew_last_short_press_sys_time(uint16_t sys_time)
{
      m_last_key_press_time = sys_time;
}


uint16_t get_last_short_press_sys_time_callb(void)
{
     return m_last_key_press_time;
}


void set_key_press_event_flag_callb(KEY_PRESS_EVENT_LIST key_event)
{
      m_key_press_event_flag|=(1<<key_event);
}


bool get_key_press_event_flag_callb(KEY_PRESS_EVENT_LIST key_event)
{
      return (m_key_press_event_flag&(1<<key_event));
}


void clr_key_press_event_flag_callb(KEY_PRESS_EVENT_LIST key_event)
{
     m_key_press_event_flag&=~(1<<key_event);
}



void set_sleep_enable_state_callb(bool state)
{
       sleep_enable_state  = state;
}


bool get_sleep_eable_flag_callb(void)
{
  return  sleep_enable_state;
}


extern uint32_t zero_test_t1_record;
void sw0_relay_event_handler(void)
{

       emberEventControlSetInactive(sw0_relay_event_control);
       renew_last_short_press_sys_time( halCommonGetInt16uMillisecondTick());
       SW_RALAY_END();
       clr_key_press_event_flag_callb(SW0_SHORT_PRESS);

         if((get_key_press_event_flag_callb(SW1_SHORT_PRESS))||\
            (get_key_press_event_flag_callb(SW2_SHORT_PRESS)))
         {
           return;
         }
         else
         {
             set_sleep_enable_state_callb(false);

         }

         finish_ralay_control_t4 = halCommonGetInt16uMillisecondTick();


         emberAfCorePrintln("enetr_zero_cross_interrupt_t1: %d",enetr_zero_cross_interrupt_t1);
         emberAfCorePrintln("      test_zero_cross_flag_t2: %d",test_zero_cross_flag_t2);
         emberAfCorePrintln("      before_ralay_control_t3: %d",before_ralay_control_t3);
         emberAfCorePrintln("      finish_ralay_control_t4: %d",finish_ralay_control_t4);

         emberAfCorePrintln("         delay_control_period: %d",before_ralay_control_t3-test_zero_cross_flag_t2);
         emberAfCorePrintln("         ralay_control_period: %d",finish_ralay_control_t4-test_zero_cross_flag_t2);
}

void sw1_relay_event_handler(void)
{

       emberEventControlSetInactive(sw1_relay_event_control);
       renew_last_short_press_sys_time(halCommonGetInt16uMillisecondTick());
       SW_RALAY_END();
       clr_key_press_event_flag_callb(SW1_SHORT_PRESS);

       if((get_key_press_event_flag_callb(SW0_SHORT_PRESS))||\
                  (get_key_press_event_flag_callb(SW2_SHORT_PRESS)))
       {
         return;
       }
       else
       {
           set_sleep_enable_state_callb(false);
          // emberAfCorePrintln("ralay onoff t2:%d", halCommonGetInt16uMillisecondTick());
          // emberAfCorePrintln(" delay period:%d",halCommonGetInt16uMillisecondTick()-zero_test_t1_record);
       }
}

void sw2_relay_event_handler(void)
{


       emberEventControlSetInactive(sw2_relay_event_control);
       renew_last_short_press_sys_time(halCommonGetInt16uMillisecondTick());
       SW_RALAY_END();
       clr_key_press_event_flag_callb(SW2_SHORT_PRESS);

       if((get_key_press_event_flag_callb(SW0_SHORT_PRESS))||\
                        (get_key_press_event_flag_callb(SW1_SHORT_PRESS)))
         {
           return;
         }
         else
         {
             set_sleep_enable_state_callb(false);
         }

       SW2_LED_ON();
}


void app_switch_on_or_off(uint8_t sw_num,bool state)
{
    //sleep_enable_flag = true;

   switch(sw_num)
   {
     case SW_0:
       relay_state_set(SW_0,state);
       break;
     case SW_1:
       relay_state_set(SW_1,state);
       break;
     case SW_2:
       relay_state_set(SW_2,state);
       break;
       default:
       break;

   }

}



void sw0_ralay_state_set(bool state)
{
  if (state == false)
     {
         SW0_RALAY_OFF();

     }else
     {
         SW0_RALAY_ON();
     }

    emberEventControlSetDelayMS(sw0_relay_event_control,RALAY_CONTROL_PROID_MS);
}

void sw1_ralay_state_set(bool state)
{
  if (state == false)
     {
         SW1_RALAY_OFF();

     }else
     {
         SW1_RALAY_ON();
     }

   emberEventControlSetDelayMS(sw1_relay_event_control,RALAY_CONTROL_PROID_MS);
}


void sw2_ralay_state_set(bool state)
{
  if (state == false)
     {
         SW2_RALAY_OFF();

     }else
     {
         SW2_RALAY_ON();
     }

  emberEventControlSetDelayMS(sw2_relay_event_control,RALAY_CONTROL_PROID_MS);
}




void relay_state_set(uint8_t sw_num,bool state)
{
    switch(sw_num)
     {
       case SW_0:
         sw0_ralay_state_set(state);
         break;
       case SW_1:
         sw1_ralay_state_set(state);
         break;
       case SW_2:
         sw2_ralay_state_set(state);
         break;
       default:break;
     }

}


void key_led_ctl_callb(uint16_t* key_flag)
{
     EmberNetworkStatus state = emberAfNetworkState();
     static uint8_t led_flash_cnt = 0;
     const uint8_t  LED_FLASH_MAX_CNT = 2;
     static bool led_state = ON;

     if(*key_flag&0x07)
       {

           if (state != EMBER_JOINED_NETWORK)
              {

                    if(led_flash_cnt<LED_FLASH_MAX_CNT)
                      {
                        if(led_state == ON)
                          {
                            key_led_on_off_callb(SW_0,OFF);
                             led_state = OFF;
                          }
                        else
                          {
                            key_led_on_off_callb(SW_0,ON);
                            led_state = ON;
                            led_flash_cnt++;
                          }

                      }
                    else
                      {
                        *key_flag = 0;
                        led_flash_cnt = 0;
                        led_state = ON;
                      }

              }

             else
                 {
                     *key_flag = 0;
                 }


       }

}



void key_led_on_off_callb(uint8_t sw_num,bool state)
{


     switch(sw_num)
       {
         case SW_0:
           if(state)
             {

                 SW0_LED_ON();
             }
           else
             {

                SW0_LED_OFF();
             }
           break;
         case SW_1:
           if(state)
            {

                SW1_LED_ON();

            }
          else
            {

               SW1_LED_OFF();
            }
           break;
         case SW_2:
           if(state)
            {

                SW2_LED_ON();
            }
          else
            {

               SW2_LED_OFF();
            }
           break;
           default:break;
       }
}


extern uint32_t interrupt_t0_record;
extern uint16_t ingterrupt_cnt;
void zero_cross_siganl_isr(void)
{
    set_zero_cross_flag_callb(true);

  // GPIO_IntDisable(1<<ZERO_CROSS_SIGNAL_PIN);
   //emberAfCorePrintln("zero_cross_siganl_isr");
   //enetr_zero_cross_interrupt_t1 = halCommonGetInt16uMillisecondTick();

}

bool get_zero_cross_flag_callb(void)
{
     return m_zero_cross_flag;
}


void set_zero_cross_flag_callb(bool state)
{
  m_zero_cross_flag = state;
}


void zero_cross_siganl_gpio_init(void)
{
      GPIOINT_Init();
      GPIO_PinModeSet(ZERO_CROSS_SIGNAL_PORT,
                     ZERO_CROSS_SIGNAL_PIN,
                     ZERO_CROSS_SIGNAL_GPIO_MODE,
                     ZERO_CROSS_SIGNAL_GPIO_DOUT);
      /* Register callbacks before setting up and enabling pin interrupt. */
      GPIOINT_CallbackRegister(ZERO_CROSS_SIGNAL_PIN,
                               zero_cross_siganl_isr);
      /* Set rising and falling edge interrupts */
      GPIO_ExtIntConfig(ZERO_CROSS_SIGNAL_PORT,
                        ZERO_CROSS_SIGNAL_PIN,
                        ZERO_CROSS_SIGNAL_PIN,
                        false,
                        true,
                        true);
      GPIO_IntDisable(1<<ZERO_CROSS_SIGNAL_PIN);

}

void zero_cross_siganl_gpio_interrupt_set_callb(bool state)
{
   if(state == true)
     {
       GPIO_IntEnable(1<<ZERO_CROSS_SIGNAL_PIN);
     }
   else
     {
       GPIO_IntDisable(1<<ZERO_CROSS_SIGNAL_PIN);

     }
}



void hal_set_ralay(uint8_t ralay_num)
{
#if defined (BSP_RALAY_POLARITY) && (BSP_RALAY_POLARITY == 0)
  GPIO_PinOutClear(ralay_array[ralay_num].port, ralay_array[ralay_num].pin);
#else
  GPIO_PinOutSet(ralay_array[ralay_num].port, ralay_array[ralay_num].pin);
#endif
}

void hal_clear_ralay(uint8_t ralay_num)
{
#if defined (BSP_RALAY_POLARITY) && (BSP_RALAY_POLARITY == 0)
  GPIO_PinOutSet(ralay_array[ralay_num].port, ralay_array[ralay_num].pin);
#else
  GPIO_PinOutClear(ralay_array[ralay_num].port, ralay_array[ralay_num].pin);
#endif
}

void hal_Toggle_ralay(uint8_t ralay_num)
{
  GPIO_PinOutToggle(ralay_array[ralay_num].port, ralay_array[ralay_num].pin);
}



void ralay_ctl_gpio_init(void)
{

    CMU_ClockEnable(cmuClock_GPIO, true);
    for ( uint8_t i = 0; i < BSP_RALAY_COUNT; i++ )
      {
         GPIO_PinModeSet(ralay_array[i].port,
                        ralay_array[i].pin,
                        gpioModePushPull ,
                          0);
        //gpioModePushPull
      }

    for ( uint8_t i = RALAY_ON1; i < RALAY_MAX; i++ )
      {
        hal_clear_ralay(i);
      }

    GPIO_PinModeSet(gpioPortA,3,gpioModePushPull ,0);
    //GPIO_PinOutSet(gpioPortA,3);
   // GPIO_PinOutClear(gpioPortA,3);


}

int8_t get_user_defined_cli_cmd_index( uint8_t *cmd)
{
      uint8_t cmd_len = strlen(cmd);

     if(cmd==NULL)
       {
         return PARA_POINT_ERR;
       }
      if(cmd_len == 0)
        {
          return  CMD_BYTES_LEN_ERR;
        }
      for(uint8_t i = 0;i<cmd_len;i++)
       {

          if((*cmd>=97)&&(*cmd<=122))//'a'~'z'
            {
              break;
            }
             cmd++;
       }


      for(uint8_t i = 0;i<CLI_CMD_MAX_INDEX;i++)
        {

         if (string_cmp(cmd,user_defined_cli_cmd_list[i],strlen(user_defined_cli_cmd_list[i])))
           {
              return i;
           }

        }
     return CLI_CMD_ERR;
}

bool string_cmp(uint8_t* src_str,uint8_t* des_str,uint8_t str_len)
{

     //  emberAfCorePrintln("str_len: %d\r\n",str_len);

       if((src_str==NULL)||(des_str==NULL))
       {
         return PARA_POINT_ERR;
       }
        if(str_len == 0)
        {
          return  CMD_BYTES_LEN_ERR;
        }

    for(uint8_t i = 0;i<str_len;i++)
    {
       if(src_str[i]!=des_str[i])
         {
            return false;
         }
    }
    return true;
}


int8_t user_defined_cli_cmd_handler(uint8_t *cmd)
{

  uint8_t cmd_len = strlen(cmd);
  uint16_t heartbeat_period = 0;
  uint8_t *p = NULL;
  bool sw_init_state ;
  bool sleep_enable_state;

  int8_t rtn_err = CLI_CMD_ERR;

    if(cmd==NULL)
      {
        return PARA_POINT_ERR;
      }
     if(cmd_len == 0)
       {
         return  CMD_BYTES_LEN_ERR;
       }
     for(uint8_t i = 0;i<cmd_len;i++)
      {

         if((*cmd>=97)&&(*cmd<=122))//'a'~'z'
           {
             break;
           }
            cmd++;
      }

     for(uint8_t i = 0;i<CLI_CMD_MAX_INDEX;i++)
       {

        if (string_cmp(cmd,user_defined_cli_cmd_list[i],strlen(user_defined_cli_cmd_list[i])))
          {
               switch(i)
               {
                 case CMD_LIST:
                   rtn_err = NO_ERR;
                 break;
                 case GET_CFG:
                   emberAfCorePrintln("sw0_state: %d", g_sw_ctl_info.sw0_state);
                   emberAfCorePrintln("sw1_state: %d", g_sw_ctl_info.sw1_state);
                   emberAfCorePrintln("sw2_state: %d", g_sw_ctl_info.sw2_state);
                   emberAfCorePrintln("heartbeat_period: %d", g_sw_ctl_info.heartbeat_period);
                   emberAfCorePrintln("switch_power_up_state: %d", g_sw_ctl_info.switch_power_up_state);
                   emberAfCorePrintln("sleep_eable_flag: %d", g_sw_ctl_info.get_sleep_eable_flag());
                   rtn_err = NO_ERR;
                   break;
                  case SET_HEARTBEAT_PERIOD:

                    p = cmd+strlen(user_defined_cli_cmd_list[SET_HEARTBEAT_PERIOD])+1;
                    uint16_t t1 = (uint16_t)(p[0]-0x30)<<12;
                    uint16_t t2 = (uint16_t)(p[1]-0x30)<<8;
                    uint16_t t3= (uint16_t)(p[2]-0x30)<<4;
                    uint16_t t4= (uint16_t)(p[3]-0x30);
                    heartbeat_period = t1+t2+t3+t4;
                    emberAfCorePrintln("heartbeat_period: %d s",heartbeat_period);
                    g_sw_ctl_info.heartbeat_period = heartbeat_period;
                    save_sw_cfg_to_flash();
                    rtn_err = NO_ERR;
                  break;
                  case SET_SWITCH_INIT_STATE:
                     p = cmd+strlen(user_defined_cli_cmd_list[SET_SWITCH_INIT_STATE])+1;
                     sw_init_state= (p[0]-0x30);
                     emberAfCorePrintln("sw_init_state: %d",sw_init_state);
                     g_sw_ctl_info.switch_power_up_state = sw_init_state;
                     save_sw_cfg_to_flash();
                     rtn_err = NO_ERR;
                  break;

                  case SET_SLEEP_ENABLE_STATE:
                    p = cmd+strlen(user_defined_cli_cmd_list[SET_SLEEP_ENABLE_STATE])+1;

                    sleep_enable_state= (p[0]-0x30);
                    emberAfCorePrintln("sleep_enable_state: %d",sleep_enable_state);
                    g_sw_ctl_info.set_sleep_enable_state(sleep_enable_state) ;
                   // save_sw_cfg_to_flash();
                    rtn_err = NO_ERR;
                 break;


                  default:
                  break;

               }
          }


       }

         return rtn_err;

}


void app_init(void)
{

   g_sw_ctl_info.switch_power_up_state = SW_POWER_UP_STATE;
   g_sw_ctl_info.heartbeat_period = HEARTBEAT_PRIOD;


  g_sw_ctl_info.key_press_event_flag = 0;
  g_sw_ctl_info.network_link_cnt = 0;
  g_sw_ctl_info.network_leave_time = 0;
  g_sw_ctl_info.network_link_time = 0;
  g_sw_ctl_info.sw_ctl_callb = app_switch_on_or_off;
  g_sw_ctl_info.get_sleep_eable_flag = get_sleep_eable_flag_callb;
  g_sw_ctl_info.key_led_ctl = key_led_ctl_callb;
  g_sw_ctl_info.key_led_on_off = key_led_on_off_callb;

  g_sw_ctl_info.get_last_short_press_sys_time = get_last_short_press_sys_time_callb;
  g_sw_ctl_info.set_key_press_event_flag = set_key_press_event_flag_callb;
  g_sw_ctl_info.get_key_press_event_flag = get_key_press_event_flag_callb;
  g_sw_ctl_info.clr_key_press_event_flag = clr_key_press_event_flag_callb;


  g_sw_ctl_info.set_sleep_enable_state = set_sleep_enable_state_callb;
  g_sw_ctl_info.get_sleep_eable_flag  = get_sleep_eable_flag_callb;
  g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set = zero_cross_siganl_gpio_interrupt_set_callb;
  g_sw_ctl_info.get_zero_cross_flag = get_zero_cross_flag_callb;
  g_sw_ctl_info.set_zero_cross_flag = set_zero_cross_flag_callb;


  zero_cross_siganl_gpio_init();
  ralay_ctl_gpio_init();

  //GPIO_DbgSWOEnable(false);//禁止SWO功能,设置SWO管脚为普通gpio功能
  //GPIO_PinModeSet(BSP_LED3_PORT,BSP_LED3_PIN,gpioModePushPull,0);

 g_sw_ctl_info.key_led_on_off(SW_0,ON);
 g_sw_ctl_info.key_led_on_off(SW_1,ON);
 g_sw_ctl_info.key_led_on_off(SW_2,ON);

}




