/***************************************************************************//**
 * @file
 * @brief Callback implementation for ZigbeeMinimal sample application.
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// This callback file is created for your convenience. You may add application
// code to this file. If you regenerate this file over a previous version, the
// previous version will be overwritten and any code you have added will be
// lost.

#include "app/framework/include/af.h"
#include "single_fire_switch_app/single_fire_switch_app.h"
#include "app/framework/plugin/network-steering/network-steering.h"
#include "app/framework/plugin/network-steering/network-steering-internal.h"
#include "custom-token.h"
#include "plugin/serial/com_config.h"
#include "debug-printing.h"

#include "stack/include/mfglib.h" // Required for packetHandlers
/** @brief Stack Status
 *
 * This function is called by the application framework from the stack status
 * handler.  This callbacks provides applications an opportunity to be notified
 * of changes to the stack status and take appropriate action.  The return code
 * from this callback is ignored by the framework.  The framework will always
 * process the stack status after the callback returns.
 *
 * @param status   Ver.: always
 */


extern EmberEventControl emberAfPluginEndDeviceSupportMoveNetworkEventControls[];

EmberEventControl network_link_event_control;
EmberEventControl reset_rejoin_event_control;
EmberEventControl channel_scan_envent_control;
EmberEventControl long_press_network_link_event_control;

EmberEventControl sw0_press_short_event_control;
EmberEventControl sw1_press_short_event_control;
EmberEventControl sw2_press_short_event_control;

EmberEventControl sw0_press_low_event_control;
EmberEventControl sw1_press_low_event_control;
EmberEventControl sw2_press_low_event_control;

EmberEventControl power_up_relay_off_event_control;
EmberEventControl heartbeat_event_control;
EmberEventControl network_up_renew_sw_state_event_control;
uint8_t g_radioChannel = 255;
uint8_t g_resetRejoinRetry = 0;
uint8_t g_key_press_flag = 0;
uint8_t key_led_conflict_flag = 0;

uint32_t enetr_zero_cross_interrupt_t1;
uint32_t test_zero_cross_flag_t2;
uint32_t before_ralay_control_t3;
uint32_t finish_ralay_control_t4;

#define RALAY_USE_TEST 0
#define ZERO_CROSS_TEST 0

bool emberAfStackStatusCallback(EmberStatus status)
{
  // This value is ignored by the framework.
  //emberSetChildPower(0,0);
  return false;
}


void mfglibRxCallback(uint8_t *packet, uint8_t linkQuality, int8_t rssi)
{

//    emberAfCorePrintln("mfglibRxCallback");
//    emberAfCorePrintln("rssi:%d",rssi);
//    emberAfCorePrintln("linkQuality:%d",linkQuality);
//    //emberAfCorePrintln("packet:%s",packet);
//    emberAfCorePrintln("len:%d",strlen(packet));
//    for(uint16_t i =0;i<strlen(packet);i++)
//      {
//        emberAfCorePrintln("%x",packet[i]);
//      }
//

 }

/** @brief Complete
 *
 * This callback is fired when the Network Steering plugin is complete.
 *
 * @param status On success this will be set to EMBER_SUCCESS to indicate a
 * network was joined successfully. On failure this will be the status code of
 * the last join or scan attempt. Ver.: always
 * @param totalBeacons The total number of 802.15.4 beacons that were heard,
 * including beacons from different devices with the same PAN ID. Ver.: always
 * @param joinAttempts The number of join attempts that were made to get onto
 * an open Zigbee network. Ver.: always
 * @param finalState The finishing state of the network steering process. From
 * this, one is able to tell on which channel mask and with which key the
 * process was complete. Ver.: always
 */

void heartbeat_event_handler(void)
{

#if 0
     emberEventControlSetInactive(heartbeat_event_control);
     EmberStatus status;
     uint8_t buff[10] = {0};
     uint8_t len = 0;

     emberAfCorePrintln("heartbeat_event_handler");

      EmberNetworkStatus state = emberAfNetworkState();
     if (state!= EMBER_JOINED_NETWORK)
       {
         return;
       }

      buff[len++] = 0;
      buff[len++] = 0;
      buff[len++] = 0x10;
      buff[len++] = g_sw_ctl_info.sw0_state;
      emberAfFillCommandGlobalServerToClientReportAttributes(ZCL_ON_OFF_CLUSTER_ID, (uint8_t *) buff, len);
      emberAfSetCommandEndpoints(1,1);
      status=emberAfSendCommandUnicast(EMBER_OUTGOING_DIRECT, 0x0000);
       if(status != EMBER_SUCCESS){
          emberAfCorePrintln("heratbeat Failed to send");
       }
        else
       {
          emberAfCorePrintln("heratbeat sucessful to send");

       }
       emberEventControlSetDelayQS(heartbeat_event_control, g_sw_ctl_info.heartbeat_period*4);

#endif

}

//| Attribute ID | Data type | Attribute data |......|
void renew_sw_state_to_manager(uint8_t sw_num,bool sw_state)
{


#if 0
         EmberStatus status;
         uint8_t buff[10] = {0};
         uint8_t len = 0;

          EmberNetworkStatus state = emberAfNetworkState();
          if (state!= EMBER_JOINED_NETWORK)
            {
              return;
            }

         buff[len++] = 0;
         buff[len++] = 0;
         buff[len++] = 0x10;
         buff[len++] = sw_state;
         emberAfFillCommandGlobalServerToClientReportAttributes(ZCL_ON_OFF_CLUSTER_ID, (uint8_t *) buff, len);
         emberAfSetCommandEndpoints(sw_num+1,1);
         status=emberAfSendCommandUnicast(EMBER_OUTGOING_DIRECT, 0x0000);
         if(status == EMBER_SUCCESS){
           emberAfCorePrintln("Command is successfully sent");
         }else{
           emberAfCorePrintln("Failed to send");
           emberAfCorePrintln("Status code: 0x%x",status);
         }

#endif

}


void renew_model_id_to_manager(void)
{
//     EmberStatus status;
//     uint8_t buff[50] = {0};
//     uint8_t len = 0;
//     const uint8_t* p_model_id = "sg-fire-sw-3";
//
//
//      EmberNetworkStatus state = emberAfNetworkState();
//      if (state!= EMBER_JOINED_NETWORK)
//        {
//          return;
//        }
//
//     buff[len++] = 0x5;
//     buff[len++] = 0;
//     buff[len++] = 0x42;
//     buff[len++] = strlen(p_model_id);
//     strcpy(&buff[len],p_model_id);
//     len+=strlen(p_model_id);
//     emberAfFillCommandGlobalServerToClientReportAttributes(ZCL_BASIC_CLUSTER_ID, (uint8_t *) buff, len);
//     emberAfSetCommandEndpoints(1,1);
//     status=emberAfSendCommandUnicast(EMBER_OUTGOING_DIRECT, 0x0000);
//     if(status == EMBER_SUCCESS){
//       emberAfCorePrintln("Command is successfully sent");
//     }else{
//       emberAfCorePrintln("Failed to send");
//       emberAfCorePrintln("Status code: 0x%x",status);
//     }

}




void power_up_relay_off_event_handler(void)
{
//    static uint8_t step = 0;
//    static uint8_t time_cnt = 0;
//    uint8_t test_data[10]={1,2,3,4,5,6,7,8,9};
//    static uint8_t mfg_mode_step = 0;
//   emberEventControlSetInactive(power_up_relay_off_event_control);
//  // emberAfCorePrintln("power_up_relay_off_event_handler");
//   EmberStatus  ststa ;
//#ifndef ZERO_CROSS_TEST
//
//#if RALAY_USE_TEST
//   emberEventControlSetDelayMS(power_up_relay_off_event_control,330);
//
//   if(time_cnt++<3) return;
//
//
//   switch(step)
//   {
//
//     case 0:
//     step  = 1;
//     emberEventControlSetActive(sw0_press_short_event_control);
//     break;
//     case 1:
//     step  = 2;
//     emberEventControlSetActive(sw1_press_short_event_control);
//     break;
//     case 2:
//     step  = 0;
//     emberEventControlSetActive(sw2_press_short_event_control);
//     time_cnt = 0;
//     break;
//
//   }
//#else
////   emberEventControlSetDelayMS(sw0_press_short_event_control,100);
////   emberEventControlSetDelayMS(sw1_press_short_event_control,1100);
////   emberEventControlSetDelayMS(sw2_press_short_event_control,2100);
//
//#endif
//
//#else
//
//   emberEventControlSetActive(sw0_press_short_event_control);
//#endif
//
//   switch (mfg_mode_step)
//   {
//     case 0:
//       ststa = mfglibStart(mfglibRxCallback);
//
//           if(ststa == EMBER_SUCCESS)
//            {
//              emberAfCorePrintln("mfglibStart ok");
//
//            }
//          else
//            {
//              emberAfCorePrintln("mfglibStart fail");
//            }
//
//       mfg_mode_step = 1;
//     break;
//     case 1:
//
//       ststa = mfglibSetChannel(12);
//
//      if(ststa == EMBER_SUCCESS)
//        {
//          emberAfCorePrintln("mfglibSetChannel ok");
//        }
//      else
//        {
//          emberAfCorePrintln("mfglibSetChannel fail");
//        }
//
//       mfg_mode_step = 2;
//     break;
//
//     case 2:
//
//       ststa = mfglibSendPacket(test_data,0);
//
//                 if(ststa == EMBER_SUCCESS)
//                   {
//                     emberAfCorePrintln("mfglibSendPacket ok");
//                   }
//                 else
//                   {
//                     emberAfCorePrintln("mfglibSendPacket fail");
//                   }
//
//     break;
//
//
//   }
//
//   emberEventControlSetDelayMS(power_up_relay_off_event_control,2000);
}


void network_link_event_handler(void)
{
//  emberEventControlSetInactive(network_link_event_control);
//
//  static uint8_t poll_time_cnt = 0;
//  const uint8_t POLL_PERIOD = 4;//4*250ms = 1s
//  uint8_t j;
//  if(poll_time_cnt++<POLL_PERIOD)
//    {
//       return;
//    }
//  poll_time_cnt = 0;
//
//  EmberNetworkStatus state = emberAfNetworkState();
// if (state == EMBER_JOINED_NETWORK)
//  {
//    // emberAfCorePrintln("PollForData");
//       g_sw_ctl_info.network_link_cnt = 0;
//       EmberStatus status = emberPollForData();
//       if (status != EMBER_SUCCESS) {
//        emberAfCorePrintln("poll nwk %d: 0x%x", emberGetCurrentNetwork(), status);
//      }
//  }

}


void reset_rejoin_event_handler(void)
{
       EmberStatus status;
       tokTypeStackNodeData tok;

    //   emberAfCorePrintln("reset_rejoin_event_handler,line236");
       emberEventControlSetInactive(reset_rejoin_event_control);

       EmberNetworkStatus state = emberAfNetworkState();

       if (state == EMBER_JOINED_NETWORK)
            {
                if(g_sw_ctl_info.key_press_event_flag&(1<<LOST_PRAENT_REJOIN_NETWORK))
                  {
        //            emberAfCorePrintln("reset_rejoin_event_handler,line245");
                    g_sw_ctl_info.key_press_event_flag&=~(1<<LOST_PRAENT_REJOIN_NETWORK);
                     network_up_renew_sw_state_event_handler();//重连上后同步开关状态到网关
                     return;
                  }

            }
        else if (state == EMBER_JOINED_NETWORK_NO_PARENT)
          {
          //  emberAfCorePrintln("reset_rejoin_event_handler,line253");
            g_sw_ctl_info.key_press_event_flag|=(1<<LOST_PRAENT_REJOIN_NETWORK);

          }

       emberEventControlSetDelayMS(reset_rejoin_event_control,1000);


#if 0

       halCommonGetToken(&tok, TOKEN_STACK_NODE_DATA);
       if (tok.panId <0xFFFE) {
       emberAfCorePrintln("channel =%d", tok.radioFreqChannel);
       emberAfCorePrintln("panId = %2x", tok.panId);
       emberAfCorePrintln("radioTxPower =%d", tok.radioTxPower);
       emberAfCorePrintln("NodeId = %2x", tok.zigbeeNodeId);
       g_radioChannel = tok.radioFreqChannel;

       }
       else
        {
            return;
        }


      uint32_t chnlmask = 1 << g_radioChannel;

      emberAfCorePrintln("reset_rejoin_event_handler");

    EmberNetworkStatus state = emberAfNetworkState();
    emberAfCorePrintln("state: %d",state);

   if (state == EMBER_JOINED_NETWORK)
     {
         g_resetRejoinRetry = 0;
         return;
     }

      else if (state == EMBER_JOINED_NETWORK_NO_PARENT) {
      if (!emberAfPluginEndDeviceSupportLostParentConnectivityCallback())
      {

          emberAfCorePrintln("emberFindAndRejoinNetworkWithReason");
          status = emberFindAndRejoinNetworkWithReason(1, // Network Key is known
                                                      chnlmask,
                                                       EMBER_REJOIN_DUE_TO_END_DEVICE_REBOOT);

         if (status != EMBER_SUCCESS) {

              emberAfCorePrintln("start rejoin fail, status=0x%X chnlmask=0x%4X", status, chnlmask);

             }

     }


   }


#endif

}




int8_t lastHopRssi = 0;
void emberAfPluginNetworkSteeringCompleteCallback(EmberStatus status,
                                                  uint8_t totalBeacons,
                                                  uint8_t joinAttempts,
                                                  uint8_t finalState)
{

  emberAfCorePrintln("%p network %p: 0x%X", "Join", "complete", status);
  emberGetLastHopRssi(&lastHopRssi);
  emberAfCorePrintln("lastHopRssi:%d",lastHopRssi);
}

void emberAfPluginButtonInterfaceButton2PressedLongCallback(uint16_t timePressedMs,
                                                            bool pressedAtReset)

{
  emberAfCorePrintln("Button2PressedLong");
  g_sw_ctl_info.key_led_on_off(SW_2,ON);
}


void emberAfPluginButtonInterfaceButton2PressedShortCallback(uint16_t timePressedMs)
{

  static uint16_t  last_press_time = 0;
  static uint16_t  current_press_time = 0;
  const  uint16_t  KEY_VALID_MIN_INTERVAL = 500;//500ms

    if(timePressedMs<20)//消抖
     {
       return;
     }


  current_press_time = halCommonGetInt16uMillisecondTick();

  g_sw_ctl_info.key_led_on_off(SW_2,ON);

 // emberAfCorePrintln("key press interval：%d",current_press_time-last_press_time);

    if(current_press_time>last_press_time)
    {
       if(current_press_time<last_press_time+KEY_VALID_MIN_INTERVAL)
         {
           last_press_time = current_press_time;
           return;
         }
    }
    else
    {
        if((65535-last_press_time+current_press_time)<KEY_VALID_MIN_INTERVAL)
          {
            last_press_time = current_press_time;
             return;

          }

    }

   emberAfCorePrintln("Button2PresseShort");
   last_press_time = current_press_time;

   //g_sw_ctl_info.key_led_on_off(SW_2,ON);
   g_sw_ctl_info.set_key_press_event_flag(SW2_PRESS);
   emberEventControlSetActive(sw2_press_short_event_control);

}



void sw2_press_short_event_handler(void)
{
     uint16_t current_sys_time = halCommonGetInt16uMillisecondTick();
     uint16_t last_short_press_sys_time =  g_sw_ctl_info.get_last_short_press_sys_time();
     uint16_t delay_ms;

    static uint8_t sw2_process_step = STEP_KEYS_EVENT_TEST;
    static uint8_t zero_cross_test_timeout = 0;
    emberEventControlSetInactive(sw2_press_short_event_control);
    switch(sw2_process_step)
    {
      case STEP_KEYS_EVENT_TEST:
    //    emberAfCorePrintln("key2 STEP_KEYS_EVENT_TEST");
         if((g_sw_ctl_info.get_key_press_event_flag(SW0_SHORT_PRESS))||\
           (g_sw_ctl_info.get_key_press_event_flag(SW1_SHORT_PRESS)))
         {
            emberEventControlSetDelayMS(sw2_press_short_event_control,10);
           return;
         }
         sw2_process_step = STEP_KEYS_INTERVAL;
         emberEventControlSetActive(sw2_press_short_event_control);

        break;

      case STEP_KEYS_INTERVAL:
    //   emberAfCorePrintln("key2 STEP_KEYS_INTERVAL");
         g_sw_ctl_info.set_key_press_event_flag(SW2_SHORT_PRESS);
         g_sw_ctl_info.set_sleep_enable_state(true);


          if(current_sys_time>last_short_press_sys_time)
          {
             if(current_sys_time<last_short_press_sys_time+KEYS_INTERVAL)
               {

                 delay_ms = KEYS_INTERVAL-(current_sys_time-last_short_press_sys_time)+10;
                // emberAfCorePrintln("key delay_ms1:%d ",delay_ms);
                 emberEventControlSetDelayMS(sw2_press_short_event_control,delay_ms);
                 return;
               }
          }
          else
          {
              if((65535-last_short_press_sys_time+current_sys_time)<KEYS_INTERVAL)
                {

                  delay_ms = KEYS_INTERVAL -(65535-last_short_press_sys_time+current_sys_time)+10;
                 // emberAfCorePrintln("sw2 delay_ms2:%d ",delay_ms);
                  emberEventControlSetDelayMS(sw2_press_short_event_control,delay_ms);
                   return;
                }
          }

          if( g_sw_ctl_info.sw2_state == OFF)
            {
              sw2_process_step = STEP_ZERO_CROSS_TEST;
              g_sw_ctl_info.set_zero_cross_flag(false);
              g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(true);//开启中断
              emberEventControlSetActive(sw2_press_short_event_control);
            }else
              {
                sw2_process_step = STEP_SW_ON_OFF;
               emberEventControlSetActive(sw2_press_short_event_control);
              }

        break;
      case STEP_ZERO_CROSS_TEST:
    //   emberAfCorePrintln("key2 STEP_ZERO_CROSS_TEST");

#if 1
        if(g_sw_ctl_info.get_zero_cross_flag())
        {
          g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
          g_sw_ctl_info.set_zero_cross_flag(false);

          emberEventControlSetDelayMS(sw2_press_short_event_control,15);
          sw2_process_step = STEP_SW_ON_OFF;

          return;
        }
        else
         {

  //          emberEventControlSetDelayMS(sw2_press_short_event_control,10);


            if(zero_cross_test_timeout++<20)
             {
               emberEventControlSetDelayMS(sw2_press_short_event_control,1);

             }else //过零检测超时
             {
                 zero_cross_test_timeout = 0;
                 g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
                 g_sw_ctl_info.set_zero_cross_flag(false);

                 sw2_process_step = STEP_SW_ON_OFF;
                 emberEventControlSetActive(sw2_press_short_event_control);
                 emberAfCorePrintln("sw2 zero_cross_test_timeout");
             }

            return;
         }
#else
         sw2_process_step = STEP_SW_ON_OFF;
         emberEventControlSetActive(sw2_press_short_event_control);
#endif
       break;

      case STEP_SW_ON_OFF:

        emberAfCorePrintln("SW2 STEP_SW_ON_OFF");

        if(g_sw_ctl_info.sw2_state == false)
        {
           g_sw_ctl_info.sw_ctl_callb(SW_2,true);
           g_sw_ctl_info.sw2_state = true;
        }
      else
        {
           g_sw_ctl_info.sw_ctl_callb(SW_2,false);
           g_sw_ctl_info.sw2_state = false;
        }

        if (g_sw_ctl_info.get_key_press_event_flag(SW2_PRESS))//解决非按键按下指示灯闪的问题
          {
              g_sw_ctl_info.clr_key_press_event_flag(SW2_PRESS);
              g_key_press_flag |=(1<<SW_2 ) ;
          }

       //  save_sw_cfg_to_flash();

         renew_sw_state_to_manager(SW_2, g_sw_ctl_info.sw2_state);
         sw2_process_step = STEP_KEYS_EVENT_TEST;
         zero_cross_test_timeout = 0;

//         EmberStatus ststa =  mfglibEnd();
//         if(ststa == EMBER_SUCCESS)
//          {
//            emberAfCorePrintln("mfglibEnd ok");
//          }
//        else
//          {
//            emberAfCorePrintln("mfglibEnd fail");
//          }

       break;


    }

}


void emberAfPluginButtonInterfaceButton0PressedLongCallback(uint16_t timePressedMs,
                                                            bool pressedAtReset)
{
  emberAfCorePrintln("Button0PressedLong");
  g_sw_ctl_info.key_led_on_off(SW_0,ON);
  // networkLeaveCommand();//离网
}

void emberAfPluginButtonInterfaceButton1PressedLongCallback(uint16_t timePressedMs,
                                                            bool pressedAtReset)
{
  emberAfCorePrintln("Button1PressedLong");
  g_sw_ctl_info.key_led_on_off(SW_1,ON);
}

sl_sleeptimer_timer_handle_t sleep_timer_handle;
//uint8_t test_data = 1;



void sleep_timer_callb(sl_sleeptimer_timer_handle_t  *handle, void *data)
{
  // emberAfCorePrintln("sleep_timer_callb");
  //emberAfCorePrintln("StackTasks:%d",emberCurrentStackTasks());
  // emberAfCorePrintln("sleep_eable_flag:%d",g_sw_ctl_info.get_sleep_eable_flag());
 // COM_WriteData(COM_USART0,test_data,1);
#ifndef ZERO_CROSS_TEST
  emberEventControlSetActive(long_press_network_link_event_control);
  emberEventControlSetActive(network_link_event_control);
  g_sw_ctl_info.key_led_ctl(&g_key_press_flag);
#endif


}

// Non-volatile Data Storage: Step 1
sw_On_off_status_t sw_on_off_state;


void emberAfMainInitCallback(void)
 {

   app_init();
   sl_sleeptimer_init();
   sl_sleeptimer_start_periodic_timer(&sleep_timer_handle,8191,sleep_timer_callb,0,0,1);
   //rtc时钟3.2767khz,定时250ms,定时器预寄存器值设置为32767/4 = 8191.

   read_sw_cfg_from_flash();
#ifndef ZERO_CROSS_TEST
   emberEventControlSetDelayQS(heartbeat_event_control, g_sw_ctl_info.heartbeat_period*4);
   emberEventControlSetActive(reset_rejoin_event_control);
#endif
//   emberEventControlSetActive(power_up_relay_off_event_control);
   emberEventControlSetDelayMS(power_up_relay_off_event_control,2000);
//   EmberStatus state = emberSetRadioPower(10);
//   if(state == EMBER_SUCCESS)
//     {
//       emberAfCorePrintln("emberSetRadioPower set ok");
//     }
//   else
//     {
//       emberAfCorePrintln("emberSetRadioPower：state:%d",state);
//     }
//   emberAfCorePrintln("RadioPower:%d",emberGetRadioPower());

}


void save_sw_cfg_to_flash(void)
{

  sw_on_off_state.heartbeat_period =  g_sw_ctl_info.heartbeat_period ;
  sw_on_off_state.switch_power_up_state = g_sw_ctl_info.switch_power_up_state ;
  halCommonSetToken(TOKEN_SW_ON_OFF, &sw_on_off_state);

}
void mfglibRxCallback(uint8_t *packet, uint8_t linkQuality, int8_t rssi);
void read_sw_cfg_from_flash(void)
{

  halCommonGetToken(&sw_on_off_state, TOKEN_SW_ON_OFF);

   g_sw_ctl_info.heartbeat_period = sw_on_off_state.heartbeat_period;
   g_sw_ctl_info.switch_power_up_state = sw_on_off_state.switch_power_up_state;


  if(g_sw_ctl_info.heartbeat_period == 0)
     {
        g_sw_ctl_info.heartbeat_period = HEARTBEAT_PRIOD;
        g_sw_ctl_info.switch_power_up_state = SW_POWER_UP_STATE;
        g_sw_ctl_info.set_sleep_enable_state(DEFAULT_SLEEP_STATE);
        save_sw_cfg_to_flash();
     }


   if(g_sw_ctl_info.switch_power_up_state == ON)
     {
       g_sw_ctl_info.sw0_state = OFF;
       g_sw_ctl_info.sw1_state = OFF;
       g_sw_ctl_info.sw2_state = OFF;
     }
   else
     {
        g_sw_ctl_info.sw0_state = ON;
        g_sw_ctl_info.sw1_state = ON;
        g_sw_ctl_info.sw2_state = ON;
     }

   emberAfCorePrintln("heartbeat_period:%d",sw_on_off_state.heartbeat_period);
   emberAfCorePrintln("switch_power_up_state:%d",sw_on_off_state.switch_power_up_state);






}



void emberAfPluginButtonInterfaceButton0PressedShortCallback(uint16_t timePressedMs)
{

      static uint16_t  last_press_time = 0;
      static uint16_t  current_press_time = 0;
      const  uint16_t  KEY_VALID_MIN_INTERVAL = 500;//500ms
          if(timePressedMs<20)//消抖
            {
              return;
            }

      current_press_time = halCommonGetInt16uMillisecondTick();
      g_sw_ctl_info.key_led_on_off(SW_0,ON);

     // emberAfCorePrintln("key press interval：%d",current_press_time-last_press_time);

        if(current_press_time>last_press_time)
        {
           if(current_press_time<last_press_time+KEY_VALID_MIN_INTERVAL)
             {
               last_press_time = current_press_time;
               return;
             }
        }
        else
        {
            if((65535-last_press_time+current_press_time)<KEY_VALID_MIN_INTERVAL)
              {
                last_press_time = current_press_time;
                 return;

              }

        }

       emberAfCorePrintln("Button0PresseShort");
       last_press_time = current_press_time;

      // g_sw_ctl_info.key_led_on_off(SW_0,ON);
       emberEventControlSetActive(sw0_press_short_event_control);
       g_sw_ctl_info.set_key_press_event_flag(SW0_PRESS);

}





#ifndef ZERO_CROSS_TEST
void sw0_press_short_event_handler(void)
{
       uint16_t current_sys_time = halCommonGetInt16uMillisecondTick();
       uint16_t last_short_press_sys_time =  g_sw_ctl_info.get_last_short_press_sys_time();
       uint16_t delay_ms;

      static uint8_t sw0_process_step = STEP_KEYS_EVENT_TEST;
      static uint8_t zero_cross_test_timeout = 0;

      emberEventControlSetInactive(sw0_press_short_event_control);
      g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(true);//开启中断

      switch(sw0_process_step)
      {
        case STEP_KEYS_EVENT_TEST:
    //      emberAfCorePrintln("STEP_KEYS_EVENT_TEST");

           if((g_sw_ctl_info.get_key_press_event_flag(SW1_SHORT_PRESS))||\
             (g_sw_ctl_info.get_key_press_event_flag(SW2_SHORT_PRESS)))
           {
               emberAfCorePrintln("other key is press");
              emberEventControlSetDelayMS(sw0_press_short_event_control,10);
               return;
           }
           sw0_process_step = STEP_KEYS_INTERVAL;
           emberEventControlSetActive(sw0_press_short_event_control);
          break;

        case STEP_KEYS_INTERVAL:
      //    emberAfCorePrintln("STEP_KEYS_INTERVAL");
           g_sw_ctl_info.set_key_press_event_flag(SW0_SHORT_PRESS);
           g_sw_ctl_info.set_sleep_enable_state(true);

            if(current_sys_time>last_short_press_sys_time)
            {
               if(current_sys_time<last_short_press_sys_time+KEYS_INTERVAL)
                 {
                   delay_ms = KEYS_INTERVAL-(current_sys_time-last_short_press_sys_time)+10;
               //    emberAfCorePrintln("sw0 delay_ms1:%d ",delay_ms);
                   emberEventControlSetDelayMS(sw0_press_short_event_control,delay_ms);
                   return;
                 }
            }
            else
            {
                if((65535-last_short_press_sys_time+current_sys_time)<KEYS_INTERVAL)
                  {
                    delay_ms = KEYS_INTERVAL -(65535-last_short_press_sys_time+current_sys_time)+10;
                //    emberAfCorePrintln("sw0 delay_ms2:%d ",delay_ms);
                    emberEventControlSetDelayMS(sw0_press_short_event_control,delay_ms);
                     return;
                  }
            }

            if(g_sw_ctl_info.sw0_state == OFF)
              {
                sw0_process_step = STEP_ZERO_CROSS_TEST;
                g_sw_ctl_info.set_zero_cross_flag(false);
              //g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(true);//开启中断
                emberEventControlSetActive(sw0_press_short_event_control);
              }
            else
              {
                sw0_process_step = STEP_SW_ON_OFF;
               emberEventControlSetActive(sw0_press_short_event_control);
              }



          break;
        case STEP_ZERO_CROSS_TEST:
      //    emberAfCorePrintln("STEP_ZERO_CROSS_TEST");

#if 1

//          if(g_sw_ctl_info.sw0_state != OFF)
//            {
//
//              sw0_process_step = STEP_SW_ON_OFF;
//              return;
//            }

          if(g_sw_ctl_info.get_zero_cross_flag())
           {
             g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
             g_sw_ctl_info.set_zero_cross_flag(false);

        //     emberAfCorePrintln("have test zero cross");
             emberEventControlSetDelayMS(sw0_press_short_event_control,15);
             SW1_LED_OFF();
             sw0_process_step = STEP_SW_ON_OFF;
          //   return;
           }
           else
            {
           //    emberAfCorePrintln("no test zero cross");
            //   emberEventControlSetDelayMS(sw0_press_short_event_control,10);

               if(zero_cross_test_timeout++<20)
               {
                 emberEventControlSetDelayMS(sw0_press_short_event_control,1);

               }else //过零检测超时
               {
                   zero_cross_test_timeout = 0;
                   g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
                   g_sw_ctl_info.set_zero_cross_flag(false);

                   sw0_process_step = STEP_SW_ON_OFF;
                   emberEventControlSetActive(sw0_press_short_event_control);
                   emberAfCorePrintln("sw0 zero_cross_test_timeout");
               }

            //   return;
            }
#else
         sw0_process_step = STEP_SW_ON_OFF;
         emberEventControlSetActive(sw0_press_short_event_control);
#endif
           break;
        case STEP_SW_ON_OFF:

          emberAfCorePrintln(" SW0 STEP_SW_ON_OFF");

          if(g_sw_ctl_info.sw0_state == false)
          {
             g_sw_ctl_info.sw_ctl_callb(SW_0,true);
             g_sw_ctl_info.sw0_state = true;
          }
        else
          {
             g_sw_ctl_info.sw_ctl_callb(SW_0,false);
             g_sw_ctl_info.sw0_state = false;
          }


        if (g_sw_ctl_info.get_key_press_event_flag(SW0_PRESS))//解决非按键按下指示灯闪的问题
          {
              g_sw_ctl_info.clr_key_press_event_flag(SW0_PRESS);
              g_key_press_flag |=(1<<SW_0 ) ;
          }
           //save_sw_cfg_to_flash();
           zero_cross_test_timeout = 0;
           sw0_process_step = STEP_KEYS_EVENT_TEST;
           renew_sw_state_to_manager(SW_0, g_sw_ctl_info.sw0_state);

           SW1_LED_ON();
           g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);

//
//          EmberStatus  ststa = mfglibStart(mfglibRxCallback);
//
//           if(ststa == EMBER_SUCCESS)
//            {
//              emberAfCorePrintln("mfglibStart ok");
//
//            }
//          else
//            {
//              emberAfCorePrintln("mfglibStart fail");
//            }
//
//
//            ststa = mfglibSetChannel(12);
//
//           if(ststa == EMBER_SUCCESS)
//             {
//               emberAfCorePrintln("mfglibSetChannel ok");
//             }
//           else
//             {
//               emberAfCorePrintln("mfglibSetChannel fail");
//             }




         break;


      }

}

#else

uint16_t ingterrupt_cnt = 0;

void sw0_press_short_event_handler(void)
{
       uint16_t current_sys_time = halCommonGetInt16uMillisecondTick();
       uint16_t last_short_press_sys_time =  g_sw_ctl_info.get_last_short_press_sys_time();
       uint16_t delay_ms;

      static uint8_t sw0_process_step = STEP_KEYS_INTERVAL;
      static uint8_t zero_cross_test_timeout = 0;

     emberEventControlSetInactive(sw0_press_short_event_control);
//     // emberAfCorePrintln("sw0_press_short_event_handler");
//      g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(true);//开启中断
//
//      if(g_sw_ctl_info.get_zero_cross_flag())
//        {
//          // g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
//           g_sw_ctl_info.set_zero_cross_flag(false);
////           SW0_LED_TOGGLE();
////            SW1_LED_TOGGLE();
////            SW2_LED_TOGGLE();
//
//        }
//
//      emberEventControlSetDelayMS(sw0_press_short_event_control,1);
   g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(true);//开启中断

#if 1
      switch(sw0_process_step)
      {


        case STEP_KEYS_INTERVAL:

           g_sw_ctl_info.set_sleep_enable_state(true);

            if(g_sw_ctl_info.sw0_state == OFF)
              {
                sw0_process_step = STEP_ZERO_CROSS_TEST;
                g_sw_ctl_info.set_zero_cross_flag(false);
              //  g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(true);//开启中断
                emberEventControlSetActive(sw0_press_short_event_control);
                ingterrupt_cnt = 0;
              }
            else
              {
                sw0_process_step = STEP_SW_ON_OFF;
               emberEventControlSetActive(sw0_press_short_event_control);
              }



          break;
        case STEP_ZERO_CROSS_TEST:

          if(g_sw_ctl_info.get_zero_cross_flag())
           {
             g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
             g_sw_ctl_info.set_zero_cross_flag(false);
             emberEventControlSetDelayMS(sw0_press_short_event_control,15);
             sw0_process_step = STEP_SW_ON_OFF;
             test_zero_cross_flag_t2 = halCommonGetInt16uMillisecondTick();

             emberAfCorePrintln("ingterrupt_cnt:%d",ingterrupt_cnt);
             ingterrupt_cnt = 0;
             SW0_LED_TOGGLE();
             return;
           }
           else
            {
                // emberEventControlSetDelayMS(sw0_press_short_event_control,1);
               if(zero_cross_test_timeout++<20)
               {
                 emberEventControlSetDelayMS(sw0_press_short_event_control,1);

               }else //过零检测超时
               {
                   zero_cross_test_timeout = 0;
                   g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
                   g_sw_ctl_info.set_zero_cross_flag(false);

                   sw0_process_step = STEP_SW_ON_OFF;
                   emberEventControlSetActive(sw0_press_short_event_control);
                   emberAfCorePrintln("sw0 zero_cross_test_timeout");
               }

               return;
            }

           break;
        case STEP_SW_ON_OFF:

         // emberAfCorePrintln(" SW0 STEP_SW_ON_OFF");

          if(g_sw_ctl_info.sw0_state == false)
          {
             g_sw_ctl_info.sw_ctl_callb(SW_0,true);
             g_sw_ctl_info.sw0_state = true;
          }
        else
          {
             g_sw_ctl_info.sw_ctl_callb(SW_0,false);
             g_sw_ctl_info.sw0_state = false;
          }

          before_ralay_control_t3 = halCommonGetInt16uMillisecondTick();

           zero_cross_test_timeout = 0;
           sw0_process_step = STEP_KEYS_INTERVAL;
           emberEventControlSetDelayMS(sw0_press_short_event_control,1000);
         break;


      }

#endif

}



#endif


void emberAfPluginButtonInterfaceButton1PressedShortCallback(uint16_t timePressedMs)
{
     if(timePressedMs<20)//消抖
       {
         return;
       }
     static uint16_t  last_press_time = 0;
     static uint16_t  current_press_time = 0;
     const  uint16_t  KEY_VALID_MIN_INTERVAL = 500;//500ms

     current_press_time = halCommonGetInt16uMillisecondTick();

        g_sw_ctl_info.key_led_on_off(SW_1,ON);
    // emberAfCorePrintln("key press interval：%d",current_press_time-last_press_time);

       if(current_press_time>last_press_time)
       {
          if(current_press_time<last_press_time+KEY_VALID_MIN_INTERVAL)
            {
              last_press_time = current_press_time;
              return;
            }
       }
       else
       {
           if((65535-last_press_time+current_press_time)<KEY_VALID_MIN_INTERVAL)
             {
               last_press_time = current_press_time;
                return;

             }

       }

      emberAfCorePrintln("Button1PresseShort");
      last_press_time = current_press_time;
     // g_sw_ctl_info.key_led_on_off(SW_1,ON);
      g_sw_ctl_info.set_key_press_event_flag(SW1_PRESS);
      emberEventControlSetActive(sw1_press_short_event_control);

}

//uint8_t test_data[20] = {0x0c,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a};

void sw1_press_short_event_handler(void)
{
     uint16_t current_sys_time = halCommonGetInt16uMillisecondTick();
     uint16_t last_short_press_sys_time =  g_sw_ctl_info.get_last_short_press_sys_time();
     uint16_t delay_ms;

    static uint8_t zero_cross_test_timeout = 0;
    static uint8_t sw1_process_step = STEP_KEYS_EVENT_TEST;
    emberEventControlSetInactive(sw1_press_short_event_control);

    switch(sw1_process_step)
    {
      case STEP_KEYS_EVENT_TEST:
     //   emberAfCorePrintln(" SW1 STEP_KEYS_EVENT_TEST");
         if((g_sw_ctl_info.get_key_press_event_flag(SW0_SHORT_PRESS))||\
           (g_sw_ctl_info.get_key_press_event_flag(SW2_SHORT_PRESS)))
         {
            emberEventControlSetDelayMS(sw1_press_short_event_control,10);
           return;
         }
         sw1_process_step = STEP_KEYS_INTERVAL;
         emberEventControlSetActive(sw1_press_short_event_control);
        break;

      case STEP_KEYS_INTERVAL:
     //  emberAfCorePrintln(" SW1 STEP_KEYS_INTERVAL");
         g_sw_ctl_info.set_key_press_event_flag(SW1_SHORT_PRESS);
         g_sw_ctl_info.set_sleep_enable_state(true);

          if(current_sys_time>last_short_press_sys_time)
          {
             if(current_sys_time<last_short_press_sys_time+KEYS_INTERVAL)
               {
                 delay_ms = KEYS_INTERVAL-(current_sys_time-last_short_press_sys_time)+10;
                 emberEventControlSetDelayMS(sw1_press_short_event_control,delay_ms);
                 return;
               }
          }
          else
          {
              if((65535-last_short_press_sys_time+current_sys_time)<KEYS_INTERVAL)
                {
                  delay_ms = KEYS_INTERVAL -(65535-last_short_press_sys_time+current_sys_time)+10;
                  emberEventControlSetDelayMS(sw1_press_short_event_control,delay_ms);
                   return;
                }
          }
          if(g_sw_ctl_info.sw1_state == OFF)
            {
              sw1_process_step = STEP_ZERO_CROSS_TEST;
              g_sw_ctl_info.set_zero_cross_flag(false);
              g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(true);//kai中断
              emberEventControlSetActive(sw1_press_short_event_control);
            }else
              {
                sw1_process_step = STEP_SW_ON_OFF;
                emberEventControlSetActive(sw1_press_short_event_control);
              }


        break;
      case STEP_ZERO_CROSS_TEST:
    //    emberAfCorePrintln(" SW1 STEP_ZERO_CROSS_TEST");

#if 1
       if(g_sw_ctl_info.get_zero_cross_flag())
         {
           g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
           g_sw_ctl_info.set_zero_cross_flag(false);


           sw1_process_step = STEP_SW_ON_OFF;
           emberEventControlSetDelayMS(sw1_press_short_event_control,15);

           //zero_test_t1_record = halCommonGetInt16uMillisecondTick();
        //   emberAfCorePrintln("sw1 interrupt_t0_record t0:%d",interrupt_t0_record);
        //   emberAfCorePrintln("sw1 zero_cross test t1:%d",zero_test_t1_record);
         //  emberAfCorePrintln("sw1 t1-t0: %d ",zero_test_t1_record-interrupt_t0_record);

           return;
         }
         else
          {


             if(zero_cross_test_timeout++<20)
               {
                 emberEventControlSetDelayMS(sw1_press_short_event_control,1);

               }else //过零检测超时
               {
                   zero_cross_test_timeout = 0;
                   g_sw_ctl_info.zero_cross_siganl_gpio_interrupt_set(false);//关中断
                   g_sw_ctl_info.set_zero_cross_flag(false);

                   sw1_process_step = STEP_SW_ON_OFF;
                   emberEventControlSetActive(sw1_press_short_event_control);
                   emberAfCorePrintln("sw1 zero_cross_test_timeout");
               }
             return;
          }
#else
       sw1_process_step = STEP_SW_ON_OFF;
       emberEventControlSetActive(sw1_press_short_event_control);
#endif
       break;

      case STEP_SW_ON_OFF:

        emberAfCorePrintln(" SW1 STEP_SW_ON_OFF");

       // emberAfCorePrintln(" ralay on off delay t3:%d",halCommonGetInt16uMillisecondTick()-zero_test_t1_record);

        if(g_sw_ctl_info.sw1_state == false)
        {
           g_sw_ctl_info.sw_ctl_callb(SW_1,true);
           g_sw_ctl_info.sw1_state = true;
        }
      else
        {
           g_sw_ctl_info.sw_ctl_callb(SW_1,false);
           g_sw_ctl_info.sw1_state = false;
        }

     //    g_sw_ctl_info.key_led_on_off(SW_1,ON);

        if (g_sw_ctl_info.get_key_press_event_flag(SW1_PRESS))//解决非按键按下指示灯闪的问题
        {
            g_sw_ctl_info.clr_key_press_event_flag(SW1_PRESS);
            g_key_press_flag |=(1<<SW_1 ) ;
        }
         renew_sw_state_to_manager(SW_1, g_sw_ctl_info.sw1_state);
         sw1_process_step = STEP_KEYS_EVENT_TEST;
         zero_cross_test_timeout  = 0;
       //   save_sw_cfg_to_flash();



    //   mfglibSetPower(0,10);

//         ststa = mfglibSetPower(0,10);

//        if(ststa == EMBER_SUCCESS)
//        {
//          emberAfCorePrintln("mfglibSetPower ok");
//        }
//      else
//        {
//          emberAfCorePrintln("mfglibSetPower fail");
//        }

     //   mfglibStartTone();

//        ststa = mfglibStartTone();
//        if(ststa == EMBER_SUCCESS)
//           {
//             emberAfCorePrintln("mfglibStartTone ok");
//           }
//         else
//           {
//             emberAfCorePrintln("mfglibStartTone fail");
//           }

//         EmberStatus  ststa = mfglibSetChannel(12);
//
//          if(ststa == EMBER_SUCCESS)
//            {
//              emberAfCorePrintln("mfglibSetChannel ok");
//            }
//          else
//            {
//              emberAfCorePrintln("mfglibSetChannel fail");
//            }


//         EmberStatus ststa = mfglibSendPacket(test_data,0);
//
//         if(ststa == EMBER_SUCCESS)
//           {
//             emberAfCorePrintln("mfglibSendPacket ok");
//           }
//         else
//           {
//             emberAfCorePrintln("mfglibSendPacket fail");
//           }


          break;


    }

}




void emberAfPluginButtonInterfaceButton0LowCallback(void)
{
   emberEventControlSetDelayMS(sw0_press_low_event_control,10);
}

void emberAfPluginButtonInterfaceButton1LowCallback(void)
{
  emberEventControlSetDelayMS(sw1_press_low_event_control,10);
}

void emberAfPluginButtonInterfaceButton2LowCallback(void)
{
  emberEventControlSetDelayMS(sw2_press_low_event_control,10);
}

void sw2_press_low_event_handler(void)
{

  emberEventControlSetInactive(sw2_press_low_event_control);
  uint8_t sw_press_state = halButtonPinState(BSP_BUTTON2_PIN);

  if(sw_press_state == BUTTON_PRESSED)
    {
      g_sw_ctl_info.key_led_on_off(SW_2,OFF);
    }

}

void sw1_press_low_event_handler(void)
{

  emberEventControlSetInactive(sw1_press_low_event_control);
  uint8_t sw_press_state = halButtonPinState(BSP_BUTTON1_PIN);//消抖
   if(sw_press_state == BUTTON_PRESSED)
     {
       g_sw_ctl_info.key_led_on_off(SW_1,OFF);
     }

}

void sw0_press_low_event_handler(void)
{

   emberEventControlSetInactive(sw0_press_low_event_control);
   uint8_t sw_press_state = halButtonPinState(BSP_BUTTON0_PIN);//消抖
    if(sw_press_state == BUTTON_PRESSED)
      {
        g_sw_ctl_info.key_led_on_off(SW_0,OFF);
      }

}

void emberAfPluginButtonInterfaceButton0PressingCallback(void)
{
   EmberNetworkStatus state = emberAfNetworkState();
  if (state == EMBER_NO_NETWORK)
    {
      g_sw_ctl_info.key_press_event_flag |= (1<<START_JOIN_NETWORK);
    }
  else  if ( (state == EMBER_JOINED_NETWORK_NO_PARENT)||(state == EMBER_JOINED_NETWORK) )
    {

      g_sw_ctl_info.key_press_event_flag |= (1<<LEAVE_NETWORK);

    }
    emberAfCorePrintln("LONG_PRESS ");
}


void network_up_renew_sw_state_event_handler(void)
{



#if 1
 static  uint8_t next_sw_num = STEP_MODEL_ID_0;
  emberEventControlSetInactive(network_up_renew_sw_state_event_control);

  switch(next_sw_num)
  {
      case STEP_MODEL_ID_0:
        next_sw_num = STEP_MODEL_ID_1;
        renew_model_id_to_manager();
        emberEventControlSetDelayMS(network_up_renew_sw_state_event_control,1000);
      break;

      case STEP_MODEL_ID_1:
        next_sw_num = STEP_SOFT_VER_1;
        renew_model_id_to_manager();
        emberEventControlSetDelayMS(network_up_renew_sw_state_event_control,1000);
      break;

      case STEP_SOFT_VER_1:
        next_sw_num = STEP_SOFT_VER_2;
        renew_model_id_to_manager();
        emberEventControlSetDelayMS(network_up_renew_sw_state_event_control,1000);
     break;

      case STEP_SOFT_VER_2:
        next_sw_num = STEP_SW0;
        renew_model_id_to_manager();
        emberEventControlSetDelayMS(network_up_renew_sw_state_event_control,1000);
     break;

      case STEP_SW0:
      next_sw_num = STEP_SW1;
      renew_sw_state_to_manager(SW_0, g_sw_ctl_info.sw0_state);
      emberAfCorePrintln("network SW_0 state send");

      emberEventControlSetDelayMS(network_up_renew_sw_state_event_control,1000);
     break;

    case STEP_SW1:
      next_sw_num = STEP_SW2;
      renew_sw_state_to_manager(SW_1, g_sw_ctl_info.sw1_state);
      emberAfCorePrintln("network SW_1 state send");

      emberEventControlSetDelayMS(network_up_renew_sw_state_event_control,1000);
   break;

    case STEP_SW2:
      next_sw_num = STEP_MODEL_ID_0;
     renew_sw_state_to_manager(SW_2, g_sw_ctl_info.sw2_state);
     emberAfCorePrintln("network SW_2 state send");

      break;

    default:
      break;

  }

#endif
}


void long_press_network_link_event_handler(void)
{


   // emberAfCorePrintln("sys_tick: %d",halCommonGetInt16uMillisecondTick());
    emberEventControlSetInactive(long_press_network_link_event_control);

     EmberNetworkStatus state = emberAfNetworkState();

     if ((state == EMBER_JOINED_NETWORK)&&(g_sw_ctl_info.key_press_event_flag&(1<<START_JOIN_NETWORK)))
      {  //首次连网后的数据清零
         emberAfCorePrintln("network link ok,clear cfg para");
         g_sw_ctl_info.network_link_cnt = 0;
         g_sw_ctl_info.key_press_event_flag &=~((1<<START_JOIN_NETWORK));
         g_sw_ctl_info.network_link_time= 0;
         SW0_LED_ON();
         emberEventControlSetDelayMS(network_up_renew_sw_state_event_control,10000);//同步开关状态给网关
         return;
      }


     if(!(g_sw_ctl_info.key_press_event_flag&((1<<START_JOIN_NETWORK)+(1<<LEAVE_NETWORK)) )){
        return;
        }

    if(g_sw_ctl_info.network_link_cnt == 0)
     {
       g_sw_ctl_info.network_link_time = NETWORK_LINK_PERIOD;//首次立即入网
     }

     SW0_LED_TOGGLE();


    if(g_sw_ctl_info.key_press_event_flag&(1<<START_JOIN_NETWORK))
      {

          if(g_sw_ctl_info.network_link_time++>=NETWORK_LINK_PERIOD)//10s入网周期
           {
              g_sw_ctl_info.network_link_time = 0;
              if(g_sw_ctl_info.network_link_cnt++<NETWORK_LINK_TIMEOUT_MAX_CNT)
                {

                    EmberStatus status = emberAfPluginNetworkSteeringStart();
                    emberAfCorePrintln("%p network %p: 0x%X", "Join", "start", status);

                }
              else
                {
                  emberAfCorePrintln("network link time out");
                  SW0_LED_ON();
                  g_sw_ctl_info.network_link_cnt = 0;
                  g_sw_ctl_info.key_press_event_flag &=~((1<<START_JOIN_NETWORK));
                }

           }

      }
    else if(g_sw_ctl_info.key_press_event_flag&(1<<LEAVE_NETWORK))
      {


       //  if(g_sw_ctl_info.network_leave_time++>NETWORK_LEAVE_PERIOD)
          {
               networkLeaveCommand();//离网

             if(g_sw_ctl_info.sw0_state )
               {

                 emberEventControlSetDelayMS(sw0_press_short_event_control,10);
               }
             if(g_sw_ctl_info.sw1_state )
               {

                 emberEventControlSetDelayMS(sw1_press_short_event_control,300);
               }
             if(g_sw_ctl_info.sw2_state )
              {

                 emberEventControlSetDelayMS(sw2_press_short_event_control,600);
              }

             g_sw_ctl_info.heartbeat_period = HEARTBEAT_PRIOD;
             g_sw_ctl_info.switch_power_up_state = SW_POWER_UP_STATE;
             g_sw_ctl_info.set_sleep_enable_state(DEFAULT_SLEEP_STATE);
             save_sw_cfg_to_flash();

             g_sw_ctl_info.network_leave_time = 0;
             g_sw_ctl_info.key_press_event_flag &=~((1<<LEAVE_NETWORK));
          }

      }

}




bool emberAfPreCommandReceivedCallback(EmberAfClusterCommand* cmd)
{
    emberAfCorePrintln("emberAfPreCommandReceivedCallback");

    uint8_t dest_endpoint = cmd->apsFrame->destinationEndpoint;
    uint8_t cmd_id = cmd->commandId;

   switch(dest_endpoint-1)
   {
     case SW_0:
     if(cmd_id == ON)
       {
            if(g_sw_ctl_info.sw0_state == OFF)
             {
                emberEventControlSetActive(sw0_press_short_event_control);
             }
            else
              {
                renew_sw_state_to_manager(SW_0,ON);
              }
       }
     else if(cmd_id == OFF)
       {
           if(g_sw_ctl_info.sw0_state == ON)
           {
              emberEventControlSetActive(sw0_press_short_event_control);
           }
          else
          {
              renew_sw_state_to_manager(SW_0,OFF);
          }

       }

     break;
     case SW_1:

       if(cmd_id == ON)
            {
                 if(g_sw_ctl_info.sw1_state == OFF)
                  {
                     emberEventControlSetActive(sw1_press_short_event_control);
                  }
                 else
                   {
                     renew_sw_state_to_manager(SW_1,ON);
                   }
            }
          else if(cmd_id == OFF)
            {
                if(g_sw_ctl_info.sw1_state == ON)
                {
                   emberEventControlSetActive(sw1_press_short_event_control);
                }
               else
               {
                   renew_sw_state_to_manager(SW_1,OFF);
               }

            }
     break;
     case SW_2:

       if(cmd_id == ON)
        {
             if(g_sw_ctl_info.sw2_state == OFF)
              {
                 emberEventControlSetActive(sw2_press_short_event_control);
              }
             else
               {
                 renew_sw_state_to_manager(SW_2,ON);
               }
        }
          else if(cmd_id == OFF)
        {
            if(g_sw_ctl_info.sw2_state == ON)
            {
               emberEventControlSetActive(sw2_press_short_event_control);
            }
           else
           {
               renew_sw_state_to_manager(SW_2,OFF);
           }

        }
     break;

   }

   return false;//切记要返回false，否则OTA升级出问题
}


bool emberAfPluginIdleSleepOkToSleepCallback(uint32_t durationMs)
{
      return true;
}

void emberAfPluginIdleSleepWakeUpCallback(uint32_t durationMs)
{
}


bool emberAfPluginInterpanPreMessageReceivedCallback(const EmberAfInterpanHeader *header,
                                                     uint8_t msgLen,
                                                     uint8_t *message)
{

   return true;

}


