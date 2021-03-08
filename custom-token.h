/*
 * custom-token.h
 *
 *  Created on: 2020年12月24日
 *      Author: zhinple
 */
// File: custom-token.h
//
// Description: Custom token definitions used by the application.
//
// Copyright 2019 by Silicon Labs Corporation.  All rights reserved.

/**
* Custom Zigbee Application Tokens
*/
// Define token names here
#define NVM3KEY_SW_ON_OFF     (NVM3KEY_DOMAIN_USER | 0x0001)

#if defined(DEFINETYPES)
// Include or define any typedef for tokens here
typedef struct {
//  bool sw0_state;        // LED ON OFF status
//  bool sw1_state;        // LED ON OFF status
//  bool sw2_state;        // LED ON OFF status
  uint16_t heartbeat_period;
  bool switch_power_up_state;

} sw_On_off_status_t;
#endif //DEFINETYPES

#ifdef DEFINETOKENS
// Define the actual token storage information here
DEFINE_BASIC_TOKEN(SW_ON_OFF,
                   sw_On_off_status_t,
                  {false, false,false})
#endif
