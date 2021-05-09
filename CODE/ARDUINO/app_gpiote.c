
// ###################   Mini air quality and wither station with electronic ink display 2.13 Inch | nRF52   ############### //
//                                                                                                                           //
//        @filename   :   MWS213_V2R3_ED3.ino                                                                                //
//        @brief en   :   Wireless, battery-operated air quality(VOC Sensor SGP40), temperature,humidity and pressure        //
//                        sensor(BME280) with electronic ink display(Good Display GDEH0213B72, GDEH0213B73, Waveshare V2).   //
//                        Works on SOC nRF52.                                                                                //
//        @brief ru   :   Беcпроводной, батарейный датчик качества воздуха (ЛОС сенсорс SGP40), температуры, влажности       //
//                        и давления(BME280) с дисплеем на электронных чернилах(Good Display GDEH0213B72, GDEH0213B73,       //
//                        Waveshare V2).                                                                                     //
//                        Работает на nRF52832, nRF52840.                                                                    //
//                                                                                                                           //
//        @author     :   Andrew Lamchenko aka Berk                                                                          //
//                                                                                                                           //
//        Copyright (C) 2021, EFEKTALAB                                                                                      //
//        Copyright (c) 2020, Sensirion AG                                                                                   //
//        Copyright (c) 2020, Bosch Sensortec GmbH. All rights reserved.                                                     //
//        Copyright (c) 2014-2015, Arduino LLC.  All right reserved.                                                         //
//        Copyright (c) 2016, Arduino Srl.  All right reserved.                                                              //
//        Copyright (c) 2017, Sensnology AB. All right reserved.                                                             //
//        Copyright (C) 2020, Waveshare                                                                                      //
//                                                                                                                           //
// ######################################################################################################################### //

#include "app_gpiote.h"
#include <string.h>
//#include "app_util.h"
#include "nrf_gpio.h"


/**@brief GPIOTE user type. */
typedef struct
{
    uint32_t                   pins_mask;             /**< Mask defining which pins user wants to monitor. */
    uint32_t                   pins_low_to_high_mask; /**< Mask defining which pins will generate events to this user when toggling low->high. */
    uint32_t                   pins_high_to_low_mask; /**< Mask defining which pins will generate events to this user when toggling high->low. */
    uint32_t                   sense_high_pins;       /**< Mask defining which pins are configured to generate GPIOTE interrupt on transition to high level. */
    app_gpiote_event_handler_t event_handler;         /**< Pointer to function to be executed when an event occurs. */
} gpiote_user_t;


static uint32_t        m_enabled_users_mask;          /**< Mask for tracking which users are enabled. */
static uint8_t         m_user_array_size;             /**< Size of user array. */
static uint8_t         m_user_count;                  /**< Number of registered users. */
static gpiote_user_t * mp_users = NULL;               /**< Array of GPIOTE users. */


/**@brief Toggle sense level for specified pins.
 *
 * @param[in]   p_user   Pointer to user structure.
 * @param[in]   pins     Bitmask specifying for which pins the sense level is to be toggled.
 */
static void sense_level_toggle(gpiote_user_t * p_user, uint32_t pins)
{
    uint32_t pin_no;
    
    for (pin_no = 0; pin_no < NO_OF_PINS; pin_no++)
    {
        uint32_t pin_mask = (1 << pin_no);
        
        if ((pins & pin_mask) != 0)
        {
            uint32_t sense;

            // Invert sensing.
            if ((p_user->sense_high_pins & pin_mask) == 0)
            {
                sense = GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos;
                p_user->sense_high_pins |= pin_mask;
            }
            else
            {
                sense = GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos;
                p_user->sense_high_pins &= ~pin_mask;
            }

            NRF_GPIO->PIN_CNF[pin_no] &= ~GPIO_PIN_CNF_SENSE_Msk;
            NRF_GPIO->PIN_CNF[pin_no] |= sense;
        }
    }
}


/**@brief GPIOTE interrupt handler.
 */
void GPIOTE_IRQHandler(void)
{
    uint8_t  i;
    uint32_t pins_state = NRF_GPIO->IN;
    
    // Clear event.
    NRF_GPIOTE->EVENTS_PORT = 0;
    
    // Check all users.
    for (i = 0; i < m_user_count; i++)
    {
        gpiote_user_t * p_user = &mp_users[i];

        // Check if user is enabled.
        if (((1 << i) & m_enabled_users_mask) != 0)
        {
            uint32_t transition_pins;
            uint32_t event_low_to_high;
            uint32_t event_high_to_low;

            // Find set of pins on which there has been a transition.
            transition_pins = (pins_state ^ ~p_user->sense_high_pins) & p_user->pins_mask;

            // Toggle SENSE level for all pins that have changed state.
            sense_level_toggle(p_user, transition_pins);
            
            // Call user event handler if an event has occurred.
            event_high_to_low = (~pins_state & p_user->pins_high_to_low_mask) & transition_pins;
            event_low_to_high = (pins_state & p_user->pins_low_to_high_mask) & transition_pins;

            if ((event_low_to_high | event_high_to_low) != 0)
            {
                p_user->event_handler(event_low_to_high, event_high_to_low);
            }
        }
    }
}


/**@brief Disable sensing for all pins for specified user.
 *
 * @param[in]  user_id   User id.
 */
static void pins_sense_disable(app_gpiote_user_id_t user_id)
{
    uint32_t pin_no;

    for (pin_no = 0; pin_no < 32; pin_no++)
    {
        if ((mp_users[user_id].pins_mask & (1 << pin_no)) != 0)
        {
            NRF_GPIO->PIN_CNF[pin_no] &= ~GPIO_PIN_CNF_SENSE_Msk;
            NRF_GPIO->PIN_CNF[pin_no] |= GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos;
        }
    }
}


void app_gpiote_init(uint8_t max_users, void * p_buffer)
{
    mp_users             = (gpiote_user_t *)p_buffer;
    m_user_array_size    = max_users;
    m_user_count         = 0;
    m_enabled_users_mask = 0;
    
    memset(mp_users, 0, m_user_array_size * sizeof(gpiote_user_t));
    
    // Initialize GPIOTE interrupt (will not be enabled until app_gpiote_user_enable() is called).
    NRF_GPIOTE->INTENCLR = 0xFFFFFFFF;
    
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_HIGH);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}


void app_gpiote_user_register(app_gpiote_user_id_t *     p_user_id,
                                  uint32_t                   pins_low_to_high_mask,
                                  uint32_t                   pins_high_to_low_mask,
                                  app_gpiote_event_handler_t event_handler)
{ 
    // Allocate new user.
    mp_users[m_user_count].pins_mask             = pins_low_to_high_mask | pins_high_to_low_mask;
    mp_users[m_user_count].pins_low_to_high_mask = pins_low_to_high_mask;
    mp_users[m_user_count].pins_high_to_low_mask = pins_high_to_low_mask;
    mp_users[m_user_count].event_handler         = event_handler;
    
    *p_user_id = m_user_count++;
    
    // Make sure SENSE is disabled for all pins.
    pins_sense_disable(*p_user_id);
}


void app_gpiote_user_enable(app_gpiote_user_id_t user_id)
{
    uint32_t pin_no;
    uint32_t pins_state;

    // Clear any pending event.
    NRF_GPIOTE->EVENTS_PORT = 0;
    pins_state              = NRF_GPIO->IN;

    // Enable user.
    if (m_enabled_users_mask == 0)
    {
        NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    }
    m_enabled_users_mask |= (1 << user_id);
    
    // Enable sensing for all pins for specified user.
    mp_users[user_id].sense_high_pins = 0;
    for (pin_no = 0; pin_no < 32; pin_no++)
    {
        uint32_t pin_mask = (1 << pin_no);
        
        if ((mp_users[user_id].pins_mask & pin_mask) != 0)
        {
            uint32_t sense;

            if ((pins_state & pin_mask) != 0)
            {
                sense = GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos;
            }                
            else
            {
                sense = GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos;
                mp_users[user_id].sense_high_pins |= pin_mask;
            }
            
            NRF_GPIO->PIN_CNF[pin_no] &= ~GPIO_PIN_CNF_SENSE_Msk;
            NRF_GPIO->PIN_CNF[pin_no] |= sense;
        }
    }
}


void app_gpiote_user_disable(app_gpiote_user_id_t user_id)
{  
    // Disable sensing for all pins for specified user.
    pins_sense_disable(user_id);

    // Disable user.
    m_enabled_users_mask &= ~(1UL << user_id);
    if (m_enabled_users_mask == 0)
    {
        NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_PORT_Msk;
    }
}


void app_gpiote_pins_state_get(app_gpiote_user_id_t user_id, uint32_t * p_pins)
{
    gpiote_user_t * p_user;

    // Get pins.
    p_user  = &mp_users[user_id];
    *p_pins = NRF_GPIO->IN & p_user->pins_mask;
    
}
