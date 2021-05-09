
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
#ifndef APP_GPIOTE_H__
#define APP_GPIOTE_H__

#include <stdbool.h>
#include "nrf.h"
//#include "app_util.h"

/**@brief The interrupt priorities available to the application while the softdevice is active. */
typedef enum {
    APP_IRQ_PRIORITY_HIGH = 1,
    APP_IRQ_PRIORITY_LOW  = 3
} app_irq_priority_t;


/**@brief Perform rounded integer division (as opposed to truncating the result).
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Rounded (integer) result of dividing A by B.
 */
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))

/**@brief Check if the integer provided is a power of two.
 *
 * @param[in]   A   Number to be tested.
 *
 * @return      true if value is power of two.
 * @return      false if value not power of two.
 */
#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )

/**@brief Perform integer division, making sure the result is rounded up.
 *
 * @details One typical use for this is to compute the number of objects with size B is needed to
 *          hold A number of bytes.
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Integer result of dividing A by B, rounded up.
 */
#define CEIL_DIV(A, B)      \
    /*lint -save -e573 */   \
    ((((A) - 1) / (B)) + 1) \
    /*lint -restore */


#define GPIOTE_USER_NODE_SIZE   20          /**< Size of app_gpiote.gpiote_user_t (only for use inside APP_GPIOTE_BUF_SIZE()). */
#define NO_OF_PINS              32          /**< Number of GPIO pins on the nRF51 chip. */

/**@brief Compute number of bytes required to hold the GPIOTE data structures.
 *
 * @param[in]   MAX_USERS   Maximum number of GPIOTE users.
 *
 * @return      Required buffer size (in bytes).
 */
#define APP_GPIOTE_BUF_SIZE(MAX_USERS)  ((MAX_USERS) * GPIOTE_USER_NODE_SIZE)

typedef uint8_t app_gpiote_user_id_t;

/**@brief GPIOTE event handler type. */
typedef void (*app_gpiote_event_handler_t)(uint32_t event_pins_low_to_high,
                                           uint32_t event_pins_high_to_low);

/**@brief GPIOTE input event handler type. */
typedef void (*app_gpiote_input_event_handler_t)(void);

/**@brief Macro for initializing the GPIOTE module.
 *
 * @details It will handle dimensioning and allocation of the memory buffer required by the module,
 *          making sure that the buffer is correctly aligned.
 *
 * @param[in]   MAX_USERS   Maximum number of GPIOTE users.
 *
 * @note Since this macro allocates a buffer, it must only be called once (it is OK to call it
 *       several times as long as it is from the same location, e.g. to do a reinitialization).
 */
/*lint -emacro(506, APP_GPIOTE_INIT) */ /* Suppress "Constant value Boolean */
#define APP_GPIOTE_INIT(MAX_USERS)                                                                 \
    do                                                                                             \
    {                                                                                              \
        static uint32_t app_gpiote_buf[CEIL_DIV(APP_GPIOTE_BUF_SIZE(MAX_USERS), sizeof(uint32_t))];\
        app_gpiote_init((MAX_USERS), app_gpiote_buf);                          \
    } while (0)

/**@brief Function for initializing the GPIOTE module.
 *
 * @note Normally initialization should be done using the APP_GPIOTE_INIT() macro, as that will
 *       allocate the buffer needed by the GPIOTE module (including aligning the buffer correctly).
 *
 * @param[in]   max_users               Maximum number of GPIOTE users.
 * @param[in]   p_buffer                Pointer to memory buffer for internal use in the app_gpiote
 *                                      module. The size of the buffer can be computed using the
 *                                      APP_GPIOTE_BUF_SIZE() macro. The buffer must be aligned to 
 *                                      a 4 byte boundary.
 *
 * @retval      NRF_SUCCESS             Successful initialization.
 * @retval      NRF_ERROR_INVALID_PARAM Invalid parameter (buffer not aligned to a 4 byte
 *                                      boundary).
 */
void app_gpiote_init(uint8_t max_users, void * p_buffer);

/**@brief Function for registering a GPIOTE user.
 *
 * @param[out]  p_user_id               Id for the new GPIOTE user.
 * @param[in]   pins_low_to_high_mask   Mask defining which pins will generate events to this user 
 *                                      when state is changed from low->high.
 * @param[in]   pins_high_to_low_mask   Mask defining which pins will generate events to this user
 *                                      when state is changed from high->low.
 * @param[in]   event_handler           Pointer to function to be executed when an event occurs.
 *
 * @retval      NRF_SUCCESS             Successful initialization.
 * @retval      NRF_ERROR_INVALID_PARAM Invalid parameter (buffer not aligned to a 4 byte boundary).
 * @retval      NRF_ERROR_INALID_STATE  If @ref app_gpiote_init has not been called on the GPIOTE
 *                                      module.
 * @retval      NRF_ERROR_NO_MEM        Returned if the application tries to register more users
 *                                      than defined when the GPIOTE module was initialized in
 *                                      @ref app_gpiote_init.
 */
void app_gpiote_user_register(app_gpiote_user_id_t *     p_user_id,
                                  uint32_t                   pins_low_to_high_mask,
                                  uint32_t                   pins_high_to_low_mask,
                                  app_gpiote_event_handler_t event_handler);

/**@brief Function for informing the GPIOTE module that the specified user wants to use the GPIOTE module.
 *
 * @param[in]   user_id                 Id of user to enable.
 *
 * @retval      NRF_SUCCESS             On success.
 * @retval      NRF_ERROR_INVALID_PARAM Invalid user_id provided, No a valid user.
 * @retval      NRF_ERROR_INALID_STATE  If @ref app_gpiote_init has not been called on the GPIOTE
 *                                      module.
 */
void app_gpiote_user_enable(app_gpiote_user_id_t user_id);

/**@brief Function for informing the GPIOTE module that the specified user is done using the GPIOTE module.
 *
 * @param[in]   user_id                   Id of user to enable.
 *
 * @return      NRF_SUCCESS               On success.
 * @retval      NRF_ERROR_INVALID_PARAM   Invalid user_id provided, No a valid user.
 * @retval      NRF_ERROR_INALID_STATE    If @ref app_gpiote_init has not been called on the GPIOTE
 *                                        module.
 */
void app_gpiote_user_disable(app_gpiote_user_id_t user_id);


#endif // APP_GPIOTE_H__

/** @} */
