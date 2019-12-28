/*
 * Copyright (c) 2019 Nick Ward
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CTS_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CTS_H_

/**
 * @brief Heart Rate Service (CTS)
 * @defgroup bt_gatt_CTS Heart Rate Service (CTS)
 * @ingroup bluetooth
 * @{
 *
 * [Experimental] Users should note that the APIs can change
 * as a part of ongoing development.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define CTS_YEAR_UNKNOWN               0
#define CTS_YEAR_MIN                   1582
#define CTS_YEAR_MAX                   9999

#define CTS_MONTH_UNKNOWN              0
#define CTS_MONTH_MIN                  1
#define CTS_MONTH_MAX                  12

#define CTS_DAY_UNKNOWN                0
#define CTS_DAY_MIN                    1
#define CTS_DAY_MAX                    31

#define CTS_WDAY_UNKNOWN               0
#define CTS_WDAY_MIN                   1
#define CTS_WDAY_MAX                   7

#define CTS_MANUAL_TIME_UPDATE         BIT(0)
#define CTS_EXT_REF_TIME_UPDATE        BIT(1)
#define CTS_CHANGE_TZ_TIME_UPDATE      BIT(2)
#define CTS_CHANGE_DST_TIME_UPDATE     BIT(3)

#define CTS_TIME_ZONE_MIN              -48
#define CTS_TIME_ZONE_MAX              56

#define CTS_DST_OFFSET_STD_TIME        0
#define CTS_DST_OFFSET_PLUS_HALF_HR    2
#define CTS_DST_OFFSET_DAYLIGHT_TIME   4
#define CTS_DST_OFFSET_DDAYLIGHT_TIME  8
#define CTS_DST_OFFSET_NOT_KNOWN       255

/** @brief Notify current time.
 *
 * This will send a GATT notification to all current subscribers.
 *
 *  @param adjust_reason The reason the current time has been adjusted.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_gatt_cts_notify(u8_t reason);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CTS_H_ */
