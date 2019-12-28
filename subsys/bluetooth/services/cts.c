/** @file
 *  @brief GATT Current Time Service
 */

/*
 * Copyright (c) 2019 Nick Ward
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include <init.h>
#include <time.h>
#include <sys/byteorder.h>
#include <posix/time.h>
#include <posix/sys/time.h>
#include <posix/unistd.h>
#include <sys/timeutil.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/cts.h>

#define LOG_LEVEL CONFIG_BT_GATT_CTS_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(cts);

/* CTS error definitions */
#define CTS_ERR_DATA_FIELD_IGNORED			0x80

struct cts_current_time {
	/* Exact Time 256 */
		/* Day Date Time */
			/* Date Time */
	u16_t year; /* 0 (Not known), 1582-9999 */
	u8_t month; /* 0 (Not known), 1-12 */
	u8_t day; /* 0 (Not known), 1-31 */
	u8_t hours;
	u8_t minutes;
	u8_t seconds;
			/* Day of Week */
	u8_t day_of_week; /* 0 (Not known), 1 (Monday) - 7 (Sunday) */
	/* Fractions256 */
	u8_t fractions_256;
	/* Adjust reason - flags */
	u8_t adjust_reason;
} __packed;

struct cts_local_time_info {
	/* Time Zone */
	/* Offset from UTC in number of 15 minutes increments. A value of -128 means that the time zone offset is not known. The offset defined in this characteristic is constant, regardless whether daylight savings is in effect. */
	s8_t time_zone;
	/* Daylight Saving Time */
	u8_t dst_offset;
} __packed;

static bool cts_ct_known;

static struct cts_local_time_info local_time_info;

static void ct_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("Current Time notifications %s",
			notif_enabled ? "enabled" : "disabled");
}

static int get_current_time(struct cts_current_time *ct)
{
	struct timespec ts;
	struct tm tm;
	int res;

	res = clock_gettime(CLOCK_REALTIME, &ts);

	if (res != 0) {
		return res;
	}

	gmtime_r(&ts.tv_sec, &tm);

	if (cts_ct_known) {
		ct->year = sys_cpu_to_le16(tm.tm_year + 1900);
		ct->month = tm.tm_mon + 1;

		/* Day of the month */
		ct->day = tm.tm_mday;

		/* Day of the week */
		if (tm.tm_wday == 0) {
			ct->day_of_week = CTS_WDAY_MAX;
		} else {
			ct->day_of_week = tm.tm_wday;
		}
	} else {
		ct->year = CTS_YEAR_UNKNOWN;
		ct->month = CTS_MONTH_UNKNOWN;
		ct->day = CTS_DAY_UNKNOWN;
		ct->day_of_week = CTS_WDAY_UNKNOWN;
	}

	ct->hours = tm.tm_hour;
	ct->minutes = tm.tm_min;
	ct->seconds = tm.tm_sec;

	ct->fractions_256 = ((ts.tv_nsec / MSEC_PER_SEC) * 256) / USEC_PER_SEC;

	ct->adjust_reason = 0;

	return 0;
}

static ssize_t read_current_time(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		void *buf, u16_t len, u16_t offset)
{
	struct cts_current_time ct = {0};

	int ret = get_current_time(&ct);
	if (ret != 0) {
		LOG_ERR("Could not read current time");
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ct, sizeof(ct));
}

#ifdef CONFIG_BT_GATT_CTS_CHAR_CT_WR
static ssize_t write_current_time(struct bt_conn *conn,
		const struct bt_gatt_attr *attr, const void *buf,
		u16_t len, u16_t offset, u8_t flags)
{
	struct cts_current_time *ct = (struct cts_current_time *)buf;
	struct tm tm;
	struct timespec tp;

	LOG_INF("Current time write");

	if (len != sizeof(*ct)) {
		LOG_ERR("write length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_ERR("write offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	ct->year = sys_le16_to_cpu(ct->year);

	if ((ct->year < CTS_YEAR_MIN) || (ct->year > CTS_YEAR_MAX)) {
		LOG_ERR("year");
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if ((ct->month < CTS_MONTH_MIN) || (ct->month > CTS_MONTH_MAX)) {
		LOG_ERR("month");
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if ((ct->day < CTS_DAY_MIN) || (ct->day > CTS_DAY_MAX)) {
		LOG_ERR("day");
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if (ct->hours > 23) {
		LOG_ERR("hours");
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if (ct->minutes > 59) {
		LOG_ERR("minutes");
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if (ct->seconds > 59) {
		LOG_ERR("seconds");
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if (ct->day_of_week > 7) {
		LOG_ERR("day of week");
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	tm.tm_sec = ct->seconds;
	tm.tm_min = ct->minutes;
	tm.tm_hour = ct->hours;
	tm.tm_mday = ct->day;
	tm.tm_mon = ct->month - 1;
	tm.tm_year = ct->year - 1900;
	tm.tm_wday = ct->day_of_week;

	/* tm.tm_yday is not used */;

	tm.tm_isdst = 0;

	tp.tv_sec = timeutil_timegm(&tm);
	if (tp.tv_sec == -1) {
		LOG_ERR("could not adjust time");
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	tp.tv_nsec = ((ct->fractions_256 * USEC_PER_SEC) / 256) * MSEC_PER_SEC;

	clock_settime(CLOCK_REALTIME, &tp);

	cts_ct_known = true;

	LOG_INF("Write Current Time Success");

	return len;
}
#endif

static ssize_t read_local_time_info(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		void *buf, u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
			&local_time_info, sizeof(local_time_info));
}

/* Current Time Service Declaration */
BT_GATT_SERVICE_DEFINE(cts_svc,
		BT_GATT_PRIMARY_SERVICE(BT_UUID_CTS),
		BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ
#ifdef CONFIG_BT_GATT_CTS_CHAR_CT_WR
#ifdef CONFIG_BT_GATT_CTS_CHAR_CT_WR_AUTHEN
		| BT_GATT_PERM_WRITE_AUTHEN
#else
		| BT_GATT_PERM_WRITE
#endif
#endif
		,
		read_current_time,
#ifdef CONFIG_BT_GATT_CTS_CHAR_CT_WR
		write_current_time,
#else
		NULL,
#endif
		NULL),
		BT_GATT_CCC(ct_ccc_cfg_changed,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
		BT_GATT_CHARACTERISTIC(BT_UUID_CTS_LOCAL_TIME_INFO,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ, read_local_time_info, NULL, NULL),
/*		BT_GATT_CHARACTERISTIC(BT_UUID_GATT_RTI, BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_NONE, NULL, NULL, NULL),*/
		);

static int cts_init(struct device *dev)
{
	ARG_UNUSED(dev);

	cts_ct_known = false;

	local_time_info.time_zone = 0;
	local_time_info.dst_offset = CTS_DST_OFFSET_STD_TIME;

	return 0;
}

int bt_gatt_cts_notify(u8_t adjust_reason)
{
	struct cts_current_time ct = {0};
	int ret;

	cts_ct_known = true;

	ret = get_current_time(&ct);
	if (ret != 0) {
		LOG_ERR("Could not get current time");
	}

	ct.adjust_reason = adjust_reason;

	ret = bt_gatt_notify(NULL, &cts_svc.attrs[1], &ct, sizeof(ct));

	return ret == -ENOTCONN ? 0 : ret;
}

SYS_INIT(cts_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
