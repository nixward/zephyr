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

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#define LOG_LEVEL CONFIG_BT_GATT_CTS_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(cts);

/* CTS error definitions */
#define CTS_ERR_DATA_FIELD_IGNORED			0x80

struct cts_current_time {
	/* Exact Time 256 */
	/* Day Date Time */
	/* Date Time */
	u16_t year;
	u8_t month;
	u8_t day;
	u8_t hours;
	u8_t minutes;
	u8_t seconds;
	/* Day of Week */
	u8_t day_of_week;
	u8_t fractions_256;
	/* Adjust reason - flags */
	u8_t adjust_reason;
} __packed;

static s64_t cts_epoch_ms_offset;

static void ct_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("Current Time notifications %s",
			notif_enabled ? "enabled" : "disabled");
}

static int set_current_time(struct cts_current_time *ct)
{
	struct tm cts_tm;
	s64_t epoch_ms = cts_epoch_ms_offset + k_uptime_get();

	if (epoch_ms < 0) {
		return -ENOTSUP;
	}

	time_t time = epoch_ms / 1000;

	gmtime_r(&time, &cts_tm);

	ct->year = sys_cpu_to_le16(cts_tm.tm_year);
	ct->month = cts_tm.tm_mon + 1;

	/* day of the month */
	ct->day = cts_tm.tm_mday;

	/* day of the week */
	ct->day_of_week = (cts_tm.tm_wday == 0) ? 7 : cts_tm.tm_wday;

	ct->hours = cts_tm.tm_hour;
	ct->minutes = cts_tm.tm_min;
	ct->seconds = cts_tm.tm_sec;
	ct->fractions_256 = ((epoch_ms % 1000) * 256) / 1000;

	//int tm_yday;        /* day in the year */
	//int tm_isdst;       /* daylight saving time */

	return 0;
}

static ssize_t read_current_time(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		void *buf, u16_t len, u16_t offset)
{
	struct cts_current_time ct = {0};

	int ret = set_current_time(&ct);
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
	struct cts_current_time ct;
	struct tm cts_tm;
	time_t time;
	s64_t fractions_1000;
	s64_t time_1000;

	if (len > sizeof(ct)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset + len > sizeof(ct)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	/* Don't allow byte writes to year field */
	if ((offset < 2) && (len < 2)) {
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	set_current_time(&ct);

	memcpy(&ct, buf, len);

	if (offset < 2) {
		ct.year = sys_cpu_to_le16(ct.year);
	}

	if ((ct.year < 1582) || (ct.year > 9999)) {
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if ((ct.month < 1) || (ct.month > 12)) {
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if ((ct.day < 1) || (ct.day > 31)) {
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if (ct.hours > 23) {
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if (ct.minutes > 59) {
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	if (ct.seconds > 59) {
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	cts_tm.tm_year = sys_cpu_to_le16(ct.year);
	cts_tm.tm_mon = ct.month - 1;
	cts_tm.tm_mday = ct.day;
	cts_tm.tm_hour = ct.hours;
	cts_tm.tm_min = ct.minutes;
	cts_tm.tm_sec = ct.seconds;

	cts_tm.tm_isdst = 0;

	time = mktime(&cts_tm);
	if (time == -1) {
		return BT_GATT_ERR(CTS_ERR_DATA_FIELD_IGNORED);
	}

	fractions_1000 = ct.fractions_256;
	fractions_1000 = (fractions_1000 * 1000) / 256;

	time_1000 = 1000 * time;

	cts_epoch_ms_offset = time_1000 + fractions_1000 - k_uptime_get();

	return len;
}
#endif

/* Current Time Service Declaration */
BT_GATT_SERVICE_DEFINE(cts_svc,
		BT_GATT_PRIMARY_SERVICE(BT_UUID_CTS),
		BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME, BT_GATT_CHRC_NOTIFY,
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
/*		BT_GATT_CHARACTERISTIC(BT_UUID_GATT_LTI, BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ, NULL, NULL, NULL),
		BT_GATT_CHARACTERISTIC(BT_UUID_GATT_RTI, BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_NONE, NULL, NULL, NULL),*/
		);

static int cts_init(struct device *dev)
{
	ARG_UNUSED(dev);

	cts_epoch_ms_offset = 0;

	return 0;
}

int bt_gatt_cts_notify(void)
{
	struct cts_current_time ct = {0};

	int ret = set_current_time(&ct);
	if (ret != 0) {
		LOG_ERR("Could not get current time");
	}

	ret = bt_gatt_notify(NULL, &cts_svc.attrs[1], &ct, sizeof(ct));

	return ret == -ENOTCONN ? 0 : ret;
}

SYS_INIT(cts_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
