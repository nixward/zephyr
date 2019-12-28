# Bluetooth GATT Current Time service

# Copyright (c) 2019 Nick Ward
# SPDX-License-Identifier: Apache-2.0

menuconfig BT_GATT_CTS
	bool "Enable GATT Current Time service"
	depends on POSIX_CLOCK
	default n

if BT_GATT_CTS

config BT_GATT_CTS_LOG_LEVEL
	int "Current Time service log level"
	depends on LOG
	range 0 4
	default 0
	help
	  Sets log level for the Current Time service.
	  Levels are:
	  0 OFF, do not write
	  1 ERROR, only write LOG_ERR
	  2 WARNING, write LOG_WRN in addition to previous level
	  3 INFO, write LOG_INF in addition to previous levels
	  4 DEBUG, write LOG_DBG in addition to previous levels

config BT_GATT_CTS_CHAR_CT_WR
	bool "Current Time characteristic write enable"
	default n

config BT_GATT_CTS_CHAR_CT_WR_AUTHEN
	bool "Authenticated requirement for Current Time characteristic write"
	depends on BT_GATT_CTS_CHAR_CT_WR
	select BT_SMP
	default y

endif # BT_GATT_CTS
