.. _peripheral_cts:

Bluetooth: Peripheral CTS
#########################

Overview
********

Similar to the :ref:`Peripheral <ble_peripheral>` sample, except that this
application specifically exposes the CTA (Current Time) GATT Service. Once a device
connects it the current time can be written and read from the service.


Requirements
************

* BlueZ running on the host, or
* A board with BLE support

Building and Running
********************

This sample can be found under :zephyr_file:`samples/bluetooth/peripheral_cts` in the
Zephyr tree.

See :ref:`bluetooth samples section <bluetooth-samples>` for details.
