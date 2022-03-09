#!/usr/bin/env python3

import sys
import time
import logging
from queue import Queue, Empty
from pc_ble_driver_py.observers import *

TARGET_DEV_NAME = "DOLABS_SCALE"
CONNECTIONS = 1
CFG_TAG = 1


def init():
    # noinspection PyGlobalUndefined
    global config, BLEUUIDBase, BLEDriver, BLEAdvData, BLEEvtID, BLEAdapter, BLEEnableParams, BLEGapTimeoutSrc, BLEUUID, BLEConfigCommon, BLEConfig, BLEConfigConnGatt, BLEGapScanParams
    from pc_ble_driver_py import config

    config.__conn_ic_id__ = "NRF52"
    # noinspection PyUnresolvedReferences
    from pc_ble_driver_py.ble_driver import (
        BLEUUIDBase,
        BLEDriver,
        BLEAdvData,
        BLEEvtID,
        BLEEnableParams,
        BLEGapTimeoutSrc,
        BLEUUID,
        BLEGapScanParams,
        BLEConfigCommon,
        BLEConfig,
        BLEConfigConnGatt,
    )

    # noinspection PyUnresolvedReferences
    from pc_ble_driver_py.ble_adapter import BLEAdapter

    global nrf_sd_ble_api_ver
    nrf_sd_ble_api_ver = config.sd_api_ver_get()


class HRCollector(BLEDriverObserver, BLEAdapterObserver):
    def __init__(self, adapter):
        super(HRCollector, self).__init__()
        self.adapter = adapter
        self.conn_q = Queue()
        self.adapter.observer_register(self)
        self.adapter.driver.observer_register(self)
        self.adapter.default_mtu = 250
        global BASE_UUID, NUS_TX_UUID, NUS_RX_UUID

        BASE_UUID   = BLEUUIDBase([0x6E, 0x40, 0x00, 0x00, 0xB5, 0xA3, 0xF3, 0x93,
                                   0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E], 0x02) # 0x02 is the type BLE_UUID_TYPE_VENDOR_BEGIN

        NUS_TX_UUID = BLEUUID(0x0003, BASE_UUID)
        NUS_RX_UUID = BLEUUID(0x0002, BASE_UUID)

    def open(self):
        self.adapter.driver.open()
        gatt_cfg = BLEConfigConnGatt()
        gatt_cfg.att_mtu = self.adapter.default_mtu
        gatt_cfg.tag = CFG_TAG
        self.adapter.driver.ble_cfg_set(BLEConfig.conn_gatt, gatt_cfg)
        self.adapter.driver.ble_enable()

    def close(self):
        self.adapter.driver.close()

    def connect_and_discover(self):
        scan_duration = 5
        params = BLEGapScanParams(interval_ms=200, window_ms=150, timeout_s=scan_duration)

        self.adapter.driver.ble_gap_scan_start(scan_params=params)

        try:
            new_conn = self.conn_q.get(timeout=scan_duration)
            self.adapter.service_discovery(new_conn)

            self.adapter.enable_notification(
                new_conn, BLEUUID(BLEUUID.Standard.battery_level)
            )

            self.adapter.enable_notification(
                new_conn, NUS_TX_UUID
            )

            return new_conn
        except Empty:
            print(f"No heart rate collector advertising with name {TARGET_DEV_NAME} found.")
            return None

    def on_gap_evt_connected(
        self, ble_driver, conn_handle, peer_addr, role, conn_params
    ):
        print("New connection: {}".format(conn_handle))
        self.conn_q.put(conn_handle)

    def on_gap_evt_disconnected(self, ble_driver, conn_handle, reason):
        print("Disconnected: {} {}".format(conn_handle, reason))

    def on_gap_evt_adv_report(
        self, ble_driver, conn_handle, peer_addr, rssi, adv_type, adv_data
    ):
        if BLEAdvData.Types.complete_local_name in adv_data.records:
            dev_name_list = adv_data.records[BLEAdvData.Types.complete_local_name]

        elif BLEAdvData.Types.short_local_name in adv_data.records:
            dev_name_list = adv_data.records[BLEAdvData.Types.short_local_name]

        else:
            return

        dev_name = "".join(chr(e) for e in dev_name_list)
        address_string = "".join("{0:02X}".format(b) for b in peer_addr.addr)
        print(
            "Received advertisment report, address: 0x{}, device_name: {}".format(
                address_string, dev_name
            )
        )

        if dev_name == TARGET_DEV_NAME:
            self.adapter.connect(peer_addr, tag=CFG_TAG)

    def on_notification(self, ble_adapter, conn_handle, uuid, data):
        # print("Connection: {}, {} = {}".format(conn_handle, uuid, data))
        if uuid == NUS_TX_UUID:
            data_string = "".join([chr(value) for value in data])
            print("Nordic UART Service: ", data_string)



def main(selected_serial_port):
    print("Serial port used: {}".format(selected_serial_port))
    driver = BLEDriver(
        serial_port=selected_serial_port, auto_flash=False, baud_rate=1000000, log_severity_level="info"
    )

    adapter = BLEAdapter(driver)
    collector = HRCollector(adapter)
    collector.open()
    conn = collector.connect_and_discover()

    if conn is not None:
        adapter.write_req(conn, NUS_RX_UUID, [0x02])
        while conn is not None:
            continue

    collector.close()


def item_choose(item_list):
    for i, it in enumerate(item_list):
        print("\t{} : {}".format(i, it))
    print(" ")

    while True:
        try:
            choice = int(input("Enter your choice: "))
            if (choice >= 0) and (choice < len(item_list)):
                break
        except Exception:
            pass
        print("\tTry again...")
    return choice


if __name__ == "__main__":
    logging.basicConfig(
        level="INFO",
        format="%(asctime)s [%(thread)d/%(threadName)s] %(message)s",
    )
    serial_port = None
    if len(sys.argv) < 1:
        print("Please specify serial port")
        exit(1)
    init()
    if len(sys.argv) == 2:
        serial_port = sys.argv[1]
    else:
        descs = BLEDriver.enum_serial_ports()
        choices = ["{}: {}".format(d.port, d.serial_number) for d in descs]
        choice = item_choose(choices)
        serial_port = descs[choice].port
    main(serial_port)
    quit()
