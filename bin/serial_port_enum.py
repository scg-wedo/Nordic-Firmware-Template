#!/usr/bin/env python3

import sys


# noinspection PyGlobalUndefined
def init():
    global BLEDriver, Flasher
    from pc_ble_driver_py import config

    config.__conn_ic_id__ = "NRF52"
    # noinspection PyUnresolvedReferences
    from pc_ble_driver_py.ble_driver import BLEDriver, Flasher


def main():
    descs = BLEDriver.enum_serial_ports()
    print("enum_serial_ports: {} serial ports found".format(len(descs)))
    for i, d in enumerate(descs):
        print("\nSerial port #{}:".format(i))
        print("|")
        print('|-  Port: "{}"'.format(d.port))
        print('|-  Manufacturer: "{}"'.format(d.manufacturer))
        print('|-  Serial Number: "{}"'.format(d.serial_number))
        print('|-  PnP ID: "{}"'.format(d.pnp_id))
        print('|-  Location ID: "{}"'.format(d.location_id))
        print('|-  Vendor ID: "{}"'.format(d.vendor_id))
        print('|_  Product ID: "{}"'.format(d.product_id))


if __name__ == "__main__":
    port = None
    init()
    main()
    quit()
