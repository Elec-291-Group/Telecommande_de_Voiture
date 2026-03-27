import asyncio
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "JDY-23"

def handle_data(sender, data):
    print(data.decode("utf-8", errors="replace"), end="")

async def main():
    print(f"Scanning for {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
    if device is None:
        print("Device not found.")
        return

    print(f"Found {DEVICE_NAME} at {device.address}. Connecting...")
    async with BleakClient(device) as client:
        print("Connected! Discovering characteristics...\n")

        notify_uuid = None
        for service in client.services:
            for char in service.characteristics:
                props = ",".join(char.properties)
                print(f"  UUID: {char.uuid}  properties: {props}")
                if "notify" in char.properties:
                    notify_uuid = char.uuid

        if notify_uuid is None:
            print("\nNo notify characteristic found.")
            return

        write_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"

        print(f"\nSubscribing to {notify_uuid}...")
        await client.start_notify(notify_uuid, handle_data)
        print("Waiting 2s then writing test message to JDY-23...\n")
        await asyncio.sleep(2)
        await client.write_gatt_char(write_uuid, b"hello\r\n")
        print("Wrote 'hello' to JDY-23. Waiting for echo...\n")
        await asyncio.sleep(10)
        await client.stop_notify(notify_uuid)

asyncio.run(main())
