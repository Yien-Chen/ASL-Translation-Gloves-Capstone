import asyncio
from bleak import BleakClient

# UUIDs (Ensure these match your Arduino's BLE UUIDs)
SERVICE_UUID = "180C"
CHARACTERISTIC_UUID = "2A57"

# Replace this with your Arduino 33 BLE Sense MAC address
DEVICE_MAC_ADDRESS = "CE:B5:64:61:5B:E8"  # <-- Update this with your MAC address

async def connect_and_read():
    print(f"Connecting to Arduino BLE Sensor at {DEVICE_MAC_ADDRESS}...")

    # Connect to the device directly using the MAC address
    async with BleakClient(DEVICE_MAC_ADDRESS) as client:
        print("Connected! Reading sensor data...")

        def notification_handler(sender, data):
            print(f"Received: {data.decode('utf-8')}")

        # Subscribe to the characteristic
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

        try:
            while True:
                await asyncio.sleep(1)  # Keep running
        except KeyboardInterrupt:
            print("Disconnecting...")

        await client.stop_notify(CHARACTERISTIC_UUID)

# Run the async function
asyncio.run(connect_and_read())

