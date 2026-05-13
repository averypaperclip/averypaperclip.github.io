import asyncio
import logging
from bleak import BleakClient
import websockets

CHAR_UUID  = "abcdefab-1234-1234-1234-abcdefabcdef"
ESP32_ADDR = "54ED5C9B-7EDF-7D6C-7383-68CE44B59822"

# Silence the noisy InvalidUpgrade messages from browsers hitting
# http://localhost:8765
logging.getLogger("websockets.server").setLevel(logging.ERROR)

clients = set()
main_loop = None  # set in main()

async def ws_handler(websocket):
    print("WebSocket client connected")
    clients.add(websocket)
    try:
        async for _ in websocket:
            pass  # we don't expect client messages, just keep the connection alive
    finally:
        clients.discard(websocket)
        print("WebSocket client disconnected")

async def broadcast(line: str):
    if not clients:
        return
    await asyncio.gather(
        *(client.send(line) for client in list(clients)),
        return_exceptions=True
    )

async def ble_task():
    def notification_handler(_, data: bytearray):
        line = data.decode(errors="ignore").strip()
        if line:
            # Schedule the broadcast on the main loop (callback runs
            asyncio.run_coroutine_threadsafe(broadcast(line), main_loop)

    while True:
        try:
            print(f"Connecting to {ESP32_ADDR}...")
            async with BleakClient(ESP32_ADDR) as client:
                print("Connected over BLE.")
                await client.start_notify(CHAR_UUID, notification_handler)
                # Stay connected; BleakClient will detect disconnects via the context manager
                while client.is_connected:
                    await asyncio.sleep(1)
                print("BLE disconnected, reconnecting...")
        except Exception as e:
            print(f"BLE error: {e} - retrying in 5s")
            await asyncio.sleep(5)

async def main():
    global main_loop
    main_loop = asyncio.get_running_loop()

    async with websockets.serve(ws_handler, "localhost", 8765):
        print("WebSocket server listening on ws://localhost:8765")
        await ble_task()

if __name__ == "__main__":
    asyncio.run(main())