import asyncio


import sys
print("This script is using this Python executable:")
print(sys.executable)

# --- Your existing code ---
from mavsdk import System
# ... the rest of your script

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    while True:
        await asyncio.sleep(1)


asyncio.run(run())
