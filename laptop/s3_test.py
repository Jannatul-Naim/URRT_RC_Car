"""Talk to the ESP32-S3 directly, without the dashboard, for 15 seconds.

    python s3_test.py                 uses S3_HOST from config.py
    python s3_test.py 192.168.0.140   or give the address

Stop server.py first: the S3 accepts only one connection at a time.
Prints what the S3 sends, sends a "stop" drive packet 25 times a second
(like the dashboard does), and says exactly how and when the link ends.
"""
import asyncio
import sys
import time

from aiohttp import ClientSession, WSMsgType

import config

HOST = sys.argv[1] if len(sys.argv) > 1 else config.S3_HOST
URL = f"ws://{HOST}:{config.S3_WS_PORT}/"
SECONDS = 15


async def main():
    print(f"Connecting to {URL} ...")
    async with ClientSession() as session:
        try:
            ws = await session.ws_connect(URL, autoping=False, compress=0)
        except Exception as e:
            print(f"FAILED to connect: {e!r}")
            return
        print("Connected. Listening for", SECONDS, "seconds.\n")
        start = time.time()
        counts = {}

        async def sender():
            while not ws.closed:
                await ws.send_str("D,0,0,50")      # stop packet, keeps the watchdog fed
                await asyncio.sleep(0.04)

        task = asyncio.create_task(sender())
        await ws.send_str("T")
        try:
            while time.time() - start < SECONDS:
                try:
                    msg = await asyncio.wait_for(ws.receive(), timeout=2)
                except asyncio.TimeoutError:
                    print(f"{time.time() - start:5.1f}s  (nothing received for 2 s)")
                    continue
                t = time.time() - start
                if msg.type == WSMsgType.TEXT:
                    kind = msg.data.split(",")[0]
                    counts[kind] = counts.get(kind, 0) + 1
                    if kind != "T" or counts[kind] % 25 == 1:   # one telemetry line per second
                        print(f"{t:5.1f}s  {msg.data}")
                else:
                    print(f"{t:5.1f}s  LINK ENDED: {msg.type!r} data={msg.data!r} "
                          f"close_code={ws.close_code} error={ws.exception()!r}")
                    break
        finally:
            task.cancel()
            await ws.close()
        ran = time.time() - start
        print(f"\nRan {ran:.1f} s. Messages by type: {counts}")
        # Healthy = stayed up the whole time and telemetry kept coming (~25 per second)
        ok = ran >= SECONDS - 0.5 and counts.get("T", 0) >= 15 * SECONDS
        print("RESULT:", "link is healthy" if ok else "link is NOT healthy, send this output to Claude")


asyncio.run(main())
