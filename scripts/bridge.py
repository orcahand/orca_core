#!/usr/bin/env python3
"""
Bridge script that connects a local orca_core API to a remote orca-express server.

Runs on the robot computer. Connects outbound via WebSocket to the orca-express
server and relays commands to the local orca_core HTTP API.

Usage:
    python bridge.py                                          # defaults
    python bridge.py --server wss://orca-express.onrender.com # custom server
    python bridge.py --core http://localhost:8000              # custom orca_core URL
"""

import argparse
import asyncio
import json
import aiohttp
import websockets

DEFAULT_SERVER = "wss://orca-express.onrender.com/ws/orca-core"
DEFAULT_CORE_URL = "http://localhost:8000"
HEARTBEAT_INTERVAL = 10  # seconds
RECONNECT_DELAY = 5      # seconds


async def relay_command(session, core_url, method, path, body=None):
    """Forward a command to local orca_core and return the response."""
    url = f"{core_url}{path}"
    try:
        if method == "GET":
            async with session.get(url) as resp:
                return await resp.json()
        elif method == "POST":
            async with session.post(url, json=body) as resp:
                return await resp.json()
        elif method == "PUT":
            async with session.put(url, json=body) as resp:
                return await resp.json()
        elif method == "DELETE":
            async with session.delete(url, json=body) as resp:
                return await resp.json()
        else:
            return {"error": f"Unsupported method: {method}"}
    except Exception as e:
        return {"error": str(e)}


async def heartbeat(ws):
    """Periodically send heartbeat to keep connection alive."""
    while True:
        try:
            await asyncio.sleep(HEARTBEAT_INTERVAL)
            await ws.send(json.dumps({"action": "heartbeat"}))
        except Exception:
            break


async def listen_events(core_url, express_ws):
    """Subscribe to orca_core /ws/events and forward events to orca-express."""
    events_url = core_url.replace("http://", "ws://").replace("https://", "wss://") + "/ws/events"
    while True:
        try:
            async with websockets.connect(events_url) as ws:
                print(f"Event listener connected to {events_url}")
                async for message in ws:
                    try:
                        event = json.loads(message)
                        await express_ws.send(json.dumps({
                            "action": "robot_event",
                            "data": event,
                        }))
                    except Exception as e:
                        print(f"Event forward error: {e}")
        except (websockets.ConnectionClosed, ConnectionRefusedError, OSError) as e:
            print(f"Event listener disconnected: {e}. Reconnecting in {RECONNECT_DELAY}s...")
        except Exception as e:
            print(f"Event listener error: {e}. Reconnecting in {RECONNECT_DELAY}s...")
        await asyncio.sleep(RECONNECT_DELAY)


async def run_bridge(server_url, core_url):
    """Main bridge loop — connect, identify, listen for commands, relay."""
    async with aiohttp.ClientSession() as session:
        while True:
            try:
                print(f"Connecting to {server_url}...")
                async with websockets.connect(server_url) as ws:
                    print("Connected to orca-express server")

                    # Identify as bridge
                    await ws.send(json.dumps({
                        "action": "robot_identify",
                        "type": "bridge",
                    }))
                    print("Sent bridge identification")

                    # Start heartbeat task
                    hb_task = asyncio.create_task(heartbeat(ws))
                    # Start event listener (forwards orca_core events to orca-express)
                    events_task = asyncio.create_task(listen_events(core_url, ws))

                    try:
                        async for message in ws:
                            try:
                                data = json.loads(message)
                            except json.JSONDecodeError:
                                continue

                            if data.get("action") == "relay_command":
                                request_id = data.get("requestId")
                                method = data.get("method", "GET")
                                path = data.get("path", "/")
                                body = data.get("body")

                                response = await relay_command(session, core_url, method, path, body)

                                await ws.send(json.dumps({
                                    "requestId": request_id,
                                    "response": response,
                                }))
                    finally:
                        hb_task.cancel()
                        events_task.cancel()
                        try:
                            await hb_task
                        except asyncio.CancelledError:
                            pass
                        try:
                            await events_task
                        except asyncio.CancelledError:
                            pass

            except (websockets.ConnectionClosed, ConnectionRefusedError, OSError) as e:
                print(f"Connection lost: {e}. Reconnecting in {RECONNECT_DELAY}s...")
            except Exception as e:
                print(f"Unexpected error: {e}. Reconnecting in {RECONNECT_DELAY}s...")

            await asyncio.sleep(RECONNECT_DELAY)


def main():
    parser = argparse.ArgumentParser(description="ORCA Bridge — relay commands from orca-express to local orca_core")
    parser.add_argument("--server", default=DEFAULT_SERVER, help=f"orca-express WebSocket URL (default: {DEFAULT_SERVER})")
    parser.add_argument("--core", default=DEFAULT_CORE_URL, help=f"Local orca_core HTTP URL (default: {DEFAULT_CORE_URL})")
    args = parser.parse_args()

    print(f"ORCA Bridge starting")
    print(f"  Server: {args.server}")
    print(f"  Core:   {args.core}")

    asyncio.run(run_bridge(args.server, args.core))


if __name__ == "__main__":
    main()
