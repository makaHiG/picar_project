import asyncio
import json
import socket
import websockets

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

clients = set()

# UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

async def websocket_handler(websocket):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)

async def udp_listener():
    loop = asyncio.get_running_loop()

    while True:
        data, addr = await loop.sock_recvfrom(sock, 65535)

        message = data.decode()
        print(f"Received from UDP: {message}")
        # Forward to all browsers
        for client in clients.copy():
            try:
                await client.send(message)
            except:
                pass

async def main():
    ws_server = await websockets.serve(
        websocket_handler,
        "0.0.0.0",
        8765
    )

    await udp_listener()

asyncio.run(main())