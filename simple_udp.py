#!/usr/bin/env python3
import socket
import time

# Максимально простой UDP приемник
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

print("Trying to bind to port 3334...")
try:
    sock.bind(('', 3334))
    print("✅ Successfully bound to port 3334")
    sock.settimeout(2.0)
    
    for i in range(10):
        try:
            data, addr = sock.recvfrom(1024)
            print(f"📦 #{i+1}: {len(data)} bytes from {addr}")
            print(f"    Data: {data.hex()}")
        except socket.timeout:
            print(f"⏰ Timeout #{i+1}")
        except Exception as e:
            print(f"❌ Error: {e}")
            
except Exception as e:
    print(f"❌ Bind failed: {e}")
finally:
    sock.close()
    print("Socket closed")