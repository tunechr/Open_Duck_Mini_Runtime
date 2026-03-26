#!/usr/bin/env python3
"""Simple IMU client test to debug connection issues"""

import socket
import pickle
import time
import io

def test_imu_connection(host="192.168.0.139", port=1234):
    print(f"Testing connection to {host}:{port}")
    
    try:
        # Create socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)  # 5 second timeout
        
        # Connect
        print("Connecting...")
        sock.connect((host, port))
        print("Connected successfully!")
        
        buffer = b''
        count = 0
        
        while count < 10:  # Get 10 samples
            try:
                # Receive data
                chunk = sock.recv(1024)
                if not chunk:
                    print("Connection closed by server")
                    break
                
                buffer += chunk
                print(f"Received {len(chunk)} bytes, buffer size: {len(buffer)}")
                
                # Try to parse pickle objects from buffer
                while buffer:
                    try:
                        stream = io.BytesIO(buffer)
                        data = pickle.load(stream)
                        consumed = stream.tell()
                        buffer = buffer[consumed:]
                        
                        print(f"Sample {count + 1}: {data} (consumed {consumed} bytes)")
                        count += 1
                        
                        if count >= 10:
                            break
                            
                    except (pickle.UnpicklingError, EOFError):
                        # Need more data
                        break
                    except Exception as e:
                        print(f"Parse error: {e}")
                        break
                
                time.sleep(0.1)
                
            except socket.timeout:
                print("Timeout waiting for data")
                break
            except Exception as e:
                print(f"Error: {e}")
                break
        
        sock.close()
        print("Test completed")
        
    except Exception as e:
        print(f"Connection failed: {e}")

if __name__ == "__main__":
    test_imu_connection()
