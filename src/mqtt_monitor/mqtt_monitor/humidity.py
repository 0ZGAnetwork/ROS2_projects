import random
import time
import json
import paho.mqtt.client as mqtt
from pydantic import BaseModel
import sys

class HumidityData(BaseModel):
    sensor_name: str
    humidity: float

BROKER = '127.0.0.1'  # localhost
TOPIC = 'sensor/humidity'

def run(sensor_name):
    client = mqtt.Client()
    client.connect(BROKER, 1883, 60) #broker, port mqtt, keepalive

    while True:
        value = random.uniform(20, 100)
        data = HumidityData(
            sensor_name=sensor_name,
            humidity=value
        )
        json_data = data.json()
        client.publish(TOPIC, json_data)
        print(f"Sent: {json_data}")
        time.sleep(2)


if __name__ == '__main__':
    
    sensor_name = sys.argv[1] if len(sys.argv) > 1 else "H1"

    try:
        run(sensor_name)
    except KeyboardInterrupt:
        print("\nStopping humidity sensor...")
    finally:
        print("Humidity sensor stopped.")
        sys.exit(0)