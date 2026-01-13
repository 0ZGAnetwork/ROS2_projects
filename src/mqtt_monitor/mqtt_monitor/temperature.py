import random
import time
import json
import paho.mqtt.client as mqtt
from pydantic import BaseModel
import sys

class TemperatureData(BaseModel):
    sensor_name: str
    temperature: float

BROKER = '127.0.0.1'  # localhost
TOPIC = 'sensor/temperature'

def run(sensor_name):
    client = mqtt.Client()
    client.connect(BROKER, 1883, 60) #broker, port mqtt, keepalive

    while True:
        value = random.uniform(0, 30)
        data = TemperatureData(
            sensor_name=sensor_name,
            temperature=value
        )
        json_data = data.json()
        client.publish(TOPIC, json_data)
        print(f"Sent: {json_data}")
        time.sleep(2)


if __name__ == '__main__':
    sensor_name = sys.argv[1] if len(sys.argv) > 1 else "T1"

    try:
        run(sensor_name)
    except KeyboardInterrupt:
        print("\nStopping temperature sensor...")
    finally:
        print("Temperature sensor stopped.")
        sys.exit(0)

    #python temperature_sensor.py T1