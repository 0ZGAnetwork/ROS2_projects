import paho.mqtt.client as mqtt
from pydantic import BaseModel, ValidationError

class TemperatureData(BaseModel):
    sensor_name: str
    temperature: float

class HumidityData(BaseModel):
    sensor_name: str
    humidity: float

BROKER = '127.0.0.1'  # localhost

def on_message(client, userdata, msg):
    payload = msg.payload.decode()

    try:
        if msg.topic == "sensor/temperature":
            data = TemperatureData.parse_raw(payload)
            print(f"[TEMP] Sensor {data.sensor_name}: {data.temperature:.2f} °C")

        elif msg.topic == "sensor/humidity":
            data = HumidityData.parse_raw(payload)
            print(f"[HUM]  Sensor {data.sensor_name}: {data.humidity:.2f} %")

    except ValidationError as e:
        print("Invalid data received:", e)

client = mqtt.Client()
client.on_message = on_message

client.connect(BROKER, 1883, 60)
client.subscribe("sensor/temperature")
client.subscribe("sensor/humidity")

try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nStopping monitoring unit...")
finally:
    client.disconnect()
    print("Disconnected from broker.")
