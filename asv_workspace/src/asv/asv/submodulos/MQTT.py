import threading
import paho.mqtt.client as mqtt
import subprocess  # For executing a shell command
import os
import signal
import psutil

class MQTT(object):
    def __init__(self, name, addr='bender.us.es', port=1883, timeout=60, topics2suscribe=None, on_message=None, on_disconnect=None,user=None,password=None):

        self.client = mqtt.Client(name)

        if topics2suscribe is None:
            topics2suscribe = []
        self.topics2suscribe = topics2suscribe

        if on_message is None:
            self.on_message = self.on_message
        else:
            self.on_message = on_message

        if on_disconnect is None:
            self.on_disconnect = self.on_disconnect
        else:
            self.on_disconnect = on_disconnect
        # Con esto se puede mejorar la seguridad del MQTT si el broker esta configurado para eso
        self.client.username_pw_set(user, password)

        self._mqtt_thread = threading.Thread(target=self.mqtt_thread, args=(addr, port, timeout,))
        self._mqtt_thread.start()

    def mqtt_thread(self, addr='bender.us.es', port=1883, timeout=60):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(addr, port, timeout)
        self.client.loop_forever()
    
    def on_connect(client, userdata, flags, rc):
        print(f"Client {userdata} connected! " + str(rc))

    def on_connect(self, _client, _, __, rc):
        print("Connected to MQTT server")
        for topic in self.topics2suscribe:
            self.client.subscribe(topic)

    def on_disconnect(client, userdata, _, rc):
        print("client disconnected ok")

    @staticmethod
    def on_message(_client, user_data, msg):
        message = bool(msg.payload)
        print(message)

    def send_new_msg(self, msg, topic="coordinator"):
        #while not self.client.is_connected():
            #continue
        self.client.publish(topic, msg)

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()