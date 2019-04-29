# fake_qp_inputs.py
import numpy as np
import vizier.mqttinterface as mqtt
import time
import json

MAX_ITER = 10000
PORT = 1884
HOST = '192.168.1.8'
all_ids = list(range(50))

request_link = 'fake_api/inputs'
mqtt_client = mqtt.MQTTInterface(port=PORT, host=HOST)
mqtt_client.start()


def create_request_link(node):
  """Creates the appropriate request channel for a given node
  Args:
      node (str):  Name of the node on which the request link should be created
  Returns:
      String representing the request link.  It is of the form <node_name>/requests
  """

  return '/'.join([node, 'requests'])

for iter in range(MAX_ITER):
  
  input_dict = {'inputs': {id: [0, 0] for id in all_ids}}
  mqtt_client.send_message(request_link, json.dumps(input_dict))
  time.sleep(0.01)
  print(input_dict)




