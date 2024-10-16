from VisionClient import VisionClient

import time

client = VisionClient()
client.start()

try:
    while True:
        time.sleep(1)
        print(client.frame)
        #exit()
except KeyboardInterrupt:
    client.stop()
    client.join()
    