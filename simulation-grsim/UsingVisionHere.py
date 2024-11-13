import time
from VisionClient.Vision import Vision


"""example of how to use visin here"""


def main():
    # Create config object needed by Vision class
    class Config:
        def __init__(self):
            self.config = {
                "network": {"vision_port": 10006, "multicast_ip": "224.5.23.2"},
                "match": {"team_side": "left"},
            }

    # Create Vision client
    vision = Vision(Config())

    # Start the vision client
    vision.start()

    try:
        while True:
            # Use the built-in formatted print function
            vision.print_formatted_vision_data()
            time.sleep(0.016)  # ~60Hz update rate

    except KeyboardInterrupt:
        print("Shutting down...")
        vision.stop()
        vision.join()


if __name__ == "__main__":
    main()
