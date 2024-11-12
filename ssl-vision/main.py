import json
import time
from new_client import Vision  # Assuming your Vision class is in vision.py


def get_config(config_file=None):
    if config_file:
        config = json.loads(open(config_file, "r").read())
    else:
        config = json.loads(open("config.json", "r").read())
    return config


class Main:
    def __init__(self) -> None:
        # Load Config
        self.config = get_config()

        # Input Layer - Vision
        self.ssl_vision = Vision(self)

    def start(self):
        """Start the main loop"""
        # Start vision
        self.ssl_vision.start()

        try:
            while True:
                # Print vision data periodically
                self.ssl_vision.print_formatted_vision_data()
                time.sleep(1)  # Wait for 1 second between updates

        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Stop all threads"""
        self.ssl_vision.stop()
        self.ssl_vision.join()


if __name__ == "__main__":
    game = Main()
    game.start()
