from game_controller.gc_threaded import ThreadedGameController
import time

# Create and start threaded game controller
gc = ThreadedGameController()
gc.start()


while True:
    # Get current game state
    state = gc.get_state()
    print(f"Command: {state['command']}")
    print(f"Can play: {state['can_play']}")
    # time.sleep(1)
else:
    # When done
    gc.stop()
    gc.join()
