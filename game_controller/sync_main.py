from game_controller.gc_sync import SyncGameController


# Create and start sync game controller
gc = SyncGameController()
gc.start()

print("entering synced state controller")

# Main loop
while True:
    # Update state (non-blocking)

    if gc.update():
        state = gc.get_state()
        print(state)

    # Do other work...

    # When done
gc.stop()
