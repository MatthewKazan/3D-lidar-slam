#!/usr/bin/env python3

import subprocess
import signal
import sys


def custom_sigint_handler(sig, frame):
    """
    Custom SIGINT handler to override the default behavior of rosbridge_websocket.
    Needed because rosbridge_websocket calls shutdown on SIGINT, which can cause
    then also the websocket node calls shutdown, leading to a double shutdown.

    """

    print("Overriding SIGINT: no second shutdown")
    # Exit immediately – do not let rosbridge_websocket call shutdown again
    sys.exit(0)


def main():
    # Set up the custom SIGINT handler in this process (the node's process)
    signal.signal(signal.SIGINT, custom_sigint_handler)

    # Build the command to run the real rosbridge_websocket node
    command = ["ros2", "run", "rosbridge_server", "rosbridge_websocket"]
    # Pass any command-line arguments from the launch file on to rosbridge_websocket
    command += sys.argv[1:]

    # Execute the real node
    subprocess.run(command)


if __name__ == '__main__':
    main()
