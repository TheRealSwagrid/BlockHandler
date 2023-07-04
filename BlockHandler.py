#!/usr/bin/env python
import signal
import sys
import time
from copy import copy
from time import sleep

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class BlockHandler(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.funtionality = {"next_block": None}
        self.max_vel = 0.25
        self.acc = 0.002

    def get_next_block(self, params: dict):
        pos = [0,0,0]
        if self.funtionality["next_block"] is not None:
            pos = self.funtionality["next_block"]().position

        return {"Position3D": pos}

    def loop(self):
        pass


if __name__ == '__main__':
    # Needed for properly closing when process is being stopped with SIGTERM signal
    def handler(signum, frame):
        print("[Main] Received SIGTERM signal")
        listener.kill()
        quit(1)


    try:
        port = None
        if len(sys.argv[1:]) > 0:
            port = int(sys.argv[1])
        server = VirtualCapabilityServer(port)
        listener = BlockHandler(server)
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
