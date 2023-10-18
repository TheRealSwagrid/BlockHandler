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
        self.funtionality = {"next_block": None, "attach_block": None, "all_blocks": None}
        self.max_vel = 0.25
        self.acc = 0.002
        self.block_dimensions = [0.11, 0.22, 0.11]

    def get_next_block(self, params: dict):
        pos = [0,0,0]
        nr = 0
        if self.funtionality["next_block"] is not None:
            block = self.funtionality["next_block"]()
            pos = [block.position.x, block.position.y, block.position.z]
            nr = block.id
        return {"Position3D": pos, "SimpleIntegerParameter": nr}

    def attach_block(self, params: dict):
        tf_str = params["SimpleStringParameter"]
        id = params["SimpleIntegerParameter"]
        if self.funtionality["attach_block"] is not None:
            self.funtionality["attach_block"](id, tf_str)
        return {"SimpleIntegerParameter": id}

    def detach_block(self, params: dict):
        id = params["SimpleIntegerParameter"]
        if self.funtionality["attach_block"] is not None:
            self.funtionality["attach_block"](id, None)
        return {"SimpleIntegerParameter": id}

    def get_all_blocks(self, params: dict):
        list_of_blocks = [[0.,1.,0.], [0.,3.,0.]]
        if self.funtionality["all_blocks"] is not None:
            list_of_blocks = self.funtionality["all_blocks"]()
        return {"ListOfPoints": list_of_blocks}

    def SetBlockDimensions(self, params: dict):
        new_dims = params["ListOfPoints"]
        self.block_dimensions = new_dims
        return self.GetBlockDimensions()

    def GetBlockDimensions(self, params: dict):
        return {"ListOfPoints": self.block_dimensions}

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
