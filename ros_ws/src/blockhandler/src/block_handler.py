#!/usr/bin/env python
from enum import Enum
from random import random

import rospy
import std_msgs
import tf
from tf import TransformListener
import visualization_msgs.msg
from visualization_msgs.msg import Marker
from AbstractVirtualCapability import VirtualCapabilityServer

from BlockHandler import BlockHandler


class RosBlockHandler:

    def __init__(self):
        self.a = 0
        self.pub = rospy.Publisher("/robot", Marker, queue_size=10)
        self.blocks = list()
        for i in range(10):
            self.blocks.append(Block(i, [5., 0., i*.1], [0, 0, 0, 1]))

    def publish_all(self):
        for block in self.blocks:
            self.pub.publish(block.as_msg())

    def get_next_block(self):
        for block in reversed(self.blocks):
            if block.status is Block_Status.not_moved:
                return block

class Block_Status(Enum):
    not_moved = 0
    on_robot = 1
    on_copter = 2
    in_place = 3


class Block:

    def __init__(self, id: int = 0, pos: list = None, rot: list = None):
        self.mesh = "package://blockhandler/meshes/BIG_LEGO.dae"
        self.id = id
        self.position = pos
        self.rotation = rot
        self.scale = .01
        self.status = Block_Status.not_moved
        self.color_r = random()
        self.color_g = random()
        self.color_b = random()

    def as_msg(self) -> Marker:
        marker = Marker()
        marker.id = self.id
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"block_{self.id}"
        marker.lifetime = rospy.Duration(0)
        marker.color.r = self.color_r
        marker.color.g = self.color_g
        marker.color.b = self.color_b
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = self.rotation[0]
        marker.pose.orientation.y = self.rotation[1]
        marker.pose.orientation.z = self.rotation[2]
        marker.pose.orientation.w = self.rotation[3]
        # Scale down
        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale
        marker.color.a = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = self.mesh
        return marker

if __name__ == '__main__':
    rospy.init_node('rosnode')
    rate = rospy.Rate(40)

    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))
    bh = BlockHandler(server)
    bh.start()

    block_handler = RosBlockHandler()
    block_handler.publish_all()

    bh.funtionality["next_block"] = block_handler.get_next_block

    while not rospy.is_shutdown():
        block_handler.publish_all()
        rate.sleep()