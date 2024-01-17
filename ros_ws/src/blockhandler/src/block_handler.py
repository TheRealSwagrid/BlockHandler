#!/usr/bin/env python
import os
from enum import Enum
from random import random

from geometry_msgs import msg
import rospy
import tf2_ros
import std_msgs
import tf
from tf import TransformListener
import visualization_msgs.msg

from visualization_msgs.msg import Marker
from AbstractVirtualCapability import VirtualCapabilityServer

from BlockHandler import BlockHandler

tfBuffer = None


class RosBlockHandler:

    def __init__(self):
        self.a = 0
        self.pub = rospy.Publisher("/robot", Marker, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.blocks = list()
        self.id_enumerator = 0
        """
        for i in range(10):
            self.blocks.append(Block(i, [-1., -1., i * .1], [0, 0, 0, 1]))
        for i in range(10):
            self.blocks.append(Block(i+10, [-1., -1.11, i * .1], [0, 0, 0, 1]))
        for i in range(10):
            self.blocks.append(Block(i+20, [-1., -1.22, i * .1], [0, 0, 0, 1]))
        """

    def next_id(self):
        self.id_enumerator += 1
        return self.id_enumerator

    def spawn(self, shape: list):
        b = Block(id=self.next_id(), pos=[-1., -1., 0.], rot=[0, 0, 0, 1], shape=shape,
                  mesh="package://blockhandler/meshes/cube1m.dae")
        self.blocks.append(b)
        return b

    def get_block(self, block_id: int):
        for b in self.blocks:
            if block_id == b.id:
                return b
        return self.blocks[int(block_id)]

    def all_blocks(self):
        return [[b.position.x, b.position.y, b.position.z] for b in self.blocks]

    def publish_all(self):
        for block in self.blocks:
            self.pub.publish(block.as_msg())
            self.br.sendTransform((block.position.x, block.position.y, block.position.z),
                                  (block.rotation.x, block.rotation.y, block.rotation.z, block.rotation.w),
                                  rospy.Time.now(), f"Block_{block.id}", "world")

    def get_next_block(self):
        for block in reversed(self.blocks):
            if block.status is Block_Status.not_moved:
                return block

    def attach_block(self, block_id: int, tf_pos: str):
        rospy.logerr(f"Block {block_id} on TF: {tf_pos}")
        block = self.get_block(block_id)
        if tf_pos:
            block.status = Block_Status.on_robot
        else:
            block.status = Block_Status.in_place
        block.tf_pos = tf_pos


class Block_Status(Enum):
    not_moved = 0
    on_robot = 1
    on_copter = 2
    in_place = 3


class Block:

    def __init__(self, id: int = 0, pos: list = None, rot: list = None, shape: list = None, mesh: str = None):
        if mesh is not None:
            self.mesh = mesh
        else:
            self.mesh = "package://blockhandler/meshes/BIG_LEGO.dae"
        self.id = id
        self.position = msg.Vector3()
        self.position.x = pos[0]
        self.position.y = pos[1]
        self.position.z = pos[2]
        self.rotation = msg.Quaternion()
        self.rotation.x = rot[0]
        self.rotation.y = rot[1]
        self.rotation.z = rot[2]
        self.rotation.w = rot[3]

        self.scale = 1.
        self.shape = shape
        self.status = Block_Status.not_moved
        self.color_r = random()
        self.color_g = random()
        self.color_b = random()
        self.tf_pos = None

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
        if self.tf_pos:
            # try:
            current = tfBuffer.lookup_transform('world', self.tf_pos, rospy.Time(0), rospy.Duration(1.0))
            self.position = current.transform.translation
            self.rotation = current.transform.rotation

            # rospy.logerr(f"Block–{self.id}: rotation={self.rotation}")
            """except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
                rospy.logerr(error)
                print(repr(error))"""
        marker.pose.position = self.position
        marker.pose.orientation = self.rotation

        # Scale down
        marker.scale.x = self.scale * self.shape[0]
        marker.scale.y = self.scale * self.shape[1]
        marker.scale.z = self.scale * self.shape[2]
        marker.color.a = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        # marker.mesh_resource = self.mesh
        return marker


if __name__ == '__main__':
    rospy.init_node('rosnode', xmlrpc_port=int(os.environ["xmlrpc_port"]), tcpros_port=int(os.environ["tcpros_port"]))
    rate = rospy.Rate(30)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))
    bh = BlockHandler(server)
    bh.start()

    block_handler = RosBlockHandler()
    block_handler.publish_all()

    bh.functionality["next_block"] = block_handler.get_next_block
    bh.functionality["attach_block"] = block_handler.attach_block
    bh.functionality["all_blocks"] = block_handler.all_blocks
    bh.functionality["spawn"] = block_handler.spawn

    while not rospy.is_shutdown():
        block_handler.publish_all()
        rate.sleep()
