#!/usr/bin/env python
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
        for i in range(10):
            self.blocks.append(Block(i, [5., 0., i*.1], [0, 0, 0, 1]))

    def get_block(self, block_id: int):
        for b in self.blocks:
            if block_id == b.id:
                return b
        return self.blocks[int(block_id)]

    def publish_all(self):
        for block in self.blocks:
            self.pub.publish(block.as_msg())
            self.br.sendTransform(block.position,
                                  block.rotation, rospy.Time.now(), "Block_{block.id}",
                                  "world")

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

    def __init__(self, id: int = 0, pos: list = None, rot: list = None):
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

        self.scale = .01
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
            try:
                current = tfBuffer.lookup_transform('world', self.tf_pos, rospy.Time(0), rospy.Duration(1.0))
                self.position = current.transform.translation
                self.rotation = current.transform.rotation
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
                rospy.logerr(error)
        marker.pose.position = self.position
        marker.pose.orientation = self.rotation

        """
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = self.rotation[0]
        marker.pose.orientation.y = self.rotation[1]
        marker.pose.orientation.z = self.rotation[2]
        marker.pose.orientation.w = self.rotation[3]
        """

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

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))
    bh = BlockHandler(server)
    bh.start()

    block_handler = RosBlockHandler()
    block_handler.publish_all()

    bh.funtionality["next_block"] = block_handler.get_next_block
    bh.funtionality["attach_block"] = block_handler.attach_block

    while not rospy.is_shutdown():
        block_handler.publish_all()
        rate.sleep()