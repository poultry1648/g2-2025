"""
Publish the tuned URDF as a latched description plus a live TF tree.

``robot_description`` is published once (durably) so consumers such as the
RViz RobotModel load geometry a single time. Joint origins are then expressed
purely through TF, which is recomputed every tick so dragging a marker moves
the already-loaded meshes without reloading them.
"""

from g2_urdf_tuner.urdf_model import edge_transform, matrix_to_quaternion
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class TfPublisher:
    """Broadcasts a :class:`UrdfModel` on ``/tf`` and ``/joint_states``."""

    def __init__(self, node, model):
        """Wire up publishers on ``node`` for the given ``model``."""
        self._node = node
        self.model = model
        self._broadcaster = TransformBroadcaster(node)
        self._joint_state_pub = node.create_publisher(JointState, 'joint_states', 10)
        description_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._description_pub = node.create_publisher(
            String, 'robot_description', description_qos)

    def publish_description(self):
        """Publish the current URDF text durably."""
        message = String()
        message.data = self.model.source_text
        self._description_pub.publish(message)

    def publish(self):
        """Broadcast the whole kinematic tree and the current joint states."""
        stamp = self._node.get_clock().now().to_msg()
        transforms = []
        for joint in self.model.joints:
            translation, rotation = edge_transform(joint)
            quaternion = matrix_to_quaternion(rotation)
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = joint['parent']
            transform.child_frame_id = joint['child']
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            transform.transform.rotation.x = float(quaternion[0])
            transform.transform.rotation.y = float(quaternion[1])
            transform.transform.rotation.z = float(quaternion[2])
            transform.transform.rotation.w = float(quaternion[3])
            transforms.append(transform)
        if transforms:
            self._broadcaster.sendTransform(transforms)

        joint_state = JointState()
        joint_state.header.stamp = stamp
        for joint in self.model.movable_joints():
            joint_state.name.append(joint['name'])
            joint_state.position.append(float(joint['value']))
        self._joint_state_pub.publish(joint_state)
