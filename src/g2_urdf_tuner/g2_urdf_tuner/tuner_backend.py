"""
Live URDF tuning backend.

Owns the URDF model and the ROS publishers needed to preview edits: a latched
robot description, the live TF tree, joint states and a MarkerArray of joint
axes. A front end (the tkinter slider panel) mutates the model and calls
:meth:`tick` to push the result to RViz.
"""

from g2_urdf_tuner.tf_publisher import TfPublisher
from g2_urdf_tuner.urdf_model import matrix_to_quaternion, origin_matrix, UrdfModel
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class TunerBackend:
    """Model plus publishers shared by any tuning front end."""

    def __init__(self, node, urdf_path, save_path, axis_length=0.15):
        """Load ``urdf_path`` and start publishing it on ``node``."""
        self.node = node
        self.urdf_path = urdf_path
        self.save_path = save_path
        self.axis_length = axis_length
        self.axes_visible = True
        self.model = UrdfModel(urdf_path)
        self.tf = TfPublisher(node, self.model)
        self._axes_pub = node.create_publisher(MarkerArray, 'urdf_tuner/axes', 1)
        self._mesh_pub = node.create_publisher(MarkerArray, 'urdf_tuner/meshes', 1)
        self.mesh_visibility = {}
        self._mesh_dirty = True
        self._mesh_heartbeat = 0
        self._reset_mesh_visibility()
        self.publish_description()

    def publish_description(self):
        """Re-send the latched robot description."""
        self.tf.publish_description()

    def save(self):
        """Write the current model back to the save path."""
        self.model.save(self.save_path)

    def reload(self):
        """Discard edits and reload the URDF from disk."""
        self.model = UrdfModel(self.urdf_path)
        self.tf.model = self.model
        self._reset_mesh_visibility()
        self.publish_description()

    def set_axes_visible(self, visible):
        """Show or hide the joint axis arrows."""
        self.axes_visible = visible
        if not visible:
            array = MarkerArray()
            array.markers.append(self._delete_all_marker())
            self._axes_pub.publish(array)

    def tick(self):
        """Broadcast TF, joint states and the pending marker updates."""
        self.tf.publish()
        if self.axes_visible:
            self._publish_axes()
        self._mesh_heartbeat = (self._mesh_heartbeat + 1) % 30
        if self._mesh_dirty or self._mesh_heartbeat == 0:
            self.publish_meshes()

    def _reset_mesh_visibility(self):
        """Reset every link's mesh to visible and force a republish."""
        self.mesh_visibility = {name: True for name in self.model.link_visuals}
        self._mesh_dirty = True

    def set_link_visible(self, link_name, visible):
        """Show or hide one link's mesh and republish immediately."""
        self.mesh_visibility[link_name] = bool(visible)
        self.publish_meshes()

    def set_all_links_visible(self, visible):
        """Show or hide every link's mesh and republish immediately."""
        for link_name in self.mesh_visibility:
            self.mesh_visibility[link_name] = bool(visible)
        self.publish_meshes()

    def mark_meshes_dirty(self):
        """Flag that link poses changed and the meshes must be republished."""
        self._mesh_dirty = True

    def warn_on_conflicts(self):
        """Warn if another node also publishes TF or the description."""
        for topic in ('/tf', '/robot_description'):
            try:
                endpoints = self.node.get_publishers_info_by_topic(topic)
            except Exception:  # discovery may be unavailable very early
                continue
            others = sorted({endpoint.node_name for endpoint in endpoints
                             if endpoint.node_name != self.node.get_name()})
            if others:
                self.node.get_logger().warning(
                    'Another node is already publishing %s (%s); it will conflict with '
                    'the tuner. Stop it (e.g. display.launch.py) for reliable tuning.'
                    % (topic, ', '.join(others)))

    @staticmethod
    def _delete_all_marker():
        """Create a Marker that clears the axis MarkerArray."""
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def _basis_from_x(axis):
        """Rotation matrix whose x column points along ``axis``."""
        x_axis = axis / np.linalg.norm(axis)
        reference = np.array([0.0, 0.0, 1.0]) if abs(x_axis[2]) < 0.9 \
            else np.array([1.0, 0.0, 0.0])
        y_axis = np.cross(reference, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    @staticmethod
    def _axis_color(joint):
        """RGBA for the axis arrow, by joint type and axis validity."""
        axis = np.asarray(joint['axis'], dtype=float)
        if np.linalg.norm(axis) < 1e-9:
            return 1.0, 0.85, 0.1, 1.0
        if joint['type'] == 'prismatic':
            return 0.2, 0.45, 1.0, 1.0
        if joint['type'] in ('revolute', 'continuous'):
            return 0.15, 0.9, 0.3, 1.0
        return 0.7, 0.7, 0.7, 1.0

    def _publish_axes(self):
        """Publish one arrow per joint showing its current axis."""
        array = MarkerArray()
        stamp = self.node.get_clock().now().to_msg()
        transforms = self.model.link_world_transforms()
        for index, joint in enumerate(self.model.joints):
            joint_transform = self.model.joint_world_transform(joint, transforms)
            axis = np.asarray(joint['axis'], dtype=float)
            if np.linalg.norm(axis) < 1e-9:
                axis = np.array([0.0, 0.0, 1.0])
            axis_world = joint_transform[:3, :3] @ (axis / np.linalg.norm(axis))
            quaternion = matrix_to_quaternion(self._basis_from_x(axis_world))

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.model.root_link
            marker.ns = 'joint_axes'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = float(joint_transform[0, 3])
            marker.pose.position.y = float(joint_transform[1, 3])
            marker.pose.position.z = float(joint_transform[2, 3])
            marker.pose.orientation.x = float(quaternion[0])
            marker.pose.orientation.y = float(quaternion[1])
            marker.pose.orientation.z = float(quaternion[2])
            marker.pose.orientation.w = float(quaternion[3])
            marker.scale.x = self.axis_length
            marker.scale.y = max(self.axis_length * 0.04, 0.004)
            marker.scale.z = max(self.axis_length * 0.06, 0.006)
            red, green, blue, alpha = self._axis_color(joint)
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = alpha
            array.markers.append(marker)
        self._axes_pub.publish(array)

    def publish_meshes(self):
        """
        Publish one mesh marker per visual, deleting the hidden links.

        Driving the meshes as markers (rather than editing the URDF) means a
        link can be hidden or shown instantly without RViz reloading anything.
        """
        array = MarkerArray()
        stamp = self.node.get_clock().now().to_msg()
        transforms = self.model.link_world_transforms()
        marker_id = 0
        for link_name, visuals in self.model.link_visuals.items():
            link_matrix = transforms.get(link_name)
            visible = self.mesh_visibility.get(link_name, True)
            for visual in visuals:
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = self.model.root_link
                marker.ns = 'link_meshes'
                marker.id = marker_id
                marker.type = Marker.MESH_RESOURCE
                marker.action = Marker.ADD if visible else Marker.DELETE
                if visible and link_matrix is not None:
                    matrix = link_matrix @ origin_matrix(visual)
                    quaternion = matrix_to_quaternion(matrix[:3, :3])
                    marker.pose.position.x = float(matrix[0, 3])
                    marker.pose.position.y = float(matrix[1, 3])
                    marker.pose.position.z = float(matrix[2, 3])
                    marker.pose.orientation.x = float(quaternion[0])
                    marker.pose.orientation.y = float(quaternion[1])
                    marker.pose.orientation.z = float(quaternion[2])
                    marker.pose.orientation.w = float(quaternion[3])
                    marker.scale.x = float(visual['scale'][0])
                    marker.scale.y = float(visual['scale'][1])
                    marker.scale.z = float(visual['scale'][2])
                    marker.mesh_resource = visual['filename']
                    marker.mesh_use_embedded_materials = True
                array.markers.append(marker)
                marker_id += 1
        self._mesh_pub.publish(array)
        self._mesh_dirty = False
