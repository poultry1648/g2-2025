"""
Parse, edit and serialize URDF robots for interactive tuning.

The model keeps a parsed view of the URDF used for kinematics and marker
placement, and writes edits back into the original file in place so the diff
stays limited to the joint ``<origin>``/``<axis>`` lines that changed.
"""

import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np


def _tag(element):
    """Return an element tag without any XML namespace."""
    return element.tag.split('}')[-1]


def _parse_vec(text, default):
    """Parse a whitespace separated 3-vector, returning default on failure."""
    if not text:
        return list(default)
    values = [float(v) for v in text.replace(',', ' ').split()]
    if len(values) != 3:
        return list(default)
    return values


def format_vec(values):
    """Format a 3-vector the way URDF files conventionally spell it."""
    return ' '.join('0.0' if abs(float(v)) < 1e-12 else '%.9g' % float(v)
                    for v in values)


def rpy_to_matrix(rpy):
    """Convert URDF fixed-axis roll/pitch/yaw to a rotation matrix."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rot_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rot_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rot_z @ rot_y @ rot_x


def matrix_to_rpy(matrix):
    """Invert :func:`rpy_to_matrix`."""
    sin_pitch = max(-1.0, min(1.0, -matrix[2, 0]))
    pitch = math.asin(sin_pitch)
    if abs(matrix[2, 0]) < 1.0 - 1e-9:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        yaw = 0.0
    return [roll, pitch, yaw]


def axis_angle_to_matrix(axis, angle):
    """Rodrigues rotation matrix for a rotation about ``axis``."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12 or abs(angle) < 1e-12:
        return np.eye(3)
    x, y, z = axis / norm
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    one_c = 1.0 - cos_a
    return np.array([
        [cos_a + x * x * one_c, x * y * one_c - z * sin_a, x * z * one_c + y * sin_a],
        [y * x * one_c + z * sin_a, cos_a + y * y * one_c, y * z * one_c - x * sin_a],
        [z * x * one_c - y * sin_a, z * y * one_c + x * sin_a, cos_a + z * z * one_c],
    ])


def quaternion_to_matrix(quaternion):
    """Convert ``[x, y, z, w]`` to a rotation matrix."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


def matrix_to_quaternion(matrix):
    """Convert a rotation matrix to ``[x, y, z, w]``."""
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    return [x, y, z, w]


def origin_matrix(joint):
    """Homogeneous transform of a joint's ``<origin>``."""
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix(joint['rpy'])
    matrix[:3, 3] = joint['xyz']
    return matrix


def motion_matrix(joint):
    """Homogeneous transform contributed by the joint's current value."""
    matrix = np.eye(4)
    joint_type = joint['type']
    if joint_type in ('revolute', 'continuous'):
        matrix[:3, :3] = axis_angle_to_matrix(joint['axis'], joint['value'])
    elif joint_type == 'prismatic':
        axis = np.asarray(joint['axis'], dtype=float)
        norm = np.linalg.norm(axis)
        if norm > 1e-12:
            matrix[:3, 3] = axis / norm * joint['value']
    return matrix


def edge_transform(joint):
    """Return ``(translation, rotation)`` from parent frame to child frame."""
    rot_origin = rpy_to_matrix(joint['rpy'])
    translation = np.asarray(joint['xyz'], dtype=float).copy()
    joint_type = joint['type']
    value = joint['value']
    if joint_type in ('revolute', 'continuous'):
        rotation = rot_origin @ axis_angle_to_matrix(joint['axis'], value)
    else:
        rotation = rot_origin
        if joint_type == 'prismatic':
            axis = np.asarray(joint['axis'], dtype=float)
            norm = np.linalg.norm(axis)
            if norm > 1e-12:
                translation = translation + rot_origin @ (axis / norm * value)
    return translation, rotation


class UrdfModel:
    """A URDF document with live kinematic evaluation and in-place saving."""

    def __init__(self, path):
        """Load and index the URDF at ``path``."""
        self.path = str(path)
        self.source_text = Path(self.path).read_text()
        self.root = ET.fromstring(self.source_text)
        self.links = {}
        self.link_visuals = {}
        self.joints = []
        self.joint_by_name = {}
        self.child_joint = {}
        self.visual_dirty = set()
        self._load()

    def _load(self):
        for link in self.root:
            if _tag(link) == 'link':
                self.links[link.get('name')] = link
                self.link_visuals[link.get('name')] = self._parse_visuals(link)
        for element in self.root:
            if _tag(element) != 'joint':
                continue
            origin = None
            axis = None
            parent = None
            child = None
            for sub in element:
                tag = _tag(sub)
                if tag == 'origin':
                    origin = sub
                elif tag == 'axis':
                    axis = sub
                elif tag == 'parent':
                    parent = sub.get('link')
                elif tag == 'child':
                    child = sub.get('link')
            origin_xyz = origin.get('xyz') if origin is not None else None
            origin_rpy = origin.get('rpy') if origin is not None else None
            axis_xyz = axis.get('xyz') if axis is not None else None
            joint = {
                'name': element.get('name'),
                'type': element.get('type'),
                'parent': parent,
                'child': child,
                'xyz': _parse_vec(origin_xyz, [0.0, 0.0, 0.0]),
                'rpy': _parse_vec(origin_rpy, [0.0, 0.0, 0.0]),
                'axis': _parse_vec(axis_xyz, [0.0, 0.0, 1.0]),
                'value': 0.0,
                'element': element,
                'origin_element': origin,
                'axis_element': axis,
                'dirty_origin': False,
                'dirty_axis': False,
            }
            self.joints.append(joint)
            self.joint_by_name[joint['name']] = joint
            self.child_joint[joint['child']] = joint
        self.root_links = [name for name in self.links if name not in self.child_joint]

    @staticmethod
    def _parse_visuals(link):
        """Collect the mesh visuals of a link as origin/filename/scale dicts."""
        visuals = []
        for visual in link:
            if _tag(visual) != 'visual':
                continue
            origin = None
            geometry = None
            for sub in visual:
                tag = _tag(sub)
                if tag == 'origin':
                    origin = sub
                elif tag == 'geometry':
                    geometry = sub
            mesh = None
            if geometry is not None:
                for sub in geometry:
                    if _tag(sub) == 'mesh':
                        mesh = sub
            if mesh is None or not mesh.get('filename'):
                continue
            visuals.append({
                'xyz': _parse_vec(origin.get('xyz') if origin is not None else None,
                                  [0.0, 0.0, 0.0]),
                'rpy': _parse_vec(origin.get('rpy') if origin is not None else None,
                                  [0.0, 0.0, 0.0]),
                'filename': mesh.get('filename'),
                'scale': _parse_vec(mesh.get('scale'), [1.0, 1.0, 1.0]),
                'element': visual,
                'origin_element': origin,
            })
        return visuals

    @property
    def root_link(self):
        """Return the single root link of the kinematic tree."""
        return self.root_links[0]

    def movable_joints(self):
        """All joints that have a degree of freedom."""
        return [jd for jd in self.joints if jd['type'] != 'fixed']

    def set_origin(self, name, xyz=None, rpy=None):
        """Set a joint's ``<origin>`` and mark it for saving."""
        joint = self.joint_by_name[name]
        element = joint['origin_element']
        if element is None:
            element = ET.Element('origin')
            joint['element'].insert(0, element)
            joint['origin_element'] = element
        if xyz is not None:
            joint['xyz'] = [float(v) for v in xyz]
            element.set('xyz', format_vec(joint['xyz']))
        if rpy is not None:
            joint['rpy'] = [float(v) for v in rpy]
            element.set('rpy', format_vec(joint['rpy']))
        joint['dirty_origin'] = True

    def set_axis(self, name, axis):
        """Set a joint's ``<axis>`` and mark it for saving."""
        joint = self.joint_by_name[name]
        element = joint['axis_element']
        if element is None:
            element = ET.Element('axis')
            joint['element'].insert(0, element)
            joint['axis_element'] = element
        joint['axis'] = [float(v) for v in axis]
        element.set('xyz', format_vec(joint['axis']))
        joint['dirty_axis'] = True

    def set_visual_origin(self, link_name, index, xyz=None, rpy=None):
        """Set a link visual's mesh ``<origin>`` and mark it for saving."""
        visual = self.link_visuals[link_name][index]
        element = visual['origin_element']
        if element is None:
            element = ET.Element('origin')
            visual['element'].insert(0, element)
            visual['origin_element'] = element
        if xyz is not None:
            visual['xyz'] = [float(v) for v in xyz]
            element.set('xyz', format_vec(visual['xyz']))
        if rpy is not None:
            visual['rpy'] = [float(v) for v in rpy]
            element.set('rpy', format_vec(visual['rpy']))
        self.visual_dirty.add((link_name, index))

    def visual_origin(self, link_name, index=0):
        """Return ``(xyz, rpy)`` of a link visual's mesh origin."""
        visual = self.link_visuals[link_name][index]
        return list(visual['xyz']), list(visual['rpy'])

    def link_world_transforms(self):
        """World transform of every link, keyed by link name."""
        children = {}
        for joint in self.joints:
            children.setdefault(joint['parent'], []).append(joint)
        transforms = {}

        def visit(link, matrix):
            transforms[link] = matrix
            for joint in children.get(link, []):
                visit(joint['child'], matrix @ origin_matrix(joint) @ motion_matrix(joint))

        for root in self.root_links:
            visit(root, np.eye(4))
        return transforms

    def joint_world_transform(self, joint, transforms=None):
        """World transform of a joint's origin frame."""
        transforms = transforms or self.link_world_transforms()
        return transforms[joint['parent']] @ origin_matrix(joint)

    def is_dirty(self):
        """Whether any joint or visual has unsaved edits."""
        return bool(self.visual_dirty) or any(
            jd['dirty_origin'] or jd['dirty_axis'] for jd in self.joints)

    def save(self, path=None):
        """Write edits back to disk, touching only changed origin/axis lines."""
        path = Path(path or self.path)
        lines = path.read_text().split('\n')
        ranges = self._joint_line_ranges(lines)
        for joint in sorted(self.joints, key=lambda jd: ranges.get(jd['name'], (0, 0))[0],
                            reverse=True):
            if not (joint['dirty_origin'] or joint['dirty_axis']):
                continue
            bounds = ranges.get(joint['name'])
            if bounds is None:
                continue
            start, end = bounds
            origin_index = None
            axis_index = None
            for index in range(start, end + 1):
                stripped = lines[index].lstrip()
                if stripped.startswith('<origin'):
                    origin_index = index
                elif stripped.startswith('<axis'):
                    axis_index = index
            indent = re.match(r'\s*', lines[start]).group(0) + '    '
            if joint['dirty_origin']:
                new_line = '{}<origin xyz="{}" rpy="{}"/>'.format(
                    indent, format_vec(joint['xyz']), format_vec(joint['rpy']))
                if origin_index is not None:
                    lines[origin_index] = new_line
                else:
                    lines.insert(start + 1, new_line)
            if joint['dirty_axis'] and axis_index is not None:
                lines[axis_index] = '{}<axis xyz="{}"/>'.format(
                    indent, format_vec(joint['axis']))
        self._save_visual_origins(lines)
        path.write_text('\n'.join(lines))
        for joint in self.joints:
            joint['dirty_origin'] = False
            joint['dirty_axis'] = False
        self.visual_dirty = set()
        self.source_text = path.read_text()

    def _save_visual_origins(self, lines):
        """Rewrite the ``<origin>`` line of each dirty link visual."""
        link_ranges = self._link_line_ranges(lines)
        edits = []
        for link_name, index in self.visual_dirty:
            bounds = link_ranges.get(link_name)
            if bounds is not None:
                edits.append((bounds[0], index, link_name, bounds))
        for _, index, link_name, bounds in sorted(edits, reverse=True):
            visual = self.link_visuals[link_name][index]
            start, end = bounds
            visual_seen = -1
            origin_index = None
            insert_index = None
            indent = None
            for line_index in range(start, end + 1):
                stripped = lines[line_index].lstrip()
                if stripped.startswith('<visual'):
                    visual_seen += 1
                    if visual_seen == index:
                        indent = re.match(r'\s*', lines[line_index]).group(0) + '    '
                        insert_index = line_index + 1
                elif visual_seen == index and stripped.startswith('<origin'):
                    origin_index = line_index
                elif visual_seen == index and stripped.startswith('</visual'):
                    break
            if indent is None:
                continue
            new_line = '{}<origin xyz="{}" rpy="{}"/>'.format(
                indent, format_vec(visual['xyz']), format_vec(visual['rpy']))
            if origin_index is not None:
                lines[origin_index] = new_line
            elif insert_index is not None:
                lines.insert(insert_index, new_line)

    @staticmethod
    def _link_line_ranges(lines):
        ranges = {}
        current = None
        for index, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith('<link '):
                match = re.search(r'name="([^"]+)"', stripped)
                current = match.group(1) if match else None
                if current:
                    ranges[current] = (index, index)
            elif stripped.startswith('</link>'):
                if current in ranges:
                    start, _ = ranges[current]
                    ranges[current] = (start, index)
                current = None
        return ranges

    @staticmethod
    def _joint_line_ranges(lines):
        ranges = {}
        current = None
        for index, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith('<joint'):
                match = re.search(r'name="([^"]+)"', stripped)
                current = match.group(1) if match else None
                if current:
                    ranges[current] = (index, index)
            elif stripped.startswith('</joint>'):
                if current in ranges:
                    start, _ = ranges[current]
                    ranges[current] = (start, index)
                current = None
        return ranges
