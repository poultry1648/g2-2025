"""
Tkinter slider panel for tuning URDF joint origins and axes.

Pick a joint, drag the origin (x/y/z and roll/pitch/yaw) and axis sliders, and
watch the robot update live in RViz. Drag the joint value slider to spin the
link and confirm the axis. Save writes the model back to the URDF.
"""

import math
import tkinter as tk
from tkinter import ttk

from g2_urdf_tuner.tuner_backend import TunerBackend
import numpy as np
import rclpy
from rclpy.node import Node

DEG = math.pi / 180.0

ORIGIN_SPECS = [
    ('x', 'x (m)', -1.0, 1.0, 0.001, 4),
    ('y', 'y (m)', -1.0, 1.0, 0.001, 4),
    ('z', 'z (m)', -1.0, 1.0, 0.001, 4),
    ('roll', 'roll (deg)', -180.0, 180.0, 0.5, 1),
    ('pitch', 'pitch (deg)', -180.0, 180.0, 0.5, 1),
    ('yaw', 'yaw (deg)', -180.0, 180.0, 0.5, 1),
]

AXIS_SPECS = [
    ('ax', 'axis x', -1.0, 1.0, 0.01, 3),
    ('ay', 'axis y', -1.0, 1.0, 0.01, 3),
    ('az', 'axis z', -1.0, 1.0, 0.01, 3),
]

AXIS_PRESETS = [
    ('+X', [1.0, 0.0, 0.0]),
    ('-X', [-1.0, 0.0, 0.0]),
    ('+Y', [0.0, 1.0, 0.0]),
    ('-Y', [0.0, -1.0, 0.0]),
    ('+Z', [0.0, 0.0, 1.0]),
    ('-Z', [0.0, 0.0, -1.0]),
]

ORIGIN_INDEX = {'x': 0, 'y': 1, 'z': 2}
ROTATION_INDEX = {'roll': 0, 'pitch': 1, 'yaw': 2}
AXIS_INDEX = {'ax': 0, 'ay': 1, 'az': 2}


def _fmt(value, places):
    """Format a number without trailing zeros."""
    text = ('%.*f' % (places, value)).rstrip('0').rstrip('.')
    if text in ('', '-0'):
        text = '0'
    return text


class ValueRow:
    """A labelled horizontal slider paired with an editable numeric entry."""

    def __init__(self, parent, row, label, minimum, maximum, resolution,
                 on_change, places=4):
        """Lay out the row in the parent grid and wire up its callbacks."""
        self.on_change = on_change
        self.places = places
        self.minimum = minimum
        self.maximum = maximum
        self.resolution = resolution
        self._updating = False

        tk.Label(parent, text=label, width=11, anchor='e').grid(
            row=row, column=0, sticky='e', padx=(6, 2), pady=2)
        self.var = tk.DoubleVar(master=parent, value=0.0)
        self.scale = tk.Scale(
            parent, from_=minimum, to=maximum, resolution=resolution,
            orient='horizontal', variable=self.var, showvalue=False,
            length=300, command=self._on_scale)
        self.scale.grid(row=row, column=1, sticky='ew', padx=2, pady=2)
        self.entry_var = tk.StringVar(master=parent, value=_fmt(0.0, places))
        self.entry = tk.Entry(parent, textvariable=self.entry_var, width=11)
        self.entry.grid(row=row, column=2, sticky='w', padx=(2, 6), pady=2)
        self.entry.bind('<Return>', self._on_entry)
        self.entry.bind('<FocusOut>', self._on_entry)
        parent.columnconfigure(1, weight=1)

    def _on_scale(self, _value=None):
        if self._updating:
            return
        value = self.var.get()
        self.entry_var.set(_fmt(value, self.places))
        self.on_change(value)

    def _on_entry(self, _event=None):
        if self._updating:
            return
        try:
            value = float(self.entry_var.get())
        except ValueError:
            self.entry_var.set(_fmt(self.var.get(), self.places))
            return
        self.set_value(value)
        self.on_change(self.var.get())

    def set_value(self, value):
        """Set the row value without firing the change callback."""
        value = max(self.minimum, min(self.maximum, float(value)))
        self._updating = True
        self.var.set(value)
        self.entry_var.set(_fmt(value, self.places))
        self._updating = False

    def set_range(self, minimum, maximum, resolution):
        """Change the slider range, e.g. when switching joint type."""
        self.minimum = minimum
        self.maximum = maximum
        self.resolution = resolution
        self.scale.configure(from_=minimum, to=maximum, resolution=resolution)
        self.set_value(self.var.get())

    def set_enabled(self, enabled):
        """Enable or grey out the slider and entry."""
        state = 'normal' if enabled else 'disabled'
        self.scale.configure(state=state)
        self.entry.configure(state=state)


class TunerApp:
    """The tkinter window tying sliders to a :class:`TunerBackend`."""

    def __init__(self, backend):
        """Build the window and start the periodic publish loop."""
        self.backend = backend
        self.joint = None
        self.tick_count = 0

        self.root = tk.Tk()
        self.root.title('URDF Tuner')
        self.root.geometry('780x760')
        self.root.protocol('WM_DELETE_WINDOW', self.root.destroy)
        self.root.columnconfigure(0, weight=1)

        self.axes_visible = tk.BooleanVar(master=self.root, value=True)
        self.status = tk.StringVar(master=self.root, value='')
        self.mesh_vars = {}

        self._build_toolbar()
        self._build_origin_frame()
        self._build_axis_frame()
        self._build_value_frame()
        self._build_mesh_frame()
        self._build_status()
        self._refresh_joint_list()
        self._refresh_mesh_list()

        self.root.after(33, self._tick)

    # -- layout -----------------------------------------------------------

    def _build_toolbar(self):
        toolbar = tk.Frame(self.root)
        toolbar.grid(row=0, column=0, sticky='ew', padx=6, pady=(6, 2))
        tk.Label(toolbar, text='Joint:').pack(side='left')
        self.joint_var = tk.StringVar(master=toolbar)
        self.joint_combo = ttk.Combobox(
            toolbar, textvariable=self.joint_var, state='readonly', width=26)
        self.joint_combo.pack(side='left', padx=4)
        self.joint_combo.bind('<<ComboboxSelected>>', self._on_joint_selected)
        tk.Button(toolbar, text='Save URDF', command=self._save).pack(side='left', padx=3)
        tk.Button(toolbar, text='Reload', command=self._reload).pack(side='left', padx=3)
        tk.Button(toolbar, text='Zero origin', command=self._zero_origin).pack(side='left', padx=3)
        tk.Checkbutton(
            toolbar, text='Show axes', variable=self.axes_visible,
            command=self._toggle_axes).pack(side='left', padx=6)

    def _build_origin_frame(self):
        frame = tk.LabelFrame(self.root, text='Origin')
        frame.grid(row=1, column=0, sticky='ew', padx=6, pady=4)
        self.origin_rows = {}
        for index, (key, label, low, high, resolution, places) in enumerate(ORIGIN_SPECS):
            self.origin_rows[key] = ValueRow(
                frame, index, label, low, high, resolution,
                lambda value, k=key: self._on_origin(k, value), places)

    def _build_axis_frame(self):
        frame = tk.LabelFrame(self.root, text='Axis (unit vector in the child frame)')
        frame.grid(row=2, column=0, sticky='ew', padx=6, pady=4)
        self.axis_rows = {}
        offset = 0
        for index, (key, label, low, high, resolution, places) in enumerate(AXIS_SPECS):
            self.axis_rows[key] = ValueRow(
                frame, index, label, low, high, resolution,
                lambda value, k=key: self._on_axis(k, value), places)
            offset = index + 1
        presets = tk.Frame(frame)
        presets.grid(row=offset, column=0, columnspan=3, sticky='w', padx=6, pady=(2, 4))
        tk.Label(presets, text='Presets:').pack(side='left')
        for label, vector in AXIS_PRESETS:
            tk.Button(presets, text=label, width=3,
                      command=lambda v=vector: self._set_axis_preset(v)).pack(side='left', padx=1)
        tk.Button(presets, text='Normalize', command=self._normalize_axis).pack(
            side='left', padx=(8, 1))

    def _build_value_frame(self):
        frame = tk.LabelFrame(self.root, text='Joint value (spin to check the axis)')
        frame.grid(row=3, column=0, sticky='ew', padx=6, pady=4)
        self.value_row = ValueRow(
            frame, 0, 'value', -180.0, 180.0, 1.0, self._on_value, 2)

    def _build_status(self):
        tk.Label(self.root, textvariable=self.status, anchor='w', fg='#444').grid(
            row=5, column=0, sticky='ew', padx=8, pady=(2, 8))

    def _build_mesh_frame(self):
        frame = tk.LabelFrame(self.root, text='Meshes (tick a link to show it)')
        frame.grid(row=4, column=0, sticky='nsew', padx=6, pady=4)
        self.root.rowconfigure(4, weight=1)
        buttons = tk.Frame(frame)
        buttons.pack(fill='x', pady=(2, 2))
        tk.Button(buttons, text='Show all',
                  command=lambda: self._set_all_meshes(True)).pack(side='left', padx=2)
        tk.Button(buttons, text='Hide all',
                  command=lambda: self._set_all_meshes(False)).pack(side='left', padx=2)
        self.mesh_check_frame = tk.Frame(frame)
        self.mesh_check_frame.pack(fill='both', expand=True)

    # -- joint selection --------------------------------------------------

    def _refresh_joint_list(self):
        names = [joint['name'] for joint in self.backend.model.joints]
        self.joint_combo.configure(values=names)
        if names:
            self.joint_combo.set(names[0])
            self._select_joint(names[0])

    def _refresh_mesh_list(self):
        for child in self.mesh_check_frame.winfo_children():
            child.destroy()
        self.mesh_vars = {}
        for index, link_name in enumerate(self.backend.model.link_visuals):
            var = tk.BooleanVar(
                master=self.mesh_check_frame,
                value=self.backend.mesh_visibility.get(link_name, True))
            check = tk.Checkbutton(
                self.mesh_check_frame, text=link_name, variable=var, anchor='w',
                command=lambda name=link_name: self._on_mesh_toggle(name))
            check.grid(row=index // 3, column=index % 3, sticky='w', padx=4, pady=1)
            self.mesh_vars[link_name] = var

    def _on_mesh_toggle(self, link_name):
        self.backend.set_link_visible(link_name, self.mesh_vars[link_name].get())

    def _set_all_meshes(self, visible):
        for var in self.mesh_vars.values():
            var.set(visible)
        self.backend.set_all_links_visible(visible)

    def _on_joint_selected(self, _event=None):
        self._select_joint(self.joint_var.get())

    def _select_joint(self, name):
        joint = self.backend.model.joint_by_name.get(name)
        if joint is None:
            return
        self.joint = joint
        self.joint_var.set(name)
        self._configure_value_row(joint)
        self._load_origin(joint)
        self._load_axis(joint)
        self._load_value(joint)
        self.status.set('Editing %s [%s]' % (joint['name'], joint['type']))

    def _configure_value_row(self, joint):
        if joint['type'] in ('revolute', 'continuous'):
            self.value_row.set_range(-180.0, 180.0, 1.0)
            self.value_row.set_enabled(True)
        elif joint['type'] == 'prismatic':
            self.value_row.set_range(-0.5, 0.5, 0.001)
            self.value_row.set_enabled(True)
        else:
            self.value_row.set_enabled(False)

    def _load_origin(self, joint):
        for key in ORIGIN_INDEX:
            self.origin_rows[key].set_value(joint['xyz'][ORIGIN_INDEX[key]])
        for key in ROTATION_INDEX:
            self.origin_rows[key].set_value(joint['rpy'][ROTATION_INDEX[key]] / DEG)

    def _load_axis(self, joint):
        for key in AXIS_INDEX:
            self.axis_rows[key].set_value(joint['axis'][AXIS_INDEX[key]])

    def _load_value(self, joint):
        if joint['type'] in ('revolute', 'continuous'):
            self.value_row.set_value(joint['value'] / DEG)
        else:
            self.value_row.set_value(joint['value'])

    # -- slider callbacks -------------------------------------------------

    def _on_origin(self, key, value):
        if self.joint is None:
            return
        if key in ORIGIN_INDEX:
            xyz = list(self.joint['xyz'])
            xyz[ORIGIN_INDEX[key]] = value
            self.backend.model.set_origin(self.joint['name'], xyz=xyz)
        else:
            rpy = list(self.joint['rpy'])
            rpy[ROTATION_INDEX[key]] = value * DEG
            self.backend.model.set_origin(self.joint['name'], rpy=rpy)
        self.backend.mark_meshes_dirty()

    def _on_axis(self, key, value):
        if self.joint is None:
            return
        axis = list(self.joint['axis'])
        axis[AXIS_INDEX[key]] = value
        self.backend.model.set_axis(self.joint['name'], axis)

    def _on_value(self, value):
        if self.joint is None:
            return
        if self.joint['type'] in ('revolute', 'continuous'):
            self.joint['value'] = value * DEG
        else:
            self.joint['value'] = value
        self.backend.mark_meshes_dirty()

    def _set_axis_preset(self, vector):
        if self.joint is None:
            return
        self.backend.model.set_axis(self.joint['name'], list(vector))
        self._load_axis(self.joint)

    def _normalize_axis(self):
        if self.joint is None:
            return
        axis = np.asarray(self.joint['axis'], dtype=float)
        norm = np.linalg.norm(axis)
        if norm > 1e-9:
            self.backend.model.set_axis(self.joint['name'], (axis / norm).tolist())
            self._load_axis(self.joint)

    # -- toolbar actions --------------------------------------------------

    def _save(self):
        self.backend.save()
        self.status.set('Saved %s' % self.backend.save_path)

    def _reload(self):
        self.backend.reload()
        self._refresh_joint_list()
        self._refresh_mesh_list()
        self.status.set('Reloaded %s' % self.backend.urdf_path)

    def _zero_origin(self):
        if self.joint is None:
            return
        self.backend.model.set_origin(self.joint['name'], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.backend.mark_meshes_dirty()
        self._load_origin(self.joint)
        self.status.set('Zeroed origin of %s' % self.joint['name'])

    def _toggle_axes(self):
        self.backend.set_axes_visible(self.axes_visible.get())

    # -- main loop --------------------------------------------------------

    def _tick(self):
        self.tick_count += 1
        self.backend.tick()
        rclpy.spin_once(self.backend.node, timeout_sec=0.0)
        if self.tick_count in (30, 90):
            self.backend.publish_description()
        if self.tick_count == 45:
            self.backend.warn_on_conflicts()
        self.root.after(33, self._tick)


def main(args=None):
    """Run the slider-based URDF tuner."""
    rclpy.init(args=args)
    node = Node('urdf_tuner_gui')
    node.declare_parameter('urdf_path', '')
    node.declare_parameter('save_path', '')
    node.declare_parameter('axis_length', 0.15)

    urdf_path = node.get_parameter('urdf_path').value
    if not urdf_path:
        raise RuntimeError('The urdf_path parameter must be set to a URDF file.')
    save_path = node.get_parameter('save_path').value or urdf_path
    axis_length = float(node.get_parameter('axis_length').value)

    backend = TunerBackend(node, urdf_path, save_path, axis_length=axis_length)
    app = TunerApp(backend)
    try:
        app.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
