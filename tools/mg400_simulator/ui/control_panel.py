#!/usr/bin/env python3
"""
Right-side control panel for the MG400 Simulator.
"""

import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QSlider, QDoubleSpinBox, QPushButton, QComboBox,
    QSpinBox, QCheckBox, QFrame, QSizePolicy, QGridLayout,
    QScrollArea, QLineEdit
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui  import QFont, QColor, QPalette

from core.mg400_kinematics import JOINT_LIMITS, forward_kinematics


# ── Helpers ───────────────────────────────────────────────────────────────────

def _separator():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet('color: #3a3a3e;')
    return line


def _label(text, bold=False, size=9):
    lbl = QLabel(text)
    f   = QFont()
    f.setPointSize(size)
    f.setBold(bold)
    lbl.setFont(f)
    return lbl


def _status_badge(text, color='#555'):
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f'background:{color}; color:#eee; border-radius:3px;'
        f'padding:1px 6px; font-size:9px;'
    )
    lbl.setAlignment(Qt.AlignCenter)
    return lbl


# ── Joint row widget ──────────────────────────────────────────────────────────

class _JointRow(QWidget):
    """One row: label | slider | spinbox."""

    value_changed = pyqtSignal(int, float)   # joint_index, value_deg

    def __init__(self, idx: int, name: str, lo: float, hi: float, parent=None):
        super().__init__(parent)
        self._idx      = idx
        self._updating = False

        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(6)

        lbl = _label(name)
        lbl.setFixedWidth(28)
        h.addWidget(lbl)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(int(lo * 100), int(hi * 100))
        self._slider.setValue(0)
        self._slider.setFixedHeight(18)
        h.addWidget(self._slider, 1)

        self._spin = QDoubleSpinBox()
        self._spin.setRange(lo, hi)
        self._spin.setSingleStep(0.5)
        self._spin.setDecimals(1)
        self._spin.setSuffix('°')
        self._spin.setFixedWidth(72)
        h.addWidget(self._spin)

        self._slider.valueChanged.connect(self._on_slider)
        self._spin.valueChanged.connect(self._on_spin)

    def _on_slider(self, v):
        if self._updating:
            return
        self._updating = True
        val = v / 100.0
        self._spin.setValue(val)
        self._updating = False
        self.value_changed.emit(self._idx, val)

    def _on_spin(self, v):
        if self._updating:
            return
        self._updating = True
        self._slider.setValue(int(v * 100))
        self._updating = False
        self.value_changed.emit(self._idx, v)

    def set_value(self, deg: float):
        self._updating = True
        lo, hi = JOINT_LIMITS[self._idx]
        clamped = max(lo, min(hi, deg))
        self._slider.setValue(int(clamped * 100))
        self._spin.setValue(clamped)
        self._updating = False

    def get_value(self) -> float:
        return self._spin.value()


# ── Main control panel ────────────────────────────────────────────────────────

class ControlPanel(QWidget):
    """
    Signals:
        joints_changed(list[float])     – user moved a slider
        send_joints()                   – Send to Robot clicked
        command_triggered(str)          – dashboard command string
        speed_changed(int)              – 0-100
        suction_toggled(bool)
        control_mode_changed(str)
        ros_connect_requested(str, int, bool)
    """

    joints_changed         = pyqtSignal(list)
    send_joints            = pyqtSignal()
    command_triggered      = pyqtSignal(str)
    speed_changed          = pyqtSignal(int)
    suction_toggled        = pyqtSignal(bool)
    control_mode_changed   = pyqtSignal(str)
    ros_connect_requested  = pyqtSignal(str, int, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(310)

        scroll_area = QScrollArea(self)
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setStyleSheet('QScrollArea { border: none; }')

        inner = QWidget()
        self._layout = QVBoxLayout(inner)
        self._layout.setContentsMargins(8, 8, 8, 8)
        self._layout.setSpacing(8)

        self._build_status_section()
        self._layout.addWidget(_separator())
        self._build_joint_section()
        self._layout.addWidget(_separator())
        self._build_cartesian_section()
        self._layout.addWidget(_separator())
        self._build_motion_section()
        self._layout.addWidget(_separator())
        self._build_commands_section()
        self._layout.addWidget(_separator())
        self._build_io_section()
        self._layout.addWidget(_separator())
        self._build_ros_section()
        self._layout.addStretch(1)

        scroll_area.setWidget(inner)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll_area)

        self._joints = np.array([0.0, 0.0, -45.0, 0.0])

    # ── Section builders ──────────────────────────────────────────────────────

    def _build_status_section(self):
        box = QGroupBox('Robot Status')
        g   = QGridLayout(box)
        g.setSpacing(4)

        self._lbl_mode      = _status_badge('OFFLINE',  '#555')
        self._lbl_error     = _status_badge('OK',        '#2a6b2a')
        self._lbl_ros       = _status_badge('ROS OFF',   '#7a4000')
        self._lbl_ee_xyz    = _label('EE: — mm')
        self._lbl_ee_xyz.setStyleSheet('color:#9cf; font-size:9px;')

        g.addWidget(_label('Mode:'),       0, 0)
        g.addWidget(self._lbl_mode,        0, 1)
        g.addWidget(_label('Error:'),      1, 0)
        g.addWidget(self._lbl_error,       1, 1)
        g.addWidget(_label('ROS 2:'),      2, 0)
        g.addWidget(self._lbl_ros,         2, 1)
        g.addWidget(self._lbl_ee_xyz,      3, 0, 1, 2)

        self._layout.addWidget(box)

    def _build_joint_section(self):
        box = QGroupBox('Joint Control (degrees)')
        v   = QVBoxLayout(box)
        v.setSpacing(4)

        names = ['J1', 'J2', 'J3', 'J4']
        self._joint_rows: list[_JointRow] = []
        for i, (name, (lo, hi)) in enumerate(zip(names, JOINT_LIMITS)):
            row = _JointRow(i, name, lo, hi)
            row.value_changed.connect(self._on_joint_slider_changed)
            v.addWidget(row)
            self._joint_rows.append(row)

        # Set default values
        defaults = [0.0, 0.0, -45.0, 0.0]
        for row, val in zip(self._joint_rows, defaults):
            row.set_value(val)

        # Home button row
        h = QHBoxLayout()
        btn_home  = QPushButton('Home')
        btn_home.setToolTip('Move to J=[0,0,-45,0]')
        btn_home.clicked.connect(self._go_home)
        btn_zero  = QPushButton('Zero')
        btn_zero.setToolTip('Move to J=[0,0,0,0]')
        btn_zero.clicked.connect(self._go_zero)
        h.addWidget(btn_home)
        h.addWidget(btn_zero)
        v.addLayout(h)

        self._layout.addWidget(box)

    def _build_cartesian_section(self):
        box = QGroupBox('Cartesian Position (mm)')
        g   = QGridLayout(box)
        g.setSpacing(4)

        self._cart_labels = {}
        self._cart_spins  = {}
        for i, axis in enumerate(['X', 'Y', 'Z', 'Rz']):
            g.addWidget(_label(f'{axis}:'), i, 0)
            lbl = _label('—')
            lbl.setStyleSheet('color:#8cf; font-size:9px;')
            self._cart_labels[axis] = lbl
            g.addWidget(lbl, i, 1)

            spin = QDoubleSpinBox()
            if axis == 'Rz':
                spin.setRange(-360, 360); spin.setSuffix('°')
            else:
                spin.setRange(-500, 500); spin.setSuffix(' mm')
            spin.setDecimals(1)
            spin.setFixedWidth(90)
            self._cart_spins[axis] = spin
            g.addWidget(spin, i, 2)

        btn_goto = QPushButton('Go to XYZ')
        btn_goto.clicked.connect(self._go_to_cartesian)
        g.addWidget(btn_goto, 4, 0, 1, 3)

        self._layout.addWidget(box)

    def _build_motion_section(self):
        box = QGroupBox('Motion Settings')
        g   = QGridLayout(box)
        g.setSpacing(4)

        g.addWidget(_label('Speed:'), 0, 0)
        self._speed_slider = QSlider(Qt.Horizontal)
        self._speed_slider.setRange(1, 100)
        self._speed_slider.setValue(50)
        g.addWidget(self._speed_slider, 0, 1)
        self._speed_lbl = _label('50 %')
        self._speed_lbl.setFixedWidth(36)
        g.addWidget(self._speed_lbl, 0, 2)
        self._speed_slider.valueChanged.connect(self._on_speed_changed)

        g.addWidget(_label('Mode:'), 1, 0)
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(['jointmovj', 'movj', 'movl'])
        self._mode_combo.currentTextChanged.connect(
            lambda t: self.control_mode_changed.emit(t))
        g.addWidget(self._mode_combo, 1, 1, 1, 2)

        btn_send = QPushButton('▶  Send to Robot')
        btn_send.setStyleSheet(
            'background:#1a5c2a; color:#eee; font-weight:bold; padding:5px;')
        btn_send.clicked.connect(self.send_joints)
        g.addWidget(btn_send, 2, 0, 1, 3)

        self._layout.addWidget(box)

    def _build_commands_section(self):
        box = QGroupBox('Robot Commands')
        g   = QGridLayout(box)
        g.setSpacing(4)

        cmds = [
            ('Enable',       'EnableRobot()',   '#1a5c2a'),
            ('Disable',      'DisableRobot()',  '#4a3000'),
            ('Clear Error',  'ClearError()',    '#1a3a5c'),
            ('E-Stop',       'EmergencyStop()', '#6a1a1a'),
        ]
        for row, (label, cmd, color) in enumerate(cmds):
            btn = QPushButton(label)
            btn.setStyleSheet(f'background:{color}; color:#eee; padding:4px;')
            btn.clicked.connect(lambda _, c=cmd: self.command_triggered.emit(c))
            g.addWidget(btn, row // 2, row % 2)

        self._layout.addWidget(box)

    def _build_io_section(self):
        box = QGroupBox('I/O Control')
        g   = QGridLayout(box)
        g.setSpacing(4)

        # Suction row
        g.addWidget(_label('Suction:'), 0, 0)
        self._suction_btn = QPushButton('OFF')
        self._suction_btn.setCheckable(True)
        self._suction_btn.setStyleSheet(
            'QPushButton:checked { background:#1a5c2a; color:#eee; }'
            'QPushButton:!checked { background:#4a3000; color:#eee; }')
        self._suction_btn.toggled.connect(self._on_suction_toggle)
        g.addWidget(self._suction_btn, 0, 1, 1, 3)

        # Three independent light controls (DO 1 / DO 2 / DO 3)
        g.addWidget(_label('Lights:'), 1, 0)
        self._light_btns = []
        _LIGHT_DO = [1, 2, 3]
        for col, do_port in enumerate(_LIGHT_DO, start=1):
            btn = QPushButton(f'L{col} OFF')
            btn.setCheckable(True)
            btn.setStyleSheet(
                'QPushButton:checked { background:#5a5a00; color:#eee; }'
                'QPushButton:!checked { background:#333; color:#888; }')
            btn.toggled.connect(
                lambda on, p=do_port, b=btn: (
                    b.setText(f'L{p} ON' if on else f'L{p} OFF'),
                    self.command_triggered.emit(f'DO({p},{1 if on else 0})')
                ))
            self._light_btns.append(btn)
            g.addWidget(btn, 1, col)

        self._layout.addWidget(box)

    def _build_ros_section(self):
        box = QGroupBox('ROS 2 Bridge / Unity Mock')
        v   = QVBoxLayout(box)
        v.setSpacing(4)
        
        h = QHBoxLayout()
        h.addWidget(_label('Host:'))
        self._ros_host = QLineEdit('192.168.1.11')
        h.addWidget(self._ros_host)
        
        h.addWidget(_label('Port:'))
        self._ros_port = QSpinBox()
        self._ros_port.setRange(1, 65535)
        self._ros_port.setValue(10000)
        h.addWidget(self._ros_port)
        v.addLayout(h)

        self._ros_check = QCheckBox('Connect')
        self._ros_check.toggled.connect(
            lambda on: self.ros_connect_requested.emit(self._ros_host.text(), self._ros_port.value(), on))
        v.addWidget(self._ros_check)

        self._ros_status_lbl = _label('Not connected', size=8)
        self._ros_status_lbl.setStyleSheet('color:#888;')
        v.addWidget(self._ros_status_lbl)

        self._layout.addWidget(box)

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_joint_slider_changed(self, idx: int, val: float):
        from core.mg400_kinematics import ELBOW_ANGLE_LIMIT
        self._joints[idx] = val
        
        # Enforce J2/J3 relationship bidirectionally
        if idx == 1: # J2 moved, push J3
            j2 = val
            j3 = self._joints[2]
            if j3 - j2 < ELBOW_ANGLE_LIMIT[0]:
                self._joints[2] = j2 + ELBOW_ANGLE_LIMIT[0]
                self._joint_rows[2].set_value(self._joints[2])
            elif j3 - j2 > ELBOW_ANGLE_LIMIT[1]:
                self._joints[2] = j2 + ELBOW_ANGLE_LIMIT[1]
                self._joint_rows[2].set_value(self._joints[2])
                
        elif idx == 2: # J3 moved, push J2
            j2 = self._joints[1]
            j3 = val
            if j3 - j2 < ELBOW_ANGLE_LIMIT[0]:
                self._joints[1] = j3 - ELBOW_ANGLE_LIMIT[0]
                self._joint_rows[1].set_value(self._joints[1])
            elif j3 - j2 > ELBOW_ANGLE_LIMIT[1]:
                self._joints[1] = j3 - ELBOW_ANGLE_LIMIT[1]
                self._joint_rows[1].set_value(self._joints[1])

        self.joints_changed.emit(self._joints.tolist())
        self._update_cartesian_display()

    def _on_speed_changed(self, v: int):
        self._speed_lbl.setText(f'{v} %')
        self.speed_changed.emit(v)

    def _on_suction_toggle(self, on: bool):
        self._suction_btn.setText('ON' if on else 'OFF')
        self.suction_toggled.emit(on)

    def _go_home(self):
        home = [0.0, 0.0, -45.0, 0.0]
        self.set_joints(home)
        self.joints_changed.emit(home)

    def _go_zero(self):
        zero = [0.0, 0.0, 0.0, 0.0]
        self.set_joints(zero)
        self.joints_changed.emit(zero)

    def _go_to_cartesian(self):
        from core.mg400_kinematics import inverse_kinematics
        x  = self._cart_spins['X'].value()
        y  = self._cart_spins['Y'].value()
        z  = self._cart_spins['Z'].value()
        r  = self._cart_spins['Rz'].value()
        result = inverse_kinematics(x, y, z, r)
        if result is not None:
            self.set_joints(result.tolist())
            self.joints_changed.emit(result.tolist())
        else:
            self._lbl_error.setText('UNREACHABLE')
            self._lbl_error.setStyleSheet(
                'background:#6a1a1a; color:#eee; border-radius:3px;'
                'padding:1px 6px; font-size:9px;')

    # ── Public update methods ─────────────────────────────────────────────────

    def set_joints(self, joints_deg):
        self._joints = np.array(joints_deg[:4], dtype=float)
        for i, row in enumerate(self._joint_rows):
            row.set_value(float(self._joints[i]))
        self._update_cartesian_display()

    def _update_cartesian_display(self):
        pose = forward_kinematics(self._joints)
        px, py, pz, rx = pose[0], pose[1], pose[2], pose[3]

        self._cart_labels['X'].setText(f'{px:+.1f}')
        self._cart_labels['Y'].setText(f'{py:+.1f}')
        self._cart_labels['Z'].setText(f'{pz:+.1f}')
        self._cart_labels['Rz'].setText(f'{rx:+.1f}°')

        self._lbl_ee_xyz.setText(
            f'EE  X:{px:+.0f}  Y:{py:+.0f}  Z:{pz:+.0f} mm')

    def set_robot_mode(self, mode_str: str):
        colors = {'ENABLED': '#1a5c2a', 'DISABLED': '#4a3000',
                  'RUNNING': '#1a3a5c', 'OFFLINE': '#555'}
        self._lbl_mode.setText(mode_str)
        self._lbl_mode.setStyleSheet(
            f'background:{colors.get(mode_str, "#555")};'
            'color:#eee; border-radius:3px; padding:1px 6px; font-size:9px;')

    def set_error_status(self, ok: bool, msg: str = ''):
        if ok:
            self._lbl_error.setText('OK')
            self._lbl_error.setStyleSheet(
                'background:#2a6b2a; color:#eee; border-radius:3px;'
                'padding:1px 6px; font-size:9px;')
        else:
            self._lbl_error.setText(msg or 'ERROR')
            self._lbl_error.setStyleSheet(
                'background:#6a1a1a; color:#eee; border-radius:3px;'
                'padding:1px 6px; font-size:9px;')

    def set_ros_status(self, connected: bool):
        if connected:
            self._lbl_ros.setText('ROS ON')
            self._lbl_ros.setStyleSheet(
                'background:#1a5c2a; color:#eee; border-radius:3px;'
                'padding:1px 6px; font-size:9px;')
            self._ros_status_lbl.setText('Connected')
            self._ros_status_lbl.setStyleSheet('color:#8f8;font-size:8px;')
        else:
            self._lbl_ros.setText('ROS OFF')
            self._lbl_ros.setStyleSheet(
                'background:#7a4000; color:#eee; border-radius:3px;'
                'padding:1px 6px; font-size:9px;')
            self._ros_status_lbl.setText('Not connected')
            self._ros_status_lbl.setStyleSheet('color:#888;font-size:8px;')
