#!/usr/bin/env python3
"""
Main application window for the MG400 Simulator.
Layout: 3-D viewport (left, stretchy) | control panel (right, fixed 310px)
"""

import numpy as np
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QStatusBar, QAction, QMenuBar, QMessageBox,
    QTabWidget, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui  import QFont, QKeySequence

from ui.robot_viewport import RobotViewport
from ui.control_panel  import ControlPanel
from ui.teach_panel    import TeachPanel
from core.ros_bridge   import ROSBridge


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('MG400 Simulator')
        self.resize(1280, 760)

        self._ros = ROSBridge(
            on_joint_update=self._on_ros_joints,
            on_status_update=self._on_ros_status,
            on_connection_change=self._on_ros_connection,
        )

        self._build_ui()
        self._build_menu()
        self._connect_signals()

        # Status-bar refresh timer
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_status)
        self._timer.start(200)

    # ── UI Construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── 3-D viewport (left) ───────────────────────────────────────────
        vp_container = QWidget()
        vp_container.setStyleSheet('background:#12121a;')
        vp_layout = QVBoxLayout(vp_container)
        vp_layout.setContentsMargins(0, 0, 0, 0)
        vp_layout.setSpacing(0)

        # Thin header bar
        header = QWidget()
        header.setFixedHeight(28)
        header.setStyleSheet('background:#1a1a22; border-bottom:1px solid #2a2a35;')
        h_row = QHBoxLayout(header)
        h_row.setContentsMargins(10, 0, 10, 0)
        lbl_title = QLabel('MG400  ·  3-D Viewport')
        lbl_title.setStyleSheet('color:#aaa; font-size:10px;')
        self._lbl_hint = QLabel(
            '  LMB=orbit   MMB/Alt+LMB=pan   Scroll=zoom   '
            'Gizmo X/Y/Z=drag EE axis   F=focus EE   R=reset camera')
        self._lbl_hint.setStyleSheet('color:#666; font-size:9px;')
        h_row.addWidget(lbl_title)
        h_row.addStretch()
        h_row.addWidget(self._lbl_hint)

        self._viewport = RobotViewport()
        self._viewport.setSizePolicy(
            *[__import__('PyQt5.QtWidgets', fromlist=['QSizePolicy']).QSizePolicy.Expanding] * 2)

        vp_layout.addWidget(header)
        vp_layout.addWidget(self._viewport, 1)

        root.addWidget(vp_container, 1)

        # ── Right sidebar: tabbed control + teach ─────────────────────────
        sidebar = QWidget()
        sidebar.setFixedWidth(310)
        sidebar.setStyleSheet('background:#1c1c24;')
        sidebar_v = QVBoxLayout(sidebar)
        sidebar_v.setContentsMargins(0, 0, 0, 0)
        sidebar_v.setSpacing(0)

        self._tabs = QTabWidget()
        self._tabs.setStyleSheet(
            'QTabBar::tab { padding:4px 10px; font-size:9px; }'
            'QTabWidget::pane { border:none; }')

        self._panel = ControlPanel()
        self._panel.setStyleSheet('background:#1c1c24;')
        self._tabs.addTab(self._panel, 'Control')

        teach_scroll = QScrollArea()
        teach_scroll.setWidgetResizable(True)
        teach_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        teach_scroll.setStyleSheet('QScrollArea { border: none; }')
        self._teach = TeachPanel()
        self._teach.setStyleSheet('background:#1c1c24;')
        teach_scroll.setWidget(self._teach)
        self._tabs.addTab(teach_scroll, 'Teach & Repeat')

        sidebar_v.addWidget(self._tabs)
        root.addWidget(sidebar)

        # ── Status bar ────────────────────────────────────────────────────
        self.statusBar().setStyleSheet(
            'background:#14141c; color:#888; font-size:9px;')
        self._sb_joints = QLabel('J: —')
        self._sb_ee     = QLabel('EE: —')
        self._sb_ros    = QLabel('ROS: offline')
        for w in [self._sb_joints, self._sb_ee, self._sb_ros]:
            w.setStyleSheet('color:#888; padding:0 8px; font-size:9px;')
        self.statusBar().addWidget(self._sb_joints)
        self.statusBar().addWidget(self._sb_ee)
        self.statusBar().addPermanentWidget(self._sb_ros)

    def _build_menu(self):
        mb = self.menuBar()

        # File
        file_menu = mb.addMenu('&File')
        act_quit = QAction('&Quit', self)
        act_quit.setShortcut(QKeySequence('Ctrl+Q'))
        act_quit.triggered.connect(self.close)
        file_menu.addAction(act_quit)

        # View
        view_menu = mb.addMenu('&View')
        act_reset = QAction('Reset Camera  [R]', self)
        act_reset.triggered.connect(lambda: self._viewport.keyPressEvent(
            type('E', (), {'key': lambda s: Qt.Key_R, 'modifiers': lambda s: Qt.NoModifier})()))
        view_menu.addAction(act_reset)

        act_focus = QAction('Focus on EE  [F]', self)
        act_focus.triggered.connect(lambda: self._viewport.keyPressEvent(
            type('E', (), {'key': lambda s: Qt.Key_F, 'modifiers': lambda s: Qt.NoModifier})()))
        view_menu.addAction(act_focus)

        # Robot
        robot_menu = mb.addMenu('&Robot')
        act_home = QAction('Go Home', self)
        act_home.triggered.connect(lambda: self._panel._go_home())
        robot_menu.addAction(act_home)

        act_zero = QAction('Go Zero', self)
        act_zero.triggered.connect(lambda: self._panel._go_zero())
        robot_menu.addAction(act_zero)

        # Help
        help_menu = mb.addMenu('&Help')
        act_about = QAction('About', self)
        act_about.triggered.connect(self._show_about)
        help_menu.addAction(act_about)

    def _connect_signals(self):
        # Viewport → Panel
        self._viewport.joints_changed.connect(self._on_viewport_joints)
        self._viewport.ee_dragged.connect(self._on_ee_dragged)

        # Panel → Viewport
        self._panel.joints_changed.connect(self._on_panel_joints)

        # Teach panel signals
        self._teach.goto_waypoint.connect(self._on_teach_goto)
        self._teach.play_frame.connect(self._on_teach_frame)
        self._teach.play_started.connect(lambda: self._sb_ros.setText('▶ Playing...'))
        self._teach.play_stopped.connect(lambda: self._sb_ros.setText('ROS: offline'))

        # Panel → ROS / commands
        self._panel.send_joints.connect(self._send_joints_to_ros)
        self._panel.command_triggered.connect(self._send_command)
        self._panel.speed_changed.connect(
            lambda v: self._ros.publish_speed(v) if self._ros.connected else None)
        self._panel.suction_toggled.connect(
            lambda on: self._ros.publish_suction(on) if self._ros.connected else None)
        self._panel.control_mode_changed.connect(
            lambda m: self._ros.publish_control_mode(m) if self._ros.connected else None)
        self._panel.ros_connect_requested.connect(self._on_ros_toggle)

    # ── Signal handlers ───────────────────────────────────────────────────────

    @pyqtSlot(list)
    def _on_viewport_joints(self, joints: list):
        """Viewport (EE drag) → update panel sliders + stream to ROS."""
        self._panel.set_joints(joints)
        self._teach.set_current_joints(joints)
        if self._ros.connected:
            self._ros.publish_joint_cmd(joints)

    @pyqtSlot(float, float, float)
    def _on_ee_dragged(self, x, y, z):
        self._sb_ee.setText(f'EE drag → X:{x:+.0f}  Y:{y:+.0f}  Z:{z:+.0f} mm')

    @pyqtSlot(list)
    def _on_panel_joints(self, joints: list):
        """Panel slider → update viewport + stream to ROS."""
        self._viewport.set_joints(joints)
        self._teach.set_current_joints(joints)
        if self._ros.connected:
            self._ros.publish_joint_cmd(joints)

    @pyqtSlot(list)
    def _on_teach_goto(self, joints: list):
        """Teach panel: preview a waypoint."""
        self._viewport.set_joints(joints)
        self._panel.set_joints(joints)
        self._teach.set_current_joints(joints)

    @pyqtSlot(list)
    def _on_teach_frame(self, joints: list):
        """Teach panel: replay one frame."""
        self._viewport.set_joints(joints)
        self._panel.set_joints(joints)
        self._teach.set_current_joints(joints)

    @pyqtSlot()
    def _send_joints_to_ros(self):
        joints = self._viewport.get_joints()
        if self._ros.connected:
            self._ros.publish_joint_cmd(joints.tolist())
            self._viewport.set_ghost_joints(joints.tolist())
            self.statusBar().showMessage(
                f'Sent → J1:{joints[0]:.1f}°  J2:{joints[1]:.1f}°'
                f'  J3:{joints[2]:.1f}°  J4:{joints[3]:.1f}°', 3000)
        else:
            self.statusBar().showMessage('ROS 2 not connected — enable bridge first.', 3000)

    @pyqtSlot(str)
    def _send_command(self, cmd: str):
        if self._ros.connected:
            self._ros.publish_dashboard_cmd(cmd)
            self.statusBar().showMessage(f'CMD → {cmd}', 2000)
        else:
            self.statusBar().showMessage(f'(offline) CMD: {cmd}', 2000)

    @pyqtSlot(str, int, bool)
    def _on_ros_toggle(self, host: str, port: int, enable: bool):
        if enable:
            self.statusBar().showMessage(f"Connecting to {host}:{port}...", 2000)
            ok = self._ros.start(host, port)
            if not ok:
                QMessageBox.warning(self, 'Connection Error',
                    f'Failed to connect to Unity endpoint at {host}:{port}.\n'
                    'Ensure the ROS terminal (teleop_all.launch.py) is running on the target machine.')
                self._panel._ros_check.setChecked(False)
        else:
            self._ros.stop()

    # ── ROS callbacks (called from spin thread) ───────────────────────────────

    def _on_ros_joints(self, joints: list):
        from PyQt5.QtCore import QMetaObject, Q_ARG
        # Marshal to GUI thread
        QMetaObject.invokeMethod(self, '_apply_ros_joints',
                                 Qt.QueuedConnection,
                                 Q_ARG(object, joints))

    @pyqtSlot(object)
    def _apply_ros_joints(self, joints):
        self._viewport.set_joints(joints)
        self._panel.set_joints(joints)
        self._teach.set_current_joints(joints)

    def _on_ros_status(self, data: dict):
        from PyQt5.QtCore import QMetaObject, Q_ARG
        QMetaObject.invokeMethod(self, '_apply_ros_status',
                                 Qt.QueuedConnection,
                                 Q_ARG(object, data))

    @pyqtSlot(object)
    def _apply_ros_status(self, data: dict):
        mode = data.get('robot_mode', '')
        err  = data.get('error', False)
        self._panel.set_robot_mode(str(mode).upper() if mode else 'UNKNOWN')
        self._panel.set_error_status(not err, str(data.get('error_id', '')))

    def _on_ros_connection(self, connected: bool):
        from PyQt5.QtCore import QMetaObject, Q_ARG
        QMetaObject.invokeMethod(self, '_apply_ros_connection',
                                 Qt.QueuedConnection,
                                 Q_ARG(bool, connected))

    @pyqtSlot(bool)
    def _apply_ros_connection(self, connected: bool):
        self._panel.set_ros_status(connected)
        self._sb_ros.setText('ROS: online' if connected else 'ROS: offline')

    # ── Periodic refresh ──────────────────────────────────────────────────────

    def _refresh_status(self):
        j = self._viewport.get_joints()
        self._sb_joints.setText(
            f'J1:{j[0]:+.1f}°  J2:{j[1]:+.1f}°  '
            f'J3:{j[2]:+.1f}°  J4:{j[3]:+.1f}°')

    # ── Misc ──────────────────────────────────────────────────────────────────

    def _show_about(self):
        QMessageBox.about(self, 'MG400 Simulator',
            '<b>MG400 3-D Simulator</b><br>'
            'Part of the MG400 VR Teleop project.<br><br>'
            '<b>Viewport controls:</b><br>'
            '• Left-drag = orbit camera<br>'
            '• Middle-drag / Alt+Left = pan<br>'
            '• Scroll wheel = zoom<br>'
            '• Hover over red/green/blue gizmo arrow = highlight<br>'
            '• Click &amp; drag gizmo arrow = move EE along that world axis (IK)<br>'
            '• F = focus camera on EE<br>'
            '• R = reset camera<br><br>'
            '<b>ROS 2:</b> source workspace then enable the bridge in the panel.')

    def closeEvent(self, event):
        self._ros.stop()
        super().closeEvent(event)
