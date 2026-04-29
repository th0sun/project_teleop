#!/usr/bin/env python3
"""
Teach & Repeat Panel for MG400 Simulator.
Records waypoints, edits sequences, and replays P2P or smooth trajectories.
"""

import os
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QListWidget, QListWidgetItem, QDoubleSpinBox,
    QLineEdit, QComboBox, QFileDialog, QMessageBox, QSlider,
    QGridLayout, QAbstractItemView
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui  import QFont, QColor

from core.teach_manager import TeachManager


def _btn(text, color='#333', tip=''):
    b = QPushButton(text)
    b.setStyleSheet(f'background:{color}; color:#eee; padding:3px 6px; font-size:9px;')
    if tip:
        b.setToolTip(tip)
    return b


class TeachPanel(QWidget):
    """
    Signals:
        goto_waypoint(list)    – user selects / wants to preview a waypoint
        send_waypoint(list)    – user wants to send one waypoint to ROS
        play_frame(list)       – replay timer emits each joint frame
        teach_job_requested(str, dict) – action + Unity-format trajectory
        play_started()
        play_stopped()
    """

    goto_waypoint = pyqtSignal(list)
    send_waypoint = pyqtSignal(list)
    play_frame    = pyqtSignal(list)
    teach_job_requested = pyqtSignal(str, dict)
    play_started  = pyqtSignal()
    play_stopped  = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._manager = TeachManager()
        self._current_joints = [0.0, 0.0, -45.0, 0.0]

        self._play_frames  = []
        self._play_idx     = 0
        self._play_timer   = QTimer(self)
        self._play_timer.setInterval(20)   # 50 fps
        self._play_timer.timeout.connect(self._on_play_tick)
        self._job_timeout_timer = QTimer(self)
        self._job_timeout_timer.setSingleShot(True)
        self._job_timeout_timer.timeout.connect(self._on_job_timeout)
        self._job_validated = False
        self._job_busy = False

        self._build_ui()

    # ── Build ──────────────────────────────────────────────────────────────────

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        # ── Record ──────────────────────────────────────────────────────────
        rec_box = QGroupBox('Record')
        rec_g   = QGridLayout(rec_box)
        rec_g.setSpacing(4)

        rec_g.addWidget(QLabel('Name:'), 0, 0)
        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText('auto')
        self._name_edit.setFixedHeight(22)
        rec_g.addWidget(self._name_edit, 0, 1, 1, 2)

        rec_g.addWidget(QLabel('Duration (s):'), 1, 0)
        self._dur_spin = QDoubleSpinBox()
        self._dur_spin.setRange(0.1, 30.0)
        self._dur_spin.setValue(1.0)
        self._dur_spin.setSingleStep(0.1)
        self._dur_spin.setDecimals(1)
        self._dur_spin.setFixedHeight(22)
        rec_g.addWidget(self._dur_spin, 1, 1, 1, 2)

        btn_add = _btn('+ Add Point', '#1a5c2a', 'Record current joints as waypoint')
        btn_add.setFixedHeight(26)
        btn_add.clicked.connect(self._add_point)
        rec_g.addWidget(btn_add, 2, 0, 1, 3)

        layout.addWidget(rec_box)

        # ── Waypoint list ────────────────────────────────────────────────────
        list_box = QGroupBox('Waypoints')
        list_v   = QVBoxLayout(list_box)
        list_v.setSpacing(4)

        self._list = QListWidget()
        self._list.setFixedHeight(120)
        self._list.setStyleSheet('font-size:9px; background:#1e1e28; color:#ddd;')
        self._list.setSelectionMode(QAbstractItemView.SingleSelection)
        self._list.itemClicked.connect(self._on_item_click)
        list_v.addWidget(self._list)

        # Row buttons: Preview | Send | Update | Del | ↑ | ↓
        btn_row = QHBoxLayout()
        self._btn_preview = _btn('Preview', '#1a3a5c', 'Show this waypoint in the 3-D view only')
        self._btn_send    = _btn('Send', '#1a5c2a', 'Send this waypoint as a live /unity/joint_cmd')
        self._btn_update  = _btn('Update',  '#3a4a00', 'Overwrite with current joints')
        self._btn_del     = _btn('Delete',  '#5c1a1a')
        self._btn_up      = _btn('↑', '#333')
        self._btn_down    = _btn('↓', '#333')
        for b in [self._btn_preview, self._btn_send, self._btn_update, self._btn_del,
                  self._btn_up, self._btn_down]:
            btn_row.addWidget(b)
        self._btn_preview.clicked.connect(self._preview_selected)
        self._btn_send.clicked.connect(self._send_selected)
        self._btn_update.clicked.connect(self._update_selected)
        self._btn_del.clicked.connect(self._delete_selected)
        self._btn_up.clicked.connect(self._move_up)
        self._btn_down.clicked.connect(self._move_down)
        list_v.addLayout(btn_row)

        btn_clear = _btn('Clear All', '#4a2020')
        btn_clear.clicked.connect(self._clear_all)
        list_v.addWidget(btn_clear)

        layout.addWidget(list_box)

        # ── Playback ─────────────────────────────────────────────────────────
        play_box = QGroupBox('Live Preview Playback')
        play_g   = QGridLayout(play_box)
        play_g.setSpacing(4)

        play_g.addWidget(QLabel('Mode:'), 0, 0)
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(['Smooth (ease)', 'Point-to-Point (linear)'])
        play_g.addWidget(self._mode_combo, 0, 1, 1, 2)

        play_g.addWidget(QLabel('Speed:'), 1, 0)
        self._speed_sl = QSlider(Qt.Horizontal)
        self._speed_sl.setRange(10, 300)   # 0.1× – 3×
        self._speed_sl.setValue(100)
        play_g.addWidget(self._speed_sl, 1, 1)
        self._speed_lbl = QLabel('1.0×')
        self._speed_lbl.setFixedWidth(32)
        play_g.addWidget(self._speed_lbl, 1, 2)
        self._speed_sl.valueChanged.connect(
            lambda v: self._speed_lbl.setText(f'{v/100:.1f}×'))

        btn_play = _btn('▶  Play Live', '#1a4c6a',
                        'Animate the 3-D view and stream /unity/joint_cmd if connected')
        btn_play.setFixedHeight(26)
        btn_play.clicked.connect(self._start_play)

        self._btn_stop = _btn('■  Stop', '#5c3000')
        self._btn_stop.setFixedHeight(26)
        self._btn_stop.setEnabled(False)
        self._btn_stop.clicked.connect(self._stop_play)

        play_g.addWidget(btn_play,       2, 0, 1, 2)
        play_g.addWidget(self._btn_stop, 2, 2)

        self._progress_lbl = QLabel('')
        self._progress_lbl.setStyleSheet('color:#888; font-size:8px;')
        play_g.addWidget(self._progress_lbl, 3, 0, 1, 3)

        layout.addWidget(play_box)

        # ── Save / Load ───────────────────────────────────────────────────────
        io_box = QGroupBox('Save / Load Program')
        io_h   = QHBoxLayout(io_box)

        btn_save = _btn('💾 Save', '#2a3a4a')
        btn_save.clicked.connect(self._save)
        btn_load = _btn('📂 Load', '#2a3a4a')
        btn_load.clicked.connect(self._load)
        btn_import = _btn('Import Unity JSON', '#2a4a3a',
                          'Load frames from Unity trajectory JSON')
        btn_import.clicked.connect(self._import_unity_json)
        io_h.addWidget(btn_save)
        io_h.addWidget(btn_load)
        io_h.addWidget(btn_import)
        layout.addWidget(io_box)

        # ── ROS teach job bridge ───────────────────────────────────────────
        job_box = QGroupBox('ROS Teach Job')
        job_g = QGridLayout(job_box)
        job_g.setSpacing(4)

        self._btn_validate = _btn('Validate Plan', '#1f4f64',
                                  'Compile/check the teach job; does not move the robot')
        self._btn_execute = _btn('Execute', '#5c301a',
                                 'Send execute job to ROS; viewport follows robot feedback when connected')
        self._btn_validate.clicked.connect(lambda: self._request_teach_job('compile'))
        self._btn_execute.clicked.connect(lambda: self._request_teach_job('execute'))

        job_g.addWidget(self._btn_validate, 0, 0, 1, 2)
        job_g.addWidget(self._btn_execute, 1, 0, 1, 2)

        self._job_status_lbl = QLabel(
            'Validate first. Execute is enabled after the plan passes.')
        self._job_status_lbl.setStyleSheet('color:#888; font-size:8px;')
        job_g.addWidget(self._job_status_lbl, 2, 0, 1, 2)
        layout.addWidget(job_box)
        self._set_job_state('idle')

    # ── Public API ────────────────────────────────────────────────────────────

    def set_current_joints(self, joints):
        """Called by main window whenever robot joints change."""
        self._current_joints = list(joints[:4])

    def set_job_status(self, text: str):
        self._job_status_lbl.setText(text)

    def apply_job_status(self, data: dict):
        stage = str(data.get('stage', '') or '').lower()
        action = str(data.get('action', '') or '').lower()
        message = str(data.get('message', '') or '')
        error_code = data.get('error_code')
        metadata = data.get('metadata') if isinstance(data.get('metadata'), dict) else {}

        if stage == 'received':
            self._set_job_state('busy', f"Received {action or 'job'}...")
            return

        if stage in {'compiled', 'preview_ready'}:
            self._job_timeout_timer.stop()
            count = metadata.get('queued_command_count')
            duration = metadata.get('total_duration_s')
            detail = message or 'Plan validated'
            if count is not None and duration is not None:
                detail = f'Plan ready: {count} command(s), {float(duration):.2f}s'
            self._set_job_state('ready', detail)
            return

        if stage == 'executing':
            self._set_job_state('executing', message or 'Executing teach job...')
            return

        if stage == 'done':
            self._job_timeout_timer.stop()
            self._set_job_state('done', message or 'Teach job complete')
            return

        if stage == 'stopped':
            self._job_timeout_timer.stop()
            self._set_job_state('stopped', message or 'Teach job stopped')
            return

        if stage == 'failed':
            self._job_timeout_timer.stop()
            suffix = f' [{error_code}]' if error_code else ''
            self._set_job_state('failed', (message or 'Teach job failed') + suffix)
            return

        if message:
            self._job_status_lbl.setText(message)

    # ── Waypoint operations ───────────────────────────────────────────────────

    def _add_point(self):
        name = self._name_edit.text().strip()
        dur  = self._dur_spin.value()
        idx  = self._manager.add_waypoint(self._current_joints, name=name, duration=dur)
        self._invalidate_plan('Waypoint changed. Validate again before Execute.')
        self._name_edit.clear()
        self._refresh_list()
        self._list.setCurrentRow(idx)

    def _preview_selected(self):
        idx = self._list.currentRow()
        if idx < 0:
            return
        wp = self._manager.get(idx)
        self.goto_waypoint.emit(wp.joints)

    def _send_selected(self):
        idx = self._list.currentRow()
        if idx < 0:
            return
        wp = self._manager.get(idx)
        self.send_waypoint.emit(wp.joints)

    def _update_selected(self):
        idx = self._list.currentRow()
        if idx < 0:
            return
        self._manager.update_waypoint(idx, joints=self._current_joints)
        self._invalidate_plan('Waypoint updated. Validate again before Execute.')
        self._refresh_list()
        self._list.setCurrentRow(idx)

    def _delete_selected(self):
        idx = self._list.currentRow()
        if idx < 0:
            return
        self._manager.delete_waypoint(idx)
        self._invalidate_plan('Waypoint deleted. Validate again before Execute.')
        self._refresh_list()

    def _move_up(self):
        idx = self._list.currentRow()
        if idx < 1:
            return
        self._manager.move_up(idx)
        self._invalidate_plan('Waypoint order changed. Validate again before Execute.')
        self._refresh_list()
        self._list.setCurrentRow(idx - 1)

    def _move_down(self):
        idx = self._list.currentRow()
        if idx < 0 or idx >= self._manager.count() - 1:
            return
        self._manager.move_down(idx)
        self._invalidate_plan('Waypoint order changed. Validate again before Execute.')
        self._refresh_list()
        self._list.setCurrentRow(idx + 1)

    def _clear_all(self):
        if self._manager.count() == 0:
            return
        r = QMessageBox.question(self, 'Clear', 'Delete all waypoints?')
        if r == QMessageBox.Yes:
            self._manager.clear()
            self._invalidate_plan('No validated plan.')
            self._refresh_list()

    def _on_item_click(self, item):
        pass

    def _refresh_list(self):
        self._list.clear()
        for i, wp in enumerate(self._manager.get_all()):
            text = (f'{i+1:2d}. {wp.name:<10}  '
                    f'J=[{wp.joints[0]:+.0f},{wp.joints[1]:+.0f},'
                    f'{wp.joints[2]:+.0f},{wp.joints[3]:+.0f}]  '
                    f'{wp.duration:.1f}s')
            self._list.addItem(QListWidgetItem(text))

    # ── Playback ──────────────────────────────────────────────────────────────

    def _start_play(self):
        if self._manager.count() < 2:
            QMessageBox.information(self, 'Teach & Repeat',
                                    'Need at least 2 waypoints to play.')
            return
        speed = self._speed_sl.value() / 100.0
        smooth = self._mode_combo.currentIndex() == 0
        if smooth:
            self._play_frames = self._manager.generate_smooth(speed_mult=speed)
        else:
            self._play_frames = self._manager.generate_p2p(speed_mult=speed)

        self._play_idx = 0
        self._btn_stop.setEnabled(True)
        self._play_timer.start()
        self.play_started.emit()

    def _stop_play(self):
        self._play_timer.stop()
        self._play_frames = []
        self._btn_stop.setEnabled(False)
        self._progress_lbl.setText('')
        self.play_stopped.emit()

    def _on_play_tick(self):
        if self._play_idx >= len(self._play_frames):
            self._stop_play()
            self._progress_lbl.setText('Done')
            return
        joints = self._play_frames[self._play_idx].tolist()
        self.play_frame.emit(joints)
        total = len(self._play_frames)
        self._progress_lbl.setText(
            f'Frame {self._play_idx + 1}/{total}  ({100*(self._play_idx+1)//total}%)')
        self._play_idx += 1

    # ── Save / Load ───────────────────────────────────────────────────────────

    def _save(self):
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save Program', os.path.expanduser('~'),
            'MG400 Program (*.mg400json);;JSON (*.json)')
        if path:
            if not path.endswith(('.json', '.mg400json')):
                path += '.mg400json'
            try:
                self._manager.save(path)
                QMessageBox.information(self, 'Saved', f'Program saved to:\n{path}')
            except Exception as e:
                QMessageBox.critical(self, 'Error', str(e))

    def _load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Load Program', os.path.expanduser('~'),
            'MG400 Program (*.mg400json);;JSON (*.json)')
        if path:
            try:
                self._manager.load(path)
                self._invalidate_plan('Program loaded. Validate before Execute.')
                self._refresh_list()
            except Exception as e:
                QMessageBox.critical(self, 'Error', str(e))

    def _import_unity_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Import Unity Trajectory JSON', os.path.expanduser('~'),
            'Unity Trajectory JSON (*.json);;All Files (*)')
        if not path:
            return
        try:
            count = self._manager.load_unity_trajectory(path)
            self._invalidate_plan('Unity trajectory imported. Validate before Execute.')
            self._refresh_list()
            if count:
                self._list.setCurrentRow(0)
            QMessageBox.information(
                self,
                'Imported',
                f'Imported {count} Unity waypoint(s) from:\n{path}',
            )
        except Exception as e:
            QMessageBox.critical(self, 'Import Error', str(e))

    def get_ros_trajectory(self) -> dict:
        """Return ROS-compatible JointTrajectory dict."""
        return self._manager.to_joint_trajectory_dict()

    def get_unity_trajectory(self) -> dict:
        """Return TeachJobPublisher-compatible Unity trajectory JSON."""
        return self._manager.to_unity_trajectory_dict()

    def _request_teach_job(self, action: str):
        if self._manager.count() < 2:
            QMessageBox.information(self, 'ROS Teach Job',
                                    'Need at least 2 waypoints before sending a teach job.')
            return
        if action == 'execute' and not self._job_validated:
            QMessageBox.information(self, 'ROS Teach Job',
                                    'Validate the plan first. Execute is enabled only after validation passes.')
            return
        trajectory = self.get_unity_trajectory()
        if action == 'compile':
            self._set_job_state('busy', f'Validating {len(trajectory["frames"])} waypoint(s)...')
            self._job_timeout_timer.start(12000)
        elif action == 'execute':
            self._set_job_state('executing', f'Execute requested: {len(trajectory["frames"])} waypoint(s)')
            self._job_timeout_timer.start(45000)
        self.teach_job_requested.emit(action, trajectory)

    def _invalidate_plan(self, message: str):
        if self._job_busy:
            return
        self._job_validated = False
        self._set_job_state('idle', message)

    def _set_job_state(self, state: str, message: str = ''):
        state = state.lower()
        self._job_busy = state in {'busy', 'executing'}
        if state == 'ready':
            self._job_validated = True
        elif state in {'idle', 'failed'}:
            self._job_validated = False

        self._btn_validate.setEnabled(not self._job_busy)
        self._btn_execute.setEnabled(self._job_validated and not self._job_busy)

        colors = {
            'idle': '#888',
            'busy': '#e0b84f',
            'ready': '#76d48a',
            'executing': '#8fc7ff',
            'done': '#76d48a',
            'stopped': '#e0b84f',
            'failed': '#ff8a8a',
        }
        labels = {
            'idle': 'Not validated',
            'busy': 'Validating',
            'ready': 'Ready to execute',
            'executing': 'Executing',
            'done': 'Done',
            'stopped': 'Stopped',
            'failed': 'Failed',
        }
        text = message or labels.get(state, state)
        self._job_status_lbl.setText(f'{labels.get(state, state)}: {text}')
        self._job_status_lbl.setStyleSheet(
            f'color:{colors.get(state, "#888")}; font-size:8px;')

    def _on_job_timeout(self):
        self._set_job_state(
            'failed',
            'No status returned from ROS. Check ROS-TCP endpoint / teleop logs.',
        )
