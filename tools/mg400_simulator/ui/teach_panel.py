#!/usr/bin/env python3
"""
Teach & Repeat Panel for MG400 Simulator.
Records waypoints, edits sequences, and replays P2P or smooth trajectories.
"""

import os
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QListWidget, QListWidgetItem,
    QLineEdit, QFileDialog, QMessageBox,
    QGridLayout, QAbstractItemView, QSlider
)
from PyQt5.QtCore import pyqtSignal, QTimer, Qt

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
        teach_job_requested(str, dict, dict) – action + trajectory + tuning options
        teach_tuning_requested(dict) – live speed/acc/CP update while executing
    """

    goto_waypoint = pyqtSignal(list)
    send_waypoint = pyqtSignal(list)
    teach_job_requested = pyqtSignal(str, dict, dict)
    teach_tuning_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._manager = TeachManager()
        self._current_joints = [0.0, 0.0, -45.0, 0.0]

        self._job_timeout_timer = QTimer(self)
        self._job_timeout_timer.setSingleShot(True)
        self._job_timeout_timer.timeout.connect(self._on_job_timeout)
        self._job_validated = False
        self._job_busy = False
        self._job_state = 'idle'
        self._tuning_controls = []

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

        btn_add = _btn('+ Add Point', '#1a5c2a', 'Record current joints as waypoint')
        btn_add.setFixedHeight(26)
        btn_add.clicked.connect(self._add_point)
        rec_g.addWidget(btn_add, 1, 0, 1, 3)

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

        # ── Save / Load ───────────────────────────────────────────────────────
        io_box = QGroupBox('Program Files')
        io_h   = QHBoxLayout(io_box)

        btn_save = _btn('Save Program', '#2a3a4a',
                        'Save this GUI waypoint program as .mg400json')
        btn_save.clicked.connect(self._save)
        btn_load = _btn('Load Program', '#2a3a4a',
                        'Load a .mg400json program saved by this simulator')
        btn_load.clicked.connect(self._load)
        btn_import = _btn('Load Unity JSON', '#2a4a3a',
                          'Import raw Unity trajectory JSON frames')
        btn_import.clicked.connect(self._import_unity_json)
        io_h.addWidget(btn_save)
        io_h.addWidget(btn_load)
        io_h.addWidget(btn_import)
        layout.addWidget(io_box)

        # ── ROS teach job bridge ───────────────────────────────────────────
        job_box = QGroupBox('ROS Teach Job')
        job_g = QGridLayout(job_box)
        job_g.setSpacing(4)

        self._speed_factor_slider, self._speed_factor_value = self._make_tuning_slider(
            1, 100, 50, 'Global Dobot SpeedFactor; applied immediately when connected')
        self._speed_j_slider, self._speed_j_value = self._make_tuning_slider(
            1, 100, 40, 'Joint speed for future JointMovJ commands')
        self._acc_j_slider, self._acc_j_value = self._make_tuning_slider(
            1, 100, 80, 'Joint acceleration for future JointMovJ commands')
        self._speed_l_slider, self._speed_l_value = self._make_tuning_slider(
            1, 100, 40, 'Cartesian speed for future MovL/Arc commands')
        self._acc_l_slider, self._acc_l_value = self._make_tuning_slider(
            1, 100, 80, 'Cartesian acceleration for future MovL/Arc commands')
        self._cp_slider, self._cp_value = self._make_tuning_slider(
            0, 100, 30, 'Blend CP for future commands')

        self._add_tuning_row(job_g, 0, 'SpeedFactor:', self._speed_factor_slider, self._speed_factor_value)
        self._add_tuning_row(job_g, 1, 'SpeedJ:', self._speed_j_slider, self._speed_j_value)
        self._add_tuning_row(job_g, 2, 'AccJ:', self._acc_j_slider, self._acc_j_value)
        self._add_tuning_row(job_g, 3, 'SpeedL:', self._speed_l_slider, self._speed_l_value)
        self._add_tuning_row(job_g, 4, 'AccL:', self._acc_l_slider, self._acc_l_value)
        self._add_tuning_row(job_g, 5, 'CP:', self._cp_slider, self._cp_value)

        self._btn_validate = _btn('Validate Plan', '#1f4f64',
                                  'Compile/check the teach job; does not move the robot')
        self._btn_execute = _btn('Execute', '#5c301a',
                                 'Send execute job to ROS; viewport follows robot feedback when connected')
        self._btn_validate.clicked.connect(lambda: self._request_teach_job('compile'))
        self._btn_execute.clicked.connect(lambda: self._request_teach_job('execute'))

        job_g.addWidget(self._btn_validate, 6, 0, 1, 3)
        job_g.addWidget(self._btn_execute, 7, 0, 1, 3)

        self._job_status_lbl = QLabel(
            'Validate first. Execute is enabled after the plan passes.')
        self._job_status_lbl.setStyleSheet('color:#888; font-size:8px;')
        job_g.addWidget(self._job_status_lbl, 8, 0, 1, 3)
        layout.addWidget(job_box)
        self._set_job_state('idle')

    def _make_tuning_slider(self, lo: int, hi: int, value: int, tip: str):
        slider = QSlider(Qt.Horizontal)
        slider.setRange(lo, hi)
        slider.setValue(value)
        slider.setMinimumWidth(170)
        slider.setToolTip(tip)
        value_lbl = QLabel(f'{value}%')
        value_lbl.setMinimumWidth(34)
        value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        slider.valueChanged.connect(lambda v, lbl=value_lbl: lbl.setText(f'{v}%'))
        slider.valueChanged.connect(self._on_tuning_changed)
        self._tuning_controls.append(slider)
        return slider, value_lbl

    @staticmethod
    def _add_tuning_row(grid: QGridLayout, row: int, label: str, slider: QSlider, value_lbl: QLabel):
        grid.addWidget(QLabel(label), row, 0)
        grid.addWidget(slider, row, 1)
        grid.addWidget(value_lbl, row, 2)

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

        if stage == 'tuned':
            self._job_timeout_timer.stop()
            self._set_job_state('executing', message or 'Playback tuning updated')
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
            detail = self._status_detail(metadata)
            text = (message or 'Teach job failed') + suffix
            if detail:
                text += f' | {detail}'
            self._set_job_state('failed', text)
            return

        if message:
            self._job_status_lbl.setText(message)

    @staticmethod
    def _status_detail(metadata: dict) -> str:
        parts = []
        if 'final_max_error_deg' in metadata and metadata.get('final_max_error_deg') is not None:
            try:
                parts.append(f"final err {float(metadata['final_max_error_deg']):.2f}°")
            except (TypeError, ValueError):
                pass
        if 'final_robot_mode' in metadata and metadata.get('final_robot_mode') is not None:
            parts.append(f"mode {metadata['final_robot_mode']}")
        if 'elapsed_s' in metadata and metadata.get('elapsed_s') is not None:
            try:
                parts.append(f"{float(metadata['elapsed_s']):.1f}s")
            except (TypeError, ValueError):
                pass
        if metadata.get('queue_flushed'):
            parts.append('queue flushed')
        return ', '.join(parts)

    # ── Waypoint operations ───────────────────────────────────────────────────

    def _add_point(self):
        name = self._name_edit.text().strip()
        dur = 1.0
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
                    f'{wp.joints[2]:+.0f},{wp.joints[3]:+.0f}]')
            self._list.addItem(QListWidgetItem(text))

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
        options = self._tuning_options()
        if action == 'compile':
            self._set_job_state('busy', f'Validating {len(trajectory["frames"])} waypoint(s)...')
            self._job_timeout_timer.start(12000)
        elif action == 'execute':
            self._set_job_state('executing', f'Execute requested: {len(trajectory["frames"])} waypoint(s)')
            self._job_timeout_timer.start(45000)
        self.teach_job_requested.emit(action, trajectory, options)

    def _tuning_options(self) -> dict:
        return {
            'use_recorded_timing': False,
            'speed_factor': int(self._speed_factor_slider.value()),
            'speed_j': int(self._speed_j_slider.value()),
            'acc_j': int(self._acc_j_slider.value()),
            'speed_l': int(self._speed_l_slider.value()),
            'acc_l': int(self._acc_l_slider.value()),
            'cp': int(self._cp_slider.value()),
            'final_cp': 0,
            'command_interval_s': 0.18,
            'queue_lookahead_commands': 3,
        }

    def _on_tuning_changed(self, *_):
        if self._job_state == 'executing':
            self.teach_tuning_requested.emit(self._tuning_options())
            self._job_status_lbl.setText('Executing: tuning update sent for future commands')

    def _invalidate_plan(self, message: str):
        if self._job_busy:
            return
        self._job_validated = False
        self._set_job_state('idle', message)

    def _set_job_state(self, state: str, message: str = ''):
        state = state.lower()
        self._job_state = state
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
