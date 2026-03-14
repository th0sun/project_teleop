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
        play_frame(list)       – replay timer emits each joint frame
        play_started()
        play_stopped()
    """

    goto_waypoint = pyqtSignal(list)
    play_frame    = pyqtSignal(list)
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

        # Row buttons: Preview | Update | Del | ↑ | ↓
        btn_row = QHBoxLayout()
        self._btn_preview = _btn('Preview', '#1a3a5c', 'Jump to this waypoint')
        self._btn_update  = _btn('Update',  '#3a4a00', 'Overwrite with current joints')
        self._btn_del     = _btn('Delete',  '#5c1a1a')
        self._btn_up      = _btn('↑', '#333')
        self._btn_down    = _btn('↓', '#333')
        for b in [self._btn_preview, self._btn_update, self._btn_del,
                  self._btn_up, self._btn_down]:
            btn_row.addWidget(b)
        self._btn_preview.clicked.connect(self._preview_selected)
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
        play_box = QGroupBox('Playback')
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

        btn_play = _btn('▶  Play', '#1a4c6a')
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
        io_h.addWidget(btn_save)
        io_h.addWidget(btn_load)
        layout.addWidget(io_box)

    # ── Public API ────────────────────────────────────────────────────────────

    def set_current_joints(self, joints):
        """Called by main window whenever robot joints change."""
        self._current_joints = list(joints[:4])

    # ── Waypoint operations ───────────────────────────────────────────────────

    def _add_point(self):
        name = self._name_edit.text().strip()
        dur  = self._dur_spin.value()
        idx  = self._manager.add_waypoint(self._current_joints, name=name, duration=dur)
        self._name_edit.clear()
        self._refresh_list()
        self._list.setCurrentRow(idx)

    def _preview_selected(self):
        idx = self._list.currentRow()
        if idx < 0:
            return
        wp = self._manager.get(idx)
        self.goto_waypoint.emit(wp.joints)

    def _update_selected(self):
        idx = self._list.currentRow()
        if idx < 0:
            return
        self._manager.update_waypoint(idx, joints=self._current_joints)
        self._refresh_list()
        self._list.setCurrentRow(idx)

    def _delete_selected(self):
        idx = self._list.currentRow()
        if idx < 0:
            return
        self._manager.delete_waypoint(idx)
        self._refresh_list()

    def _move_up(self):
        idx = self._list.currentRow()
        if idx < 1:
            return
        self._manager.move_up(idx)
        self._refresh_list()
        self._list.setCurrentRow(idx - 1)

    def _move_down(self):
        idx = self._list.currentRow()
        if idx < 0 or idx >= self._manager.count() - 1:
            return
        self._manager.move_down(idx)
        self._refresh_list()
        self._list.setCurrentRow(idx + 1)

    def _clear_all(self):
        if self._manager.count() == 0:
            return
        r = QMessageBox.question(self, 'Clear', 'Delete all waypoints?')
        if r == QMessageBox.Yes:
            self._manager.clear()
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
                self._refresh_list()
            except Exception as e:
                QMessageBox.critical(self, 'Error', str(e))

    def get_ros_trajectory(self) -> dict:
        """Return ROS-compatible JointTrajectory dict."""
        return self._manager.to_joint_trajectory_dict()
