#!/usr/bin/env python3
"""
MG400 3-D Simulator – entry point.

Usage:
    # Standalone (no ROS):
    cd tools/mg400_simulator
    pip install -r requirements.txt
    python3 main.py

    # With ROS 2 bridge:
    source ~/teleop_mg400_ws/install/setup.bash
    python3 tools/mg400_simulator/main.py
"""

import sys
import os

# Allow imports from this directory when run directly
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore    import Qt
from PyQt5.QtGui     import QPalette, QColor, QFont

from ui.main_window import MainWindow


def _dark_palette() -> QPalette:
    pal = QPalette()
    roles = {
        QPalette.Window:          '#1c1c24',
        QPalette.WindowText:      '#d0d0d8',
        QPalette.Base:            '#13131a',
        QPalette.AlternateBase:   '#1a1a22',
        QPalette.ToolTipBase:     '#2a2a38',
        QPalette.ToolTipText:     '#d0d0d8',
        QPalette.Text:            '#d0d0d8',
        QPalette.Button:          '#2a2a38',
        QPalette.ButtonText:      '#d0d0d8',
        QPalette.BrightText:      '#ffffff',
        QPalette.Link:            '#5da8ff',
        QPalette.Highlight:       '#2a5aaa',
        QPalette.HighlightedText: '#ffffff',
    }
    for role, hex_color in roles.items():
        pal.setColor(role, QColor(hex_color))
    # Disabled state – slightly muted
    for role, hex_color in roles.items():
        c = QColor(hex_color)
        c.setAlphaF(0.45)
        pal.setColor(QPalette.Disabled, role, c)
    return pal


def main():
    # High-DPI support
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps,    True)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setPalette(_dark_palette())

    f = QFont('Segoe UI', 9)
    app.setFont(f)

    app.setStyleSheet("""
        QGroupBox {
            border: 1px solid #2e2e3e;
            border-radius: 4px;
            margin-top: 10px;
            font-size: 9pt;
            font-weight: bold;
            color: #aaa;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            padding: 0 3px;
        }
        QPushButton {
            background: #2a2a38;
            color: #d0d0d8;
            border: 1px solid #3a3a50;
            border-radius: 3px;
            padding: 4px 10px;
            font-size: 9pt;
        }
        QPushButton:hover  { background: #35354a; }
        QPushButton:pressed { background: #1e1e2c; }
        QDoubleSpinBox, QSpinBox, QComboBox {
            background: #1a1a26;
            color: #d0d0d8;
            border: 1px solid #3a3a50;
            border-radius: 3px;
            padding: 2px 4px;
            font-size: 9pt;
        }
        QSlider::groove:horizontal {
            height: 4px;
            background: #2e2e3e;
            border-radius: 2px;
        }
        QSlider::handle:horizontal {
            width: 12px;
            height: 12px;
            background: #5080cc;
            border-radius: 6px;
            margin: -4px 0;
        }
        QSlider::sub-page:horizontal { background: #3a5a9a; border-radius: 2px; }
        QScrollBar:vertical {
            background: #14141c;
            width: 8px;
        }
        QScrollBar::handle:vertical {
            background: #3a3a50;
            border-radius: 4px;
            min-height: 20px;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        QMenuBar { background: #14141c; color: #bbb; }
        QMenuBar::item:selected { background: #2a2a38; }
        QMenu { background: #1c1c28; color: #d0d0d8; border: 1px solid #3a3a50; }
        QMenu::item:selected { background: #2a4a8a; }
        QStatusBar { background: #14141c; color: #666; }
        QCheckBox { color: #aaa; font-size: 9pt; }
        QCheckBox::indicator {
            width: 14px; height: 14px;
            background: #1a1a26;
            border: 1px solid #3a3a50;
            border-radius: 2px;
        }
        QCheckBox::indicator:checked { background: #2a5aaa; }
    """)

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
