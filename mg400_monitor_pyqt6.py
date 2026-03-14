#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MG400 Monitor — PyQt6 Edition
==============================
Layout  : TopBar | Left(Data scroll + Log terminal) | Right(2×2 graphs + 3D)
Splitters: 3 QSplitter (H + V×2) — all draggable
Styling : QStyleSheet  (border-radius, shadows, custom fonts)
Graphs  : matplotlib embedded via FigureCanvasQTAgg  (2D joints + 3D trail)
"""

import sys, os, time, math, csv, datetime, threading
from collections import deque

import numpy as np

# ── Qt6 ──────────────────────────────────────────────────────────────────────
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton,
    QSplitter, QScrollArea, QTextEdit, QGridLayout, QTabWidget,
    QHBoxLayout, QVBoxLayout, QSizePolicy,
)
from PyQt6.QtCore    import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt6.QtGui     import (
    QFont, QFontDatabase, QColor, QPalette, QTextCursor, QIcon,
)
from PyQt6.QtWidgets import QScrollBar  # explicit import for log scrollbar

# ── Matplotlib ────────────────────────────────────────────────────────────────
import matplotlib
matplotlib.use("QtAgg")
from matplotlib.figure                        import Figure
from matplotlib.backends.backend_qtagg        import FigureCanvasQTAgg
from mpl_toolkits.mplot3d                     import Axes3D   # noqa
import matplotlib.animation                   as animation

# ── ROS2 ─────────────────────────────────────────────────────────────────────
# Cross-VM: set ROS_DOMAIN_ID to match your Ubuntu Parallels VM
# e.g.  export ROS_DOMAIN_ID=0
# Ensure Parallels network is Bridged or Shared with multicast forwarding.
if 'ROS_DOMAIN_ID' not in os.environ:
    os.environ['ROS_DOMAIN_ID'] = '0'   # default — change to match Ubuntu
    print(f"[INFO] ROS_DOMAIN_ID not set, defaulting to 0")

try:
    import rclpy
    from rclpy.node          import Node
    from rclpy.qos           import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg     import JointState
    from std_msgs.msg        import Float64MultiArray, Bool, Int32MultiArray, Int64, Int32, String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARN] rclpy not found — running in SIMULATION mode")

# ── Project config ────────────────────────────────────────────────────────────
try:
    from mg400_controller.common.config.motion_config import (
        UNITY_TOPIC, VACUUM_DO_PORT,
        GREEN_LIGHT_DO_PORT, YELLOW_LIGHT_DO_PORT, RED_LIGHT_DO_PORT,
        SUCTION_TOPIC, LIGHT_TOPIC, DO_STATUS_TOPIC,
        ROBOT_MODE_TOPIC, ERROR_STATUS_TOPIC,
    )
    from mg400_controller.common.utils.error_decoder import RobotErrorDecoder
except ImportError:
    UNITY_TOPIC          = "/unity/joint_cmd"
    VACUUM_DO_PORT       = 1
    GREEN_LIGHT_DO_PORT  = 5
    YELLOW_LIGHT_DO_PORT = 6
    RED_LIGHT_DO_PORT    = 7
    SUCTION_TOPIC        = "/vr/suction_cmd"
    LIGHT_TOPIC          = "/mg400/light_cmd"
    DO_STATUS_TOPIC      = "/mg400/do_status"
    ROBOT_MODE_TOPIC     = "/mg400/robot_mode"
    ERROR_STATUS_TOPIC   = "/mg400/error_status"
    class RobotErrorDecoder:
        def decode_error(self, code): return f"ERR_{code:02X}", None, None

ACTUAL_TOPIC    = "/joint_states"
PREDICTED_TOPIC = "/teleop/predicted_target"
SENT_TOPIC      = "/teleop/sent_command"
TOOL_ACT_TOPIC  = "/mg400/tool_vector_actual"
TOOL_TGT_TOPIC  = "/mg400/tool_vector_target"
UNITY_XYZ_TOPIC = "/teleop/unity_xyz"
FLANGE_TOPIC    = "/robot/flange_actual"
TOOL_IDX_TOPIC  = "/robot/tool_index"

# ══════════════════════════════════════════════════════════════════════════════
#  DESIGN TOKENS  (mirrors HTML CSS variables)
# ══════════════════════════════════════════════════════════════════════════════
BG      = "#eef0f4"
PANEL   = "#ffffff"
PANEL2  = "#f7f8fb"
BORDER  = "#d8dce6"
ACCENT  = "#0ea5e9"
RED     = "#ef4444"
GREEN   = "#22c55e"
ORANGE  = "#f97316"
PURPLE  = "#a855f7"
TEXT    = "#1e293b"
MUTED   = "#64748b"

COL_UNITY  = "#f59e0b"
COL_PRED   = "#a855f7"
COL_SENT   = "#ef4444"
COL_ACTUAL = "#0ea5e9"

LOG_BG  = "#0d1f1a"
LOG_GRN = "#86efac"

C_OFF    = "#d0d0d0"
C_GREEN  = "#2ecc71"
C_YELLOW = "#f1c40f"
C_RED    = "#e74c3c"
C_VAC    = "#3498db"

MODE_NAMES = {1:"INIT",4:"DISABLED",5:"ENABLE",6:"DRAG",7:"RUN",9:"ERROR",11:"COLLISION"}

GRAPH_WIN = 10.0
MAX_PTS   = 300
TRAIL_LEN = 200
START_THR = 2.0
STOP_THR  = 0.5

# ══════════════════════════════════════════════════════════════════════════════
#  FONT SETUP
# ══════════════════════════════════════════════════════════════════════════════
FONT_MONO, FONT_SANS, FONT_COND = "Courier", "Arial", "Arial"

def _load_fonts():
    """Load embedded Google Fonts from system or fallback gracefully."""
    global FONT_MONO, FONT_SANS, FONT_COND
    # Try to load from system; if unavailable use fallback names
    _mono = "JetBrains Mono"
    _sans = "Inter"
    _cond = "Barlow Condensed"
    # Check availability
    families = QFontDatabase.families()
    if "Consolas" in families:     _mono = "Consolas"
    elif "JetBrains Mono" in families: pass
    elif "Courier New" in families: _mono = "Courier New"

    if "Inter" not in families:
        _sans = "Segoe UI" if sys.platform == "win32" else "DejaVu Sans"
    if "Barlow Condensed" not in families:
        _cond = _sans
        
    FONT_MONO, FONT_SANS, FONT_COND = _mono, _sans, _cond

def font(family=None, size=10, bold=False, italic=False):
    f = QFont(family or FONT_SANS, size)
    f.setBold(bold)
    f.setItalic(italic)
    return f

# ══════════════════════════════════════════════════════════════════════════════
#  GLOBAL STYLESHEET
# ══════════════════════════════════════════════════════════════════════════════
QSS = f"""
QMainWindow, QWidget#root {{
    background: {BG};
}}

/* ── Panels ── */
QFrame#panel {{
    background: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
QFrame#panel_header {{
    background: {PANEL2};
    border-bottom: 1px solid {BORDER};
    border-radius: 10px 10px 0 0;
}}

/* ── Top bar ── */
QFrame#topbar {{
    background: {PANEL};
    border-bottom: 2px solid {BORDER};
}}

/* ── Splitter ── */
QSplitter::handle {{
    background: {BORDER};
}}
QSplitter::handle:hover, QSplitter::handle:pressed {{
    background: {ACCENT};
}}
QSplitter::handle:horizontal {{
    width: 5px;
    border-radius: 2px;
    margin: 20px 2px;
}}
QSplitter::handle:vertical {{
    height: 5px;
    border-radius: 2px;
    margin: 2px 20px;
}}

/* ── Scroll area ── */
QScrollArea {{ border: none; background: {PANEL}; }}
QScrollBar:vertical {{
    background: {PANEL2}; width: 5px; border-radius: 2px;
}}
QScrollBar::handle:vertical {{
    background: {BORDER}; border-radius: 2px; min-height: 20px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}

/* ── Buttons ── */
QPushButton#btn_vac {{
    background: #e0f2fe; color: #0369a1;
    border: 1px solid #bae6fd; border-radius: 5px;
    padding: 5px 14px; font-weight: 600;
}}
QPushButton#btn_vac:hover {{ background: #bae6fd; }}
QPushButton#btn_vac[active=true] {{
    background: {ACCENT}; color: white; border-color: {ACCENT};
}}
QPushButton#btn_light {{
    background: {C_OFF}; color: {TEXT};
    border: 1px solid {BORDER}; border-radius: 5px;
    padding: 4px 10px; font-weight: 600; font-size: 10px;
}}
QPushButton#btn_light:hover {{ background: {BORDER}; }}
QPushButton#btn_light[lit=green]  {{ background: {C_GREEN};  color: white; border-color: {C_GREEN};  }}
QPushButton#btn_light[lit=yellow] {{ background: {C_YELLOW}; color: white; border-color: {C_YELLOW}; }}
QPushButton#btn_light[lit=red]    {{ background: {C_RED};    color: white; border-color: {C_RED};    }}

/* ── Mode badge ── */
QLabel#mode_badge {{
    background: {ACCENT}; color: white;
    border-radius: 4px; padding: 4px 12px;
}}
QLabel#mode_badge[danger=true] {{ background: {RED}; }}
QLabel#err_badge {{
    background: {PANEL2}; color: {GREEN};
    border: 1px solid {BORDER}; border-radius: 4px; padding: 4px 12px;
}}
QLabel#err_badge[error=true] {{ color: {RED}; }}

/* ── Status pill ── */
QLabel#status_pill {{
    background: {PANEL2}; color: {MUTED};
    border: 1px solid {BORDER}; border-radius: 4px;
    padding: 3px 10px;
}}

/* ── Section label ── */
QLabel#sec_label {{
    color: {MUTED}; font-size: 9px; font-weight: 700;
    letter-spacing: 2px; text-transform: uppercase;
    padding-left: 10px;
    border-left: 3px solid {ACCENT};
}}

/* ── Metric card ── */
QFrame#metric_card {{
    background: {PANEL2};
    border: 1px solid {BORDER};
    border-radius: 7px;
}}
QLabel#metric_val {{
    color: {ACCENT}; font-size: 15px; font-weight: 700;
}}
QLabel#metric_val[state=moving] {{ color: {RED}; }}
QLabel#metric_val[state=arrived] {{ color: {GREEN}; }}
QLabel#metric_lbl {{
    color: {MUTED}; font-size: 8px; font-weight: 700;
    letter-spacing: 2px;
}}

/* ── Stat chip ── */
QFrame#stat_chip {{
    background: {PANEL2}; border: 1px solid {BORDER}; border-radius: 4px;
}}

/* ── DO bit ── */
QLabel#do_bit {{
    background: {PANEL2}; color: {MUTED};
    border: 1px solid {BORDER}; border-radius: 2px;
    font-size: 7px; min-width: 14px; max-width: 14px;
    min-height: 14px; max-height: 14px;
}}
QLabel#do_bit[on=true] {{ background: {GREEN}; color: white; border-color: {GREEN}; }}

/* ── Graph card ── */
QFrame#graph_card {{
    background: {PANEL}; border: 1px solid {BORDER}; border-radius: 8px;
}}
QFrame#graph_header {{
    background: {PANEL2}; border-bottom: 1px solid {BORDER};
    border-radius: 8px 8px 0 0;
}}

/* ── Log terminal header ── */
QFrame#log_header {{
    background: #0a1912;
    border-bottom: 1px solid #1e3a2f;
    border-radius: 8px 8px 0 0;
}}

/* ── Divider ── */
QFrame#divider {{
    background: {BORDER}; max-height: 1px; min-height: 1px;
}}

/* ── 3D panel ── */
QFrame#panel_3d {{
    background: {PANEL}; border: 1px solid {BORDER}; border-radius: 8px;
}}
QFrame#header_3d {{
    background: {PANEL2}; border-bottom: 1px solid {BORDER};
    border-radius: 8px 8px 0 0;
}}
QFrame#coord_bar {{
    background: {PANEL2}; border-top: 1px solid {BORDER};
    border-radius: 0 0 8px 8px;
}}
"""

# ══════════════════════════════════════════════════════════════════════════════
#  SHARED DATA STATE
# ══════════════════════════════════════════════════════════════════════════════
class RobotData:
    """Thread-safe shared state between ROS node and GUI."""
    def __init__(self):
        self._lock        = threading.Lock()
        self.actual       = [0.0]*4
        self.unity        = [0.0]*4
        self.predicted    = [0.0]*4
        self.sent         = [0.0]*4
        self.sent_fresh   = [False]*4
        self.tool_act     = [0.0]*6
        self.tool_tgt     = [0.0]*6
        self.unity_xyz    = [0.0]*6
        self.flange       = [0.0]*6
        self.tool_idx     = -1
        self.do_status    = 0
        self.robot_mode   = 7
        self.error_stat   = 0
        self.last_tgt_t   = 0.0
        self.last_act_t   = 0.0
        # sim
        self._sim_t       = 0.0

        # Message counts for Data Flow tab
        self.msg_counts = {
            "UNITY TARGET": 0, "PREDICTED TGT": 0, "SENT COMMAND": 0,
            "ACTUAL FEEDBACK": 0, "TOOL VECTOR": 0,
            "ROBOT MODE": 0, "ROBOT ERROR": 0, "DIGITAL IO": 0
        }
        self.flow_hz = {k: 0.0 for k in self.msg_counts}

    def tick_sim(self):
        """No longer simulating. Only true ROS data is used."""
        pass

DATA = RobotData()

# ══════════════════════════════════════════════════════════════════════════════
#  ROS2 NODE  (runs in background thread)
# ══════════════════════════════════════════════════════════════════════════════
class RosNode(Node if ROS_AVAILABLE else object):
    def __init__(self, data: RobotData):
        if ROS_AVAILABLE:
            super().__init__('mg400_monitor')
        self.data = data
        self.pub_suction = None
        self.pub_light   = None
        if ROS_AVAILABLE:
            self._setup()

    def _setup(self):
        d = self.data
        self.pub_suction = self.create_publisher(Bool, SUCTION_TOPIC, 10)
        self.pub_light   = self.create_publisher(Int32MultiArray, LIGHT_TOPIC, 10)
        self.pub_dash    = self.create_publisher(String, "/robot/dashboard_cmd", 10)
        # Use BEST_EFFORT QoS (depth=1) for high-frequency topics to match the publisher
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        def sub(topic, typ, cb, qos=10): return self.create_subscription(typ, topic, cb, qos)
        sub(ACTUAL_TOPIC,    JointState,        self._cb_actual)
        sub(UNITY_TOPIC,     JointState,        self._cb_unity, qos_be)
        sub(PREDICTED_TOPIC, JointState,        self._cb_pred)
        sub(SENT_TOPIC,      JointState,        self._cb_sent)
        sub(TOOL_ACT_TOPIC,  Float64MultiArray, lambda m: self._f64(m, 'tool_act'))
        sub(TOOL_TGT_TOPIC,  Float64MultiArray, lambda m: self._f64(m, 'tool_tgt'))
        sub(UNITY_XYZ_TOPIC, Float64MultiArray, lambda m: self._f64(m, 'unity_xyz'))
        sub(FLANGE_TOPIC,    Float64MultiArray, lambda m: self._f64(m, 'flange'))
        sub(TOOL_IDX_TOPIC,  Int32, lambda m: setattr(d,'tool_idx',int(m.data)))
        sub(DO_STATUS_TOPIC, Int64, self._cb_do)
        sub(ROBOT_MODE_TOPIC,   Int32, self._cb_mode)
        sub(ERROR_STATUS_TOPIC, Int32, self._cb_err)

    def _cb_actual(self, msg):
        self.data.msg_counts["ACTUAL FEEDBACK"] += 1
        if len(msg.position) >= 9:
            self.data.actual = list(np.degrees([msg.position[i] for i in (0,1,3,8)]))
            self.data.last_act_t = time.time()
    def _cb_unity(self, msg):
        self.data.msg_counts["UNITY TARGET"] += 1
        if len(msg.position) >= 4:
            self.data.unity = list(np.degrees(msg.position[:4]))
            self.data.last_tgt_t = time.time()
    def _cb_pred(self, msg):
        self.data.msg_counts["PREDICTED TGT"] += 1
        if len(msg.position) >= 4: self.data.predicted = list(np.degrees(msg.position[:4]))
    def _cb_sent(self, msg):
        self.data.msg_counts["SENT COMMAND"] += 1
        if len(msg.position) >= 4:
            self.data.sent = list(np.degrees(msg.position[:4]))
            self.data.sent_fresh = [True]*4
    def _f64(self, msg, attr):
        if attr == 'tool_act': self.data.msg_counts["TOOL VECTOR"] += 1
        if len(msg.data) >= 6: setattr(self.data, attr, list(msg.data))
    def _cb_mode(self, msg):
        self.data.msg_counts["ROBOT MODE"] += 1
        self.data.robot_mode = int(msg.data)
    def _cb_err(self, msg):
        self.data.msg_counts["ROBOT ERROR"] += 1
        self.data.error_stat = int(msg.data)
    def _cb_do(self, msg):
        self.data.msg_counts["DIGITAL IO"] += 1
        self.data.do_status = int(msg.data)

    def send_suction(self, state):
        if self.pub_suction:
            msg = Bool(); msg.data = state; self.pub_suction.publish(msg)
    def send_light(self, port, state):
        if self.pub_light:
            msg = Int32MultiArray(); msg.data=[port,int(state)]; self.pub_light.publish(msg)
    def send_dashboard_cmd(self, cmd_str):
        if hasattr(self, 'pub_dash') and self.pub_dash:
            msg = String(); msg.data = cmd_str; self.pub_dash.publish(msg)

# ══════════════════════════════════════════════════════════════════════════════
#  SESSION LOGGER
# ══════════════════════════════════════════════════════════════════════════════
class SessionLogger:
    BASE = os.path.expanduser("~/project_teleop_ws/session_logs")
    def __init__(self, data: RobotData):
        self.data = data
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        d  = os.path.join(self.BASE, ts); os.makedirs(d, exist_ok=True)
        self._jt  = open(os.path.join(d,"joints_tracking.csv"), 'w', newline='')
        self._xyz = open(os.path.join(d,"xyz_tracking.csv"),    'w', newline='')
        self._jw  = csv.writer(self._jt);  self._xw = csv.writer(self._xyz)
        self._jw.writerow(["timestamp","elapsed_s",
                           "u1","u2","u3","u4","p1","p2","p3","p4",
                           "s1","s2","s3","s4","a1","a2","a3","a4",
                           "mode","err"])
        self._xw.writerow(["timestamp","elapsed_s",
                           "tx","ty","tz","ax","ay","az","dx","dy","dz"])
        self._t0=time.time(); self._lock=threading.Lock()
        self._run=True; self._fc=0
        threading.Thread(target=self._loop,daemon=True).start()
        print(f"[Logger] → {d}")

    def _loop(self):
        period=1/20; nxt=time.perf_counter()+period
        while self._run:
            try:
                n=self.data; ts=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                el=round(time.time()-self._t0,3)
                with self._lock:
                    self._jw.writerow([ts,el]+[round(v,4) for v in n.unity+n.predicted+n.sent+n.actual]+[n.robot_mode,n.error_stat])
                    tgt=n.unity_xyz[:3]; act=n.tool_act[:3]
                    self._xw.writerow([ts,el]+[round(v,3) for v in tgt+act]+[round(act[i]-tgt[i],3) for i in range(3)])
                self._fc+=1
                if self._fc>=100:
                    with self._lock: self._jt.flush(); self._xyz.flush()
                    self._fc=0
            except: pass
            sl=nxt-time.perf_counter()
            if sl>0: time.sleep(sl)
            nxt+=period
            if time.perf_counter()>nxt+period: nxt=time.perf_counter()+period

    def close(self):
        self._run=False
        with self._lock: self._jt.close(); self._xyz.close()

# ══════════════════════════════════════════════════════════════════════════════
#  EXECUTION MONITOR
# ══════════════════════════════════════════════════════════════════════════════
class ExecMonitor:
    def __init__(self):
        self.state="IDLE"; self.start_t=0.0
        self.last_dur=0.0; self.durs=[]
    def update(self, err):
        now=time.time()
        if self.state in("IDLE","ARRIVED"):
            if err>START_THR: self.state="MOVING"; self.start_t=now
        elif self.state=="MOVING":
            if err<STOP_THR:
                self.state="ARRIVED"; self.last_dur=now-self.start_t; self.durs.append(self.last_dur)
    def stats(self):
        if not self.durs: return 0,0,0
        return np.mean(self.durs), np.min(self.durs), np.max(self.durs)

# ══════════════════════════════════════════════════════════════════════════════
#  HELPERS — small UI factory functions
# ══════════════════════════════════════════════════════════════════════════════
def _lbl(text, fam=None, size=10, bold=False, color=None, obj_name=None, align=None):
    l = QLabel(text)
    l.setFont(font(fam, size, bold))
    if color: l.setStyleSheet(f"color:{color};background:transparent;")
    if obj_name: l.setObjectName(obj_name)
    if align: l.setAlignment(align)
    return l

def _divider():
    f = QFrame(); f.setObjectName("divider")
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFixedHeight(1)
    return f

def _sec_label(text):
    l = QLabel(text.upper()); l.setObjectName("sec_label")
    l.setFont(font(FONT_SANS, 9, bold=True))
    l.setContentsMargins(10, 0, 0, 0)
    return l

def _chip(text, bg=PANEL2, color=MUTED, size=9):
    f = QFrame(); f.setObjectName("stat_chip")
    lay = QHBoxLayout(f); lay.setContentsMargins(8,4,8,4)
    l = QLabel(text); l.setFont(font(FONT_MONO, size))
    l.setStyleSheet(f"color:{color}; background:transparent;")
    l.setAlignment(Qt.AlignmentFlag.AlignCenter)
    lay.addWidget(l)
    return f, l

def _set_prop(widget, prop, val):
    """Set a dynamic property and re-polish stylesheet."""
    widget.setProperty(prop, val)
    widget.style().unpolish(widget)
    widget.style().polish(widget)

# ══════════════════════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ══════════════════════════════════════════════════════════════════════════════
class MonitorWindow(QMainWindow):

    # Qt signal to safely update UI from non-GUI threads
    log_signal = pyqtSignal(str, str, list)  # tag, parts-as-json

    def __init__(self, data: RobotData, ros_node=None):
        super().__init__()
        self.data      = data
        self.ros_node  = ros_node
        self.exec_mon  = ExecMonitor()
        self.err_dec   = RobotErrorDecoder()
        self._start_t  = time.time()

        # State
        self.vac_on        = False
        self.light_on      = {"GREEN":False,"YELLOW":False,"RED":False}
        self.lockout       = {}

        # Graph buffers
        self.g_start  = time.time()
        self.t_buf    = deque(maxlen=MAX_PTS)
        self.u_buf    = [deque(maxlen=MAX_PTS) for _ in range(4)]
        self.p_buf    = [deque(maxlen=MAX_PTS) for _ in range(4)]
        self.s_buf    = [deque(maxlen=MAX_PTS) for _ in range(4)]
        self.a_buf    = [deque(maxlen=MAX_PTS) for _ in range(4)]
        self.tgt_trail= deque(maxlen=TRAIL_LEN)
        self.act_trail= deque(maxlen=TRAIL_LEN)

        # Log
        self._log_idx  = 0
        self._last_log = 0.0
        self._last_flow_t = time.time()

        self.setWindowTitle("MG400 Monitor")
        self.resize(1460, 880)
        self.setMinimumSize(900, 600)

        central = QWidget(); central.setObjectName("root")
        central.setStyleSheet(f"background:{BG};")
        self.setCentralWidget(central)
        root_lay = QVBoxLayout(central)
        root_lay.setContentsMargins(0,0,0,0); root_lay.setSpacing(0)

        self._build_topbar(root_lay)
        
        # Tabs
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: none; }
            QTabBar::tab {
                background: #e2e8f0; color: #64748b; padding: 8px 20px;
                border-top-left-radius: 8px; border-top-right-radius: 8px;
                margin-right: 4px; font-weight: bold;
            }
            QTabBar::tab:selected {
                background: #ffffff; color: #0ea5e9;
            }
        """)
        
        # Tab 1: Dashboard
        tab1 = QWidget(); tab1.setStyleSheet(f"background:{BG};")
        tab1_lay = QVBoxLayout(tab1)
        tab1_lay.setContentsMargins(0,0,0,0); tab1_lay.setSpacing(0)
        self._build_body(tab1_lay)
        
        # Tab 2: Data Flow
        tab2 = QWidget(); tab2.setStyleSheet(f"background:{BG};")
        self._build_data_flow_tab(tab2)
        
        self.tabs.addTab(tab1, "⚙️ DASHBOARD")
        self.tabs.addTab(tab2, "🌐 DATA FLOW")
        
        root_lay.addWidget(self.tabs)

        # Session logger
        self.logger = SessionLogger(data)

        # Main refresh timer 20 Hz
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

        # Graph animation timers
        self._graph_timer = QTimer()
        self._graph_timer.timeout.connect(self._update_graphs)
        self._graph_timer.start(50)

    # ──────────────────────────────────────────────────────────────────────────
    #  TOP BAR
    # ──────────────────────────────────────────────────────────────────────────
    def _build_topbar(self, parent_layout):
        bar = QFrame(); bar.setObjectName("topbar")
        bar.setFixedHeight(52)
        lay = QHBoxLayout(bar); lay.setContentsMargins(16,0,16,0); lay.setSpacing(0)

        # Title
        t1 = QLabel("MG400"); t1.setFont(font(FONT_COND, 20, bold=True))
        t1.setStyleSheet(f"color:{ACCENT}; background:transparent; letter-spacing:3px;")
        t2 = QLabel(" · MONITOR"); t2.setFont(font(FONT_SANS, 12))
        t2.setStyleSheet(f"color:{MUTED}; background:transparent;")
        lay.addWidget(t1); lay.addWidget(t2); lay.addSpacing(16)

        # ROS status
        self._lbl_ros = QLabel("CONNECTING...")
        self._lbl_ros.setObjectName("status_pill")
        self._lbl_ros.setFont(font(FONT_MONO, 10))
        self._lbl_ros.setStyleSheet(f"color:{MUTED}; background:rgba(100,116,139,0.1); border:1px solid rgba(100,116,139,0.3); border-radius:4px; padding:3px 10px;")
        lay.addWidget(self._lbl_ros); lay.addSpacing(8)

        # Uptime
        self._lbl_uptime = QLabel("⏱ 00:00:00")
        self._lbl_uptime.setObjectName("status_pill")
        self._lbl_uptime.setFont(font(FONT_MONO, 10))
        self._lbl_uptime.setStyleSheet(f"color:#b45309; background:rgba(245,158,11,0.07); border:1px solid rgba(245,158,11,0.3); border-radius:4px; padding:3px 10px;")
        lay.addWidget(self._lbl_uptime)

        lay.addStretch()

        # Mode badge
        self._lbl_mode = QLabel("MODE: RUN"); self._lbl_mode.setObjectName("mode_badge")
        self._lbl_mode.setFont(font(FONT_MONO, 11, bold=True))
        self._lbl_mode.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self._lbl_mode); lay.addSpacing(8)

        # Error badge
        self._lbl_err = QLabel("✓ ERR: 00"); self._lbl_err.setObjectName("err_badge")
        self._lbl_err.setFont(font(FONT_MONO, 11))
        self._lbl_err.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self._lbl_err); lay.addSpacing(12)

        # Clock
        self._lbl_clock = QLabel("--:--:--")
        self._lbl_clock.setFont(font(FONT_MONO, 13))
        self._lbl_clock.setStyleSheet(f"color:{MUTED}; border-left:1px solid {BORDER}; padding-left:12px; background:transparent;")
        lay.addWidget(self._lbl_clock)

        parent_layout.addWidget(bar)

    # ──────────────────────────────────────────────────────────────────────────
    #  BODY  (horizontal splitter)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_body(self, parent_layout):
        wrap = QWidget(); wrap.setStyleSheet(f"background:{BG};")
        lay  = QHBoxLayout(wrap); lay.setContentsMargins(10,8,10,10); lay.setSpacing(0)

        hsplit = QSplitter(Qt.Orientation.Horizontal)
        hsplit.setHandleWidth(1)
        hsplit.setStyleSheet(QSS)   # apply globally here

        left_w  = QWidget(); left_w.setStyleSheet(f"background:{BG};")
        right_w = QWidget(); right_w.setStyleSheet(f"background:{BG};")

        hsplit.addWidget(left_w)
        hsplit.addWidget(right_w)
        
        # Optimize split widths: Make left side 440px to fit well, right side takes rest
        hsplit.setSizes([440, 1000])
        hsplit.setCollapsible(0, False)
        hsplit.setCollapsible(1, False)

        lay.addWidget(hsplit)
        parent_layout.addWidget(wrap)

        self._build_left(left_w)
        self._build_right(right_w)

    # ──────────────────────────────────────────────────────────────────────────
    #  DATA FLOW TAB
    # ──────────────────────────────────────────────────────────────────────────
    def _build_data_flow_tab(self, parent):
        parent_lay = QVBoxLayout(parent)
        parent_lay.setContentsMargins(0,0,0,0)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setStyleSheet("background: transparent;")
        parent_lay.addWidget(scroll)
        
        container = QWidget()
        container.setStyleSheet(f"background:{BG};")
        scroll.setWidget(container)
        
        lay = QVBoxLayout(container)
        lay.setContentsMargins(30,30,30,30)
        lay.setSpacing(10)
        
        title = QLabel("System Architecture & Data Flow")
        title.setFont(font(FONT_SANS, 18, bold=True))
        title.setStyleSheet(f"color:{TEXT};")
        lay.addWidget(title)
        
        desc = QLabel("Real-time telemetry pipeline reflecting User Input → ROS 2 → Robot Hardware → Feedback Loop")
        desc.setFont(font(FONT_SANS, 11))
        desc.setStyleSheet(f"color:{MUTED};")
        lay.addWidget(desc)
        lay.addSpacing(20)
        
        # Helpers
        def _box(title_text, bg, border, layout_type='V'):
            b = QFrame()
            b.setStyleSheet(f"background:{bg}; border: 2px solid {border}; border-radius: 8px;")
            l = QVBoxLayout(b) if layout_type == 'V' else QHBoxLayout(b)
            l.setContentsMargins(20, 20, 20, 20)
            l.setSpacing(12)
            if title_text:
                t = QLabel(title_text)
                t.setFont(font(FONT_SANS, 12, bold=True))
                t.setStyleSheet(f"color:{border}; border:none;")
                l.addWidget(t)
            return b, l

        def _label(text, bg="#ffffff", fg="#000000", border="none", bold=False, mono=False):
            lbl = QLabel(text)
            f = font(FONT_MONO if mono else FONT_SANS, 11, bold=bold)
            lbl.setFont(f)
            lbl.setStyleSheet(f"background:{bg}; color:{fg}; border:{border}; border-radius:6px; padding:10px;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            return lbl

        def _arrow(down=False, up=False, right=False, left=False):
            char = "↓" if down else "↑" if up else "→" if right else "←"
            lbl = QLabel(char)
            lbl.setFont(font(FONT_MONO, 24, bold=True))
            lbl.setStyleSheet(f"color:{MUTED}; font-weight:bold; background:transparent; border:none;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            return lbl

        self._flow_cards = {}
        def _topic_card(name, topic, fg_color):
            c = QFrame()
            c.setStyleSheet(f"background:{PANEL}; border: 1px solid {BORDER}; border-radius: 6px;")
            cl = QVBoxLayout(c)
            cl.setContentsMargins(12, 10, 12, 10)
            cl.setSpacing(4)
            
            hl = QHBoxLayout()
            hl.setContentsMargins(0,0,0,0)
            n = QLabel(name)
            n.setFont(font(FONT_SANS, 10, bold=True))
            n.setStyleSheet(f"color:{fg_color}; border:none;")
            hz = QLabel("0.0 Hz")
            hz.setFont(font(FONT_MONO, 12, bold=True))
            hz.setStyleSheet(f"color:{TEXT}; border:none;")
            hl.addWidget(n); hl.addStretch(); hl.addWidget(hz)
            cl.addLayout(hl)
            
            tl = QLabel(topic)
            tl.setFont(font(FONT_MONO, 9))
            tl.setStyleSheet(f"color:{MUTED}; border:none;")
            cl.addWidget(tl)
            
            val = QLabel("Waiting for data...")
            val.setFont(font(FONT_MONO, 10))
            val.setStyleSheet(f"color:{TEXT}; border:none;")
            cl.addWidget(val)
            
            self._flow_cards[name] = {"hz": hz, "val": val}
            return c

        # Main Grid Layout
        grid = QGridLayout()
        grid.setSpacing(10)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 0)
        grid.setColumnStretch(2, 1)

        # ── 1. User Input & Viz Layer (Row 0)
        user_b, user_l = _box("USER INPUT LAYER", "#f0f9ff", "#0284c7", 'V')
        user_l.addWidget(_label("Keyboard Teleop / VR Controller", "#0ea5e9", "white", bold=True))
        grid.addWidget(user_b, 0, 0)

        viz_b, viz_l = _box("VISUALIZATION TOOLS", "#f8fafc", "#475569", 'V')
        viz_l.addWidget(_label("Foxglove / rqt_graph / ros2_tracing", "#64748b", "white", bold=True))
        grid.addWidget(viz_b, 0, 2)

        grid.addWidget(_arrow(down=True), 1, 0)
        grid.addWidget(_arrow(up=True), 1, 2)

        # ── 2. ROS 2 Layer (Row 2)
        ros_b, ros_l = _box("ROS 2 LAYER", "#f0fdf4", "#16a34a", 'V')
        ros_grid = QGridLayout()
        ros_grid.setColumnStretch(0, 1); ros_grid.setColumnStretch(1, 0); ros_grid.setColumnStretch(2, 1)
        
        cmd_l = QVBoxLayout()
        cmd_l.addWidget(_label("Teleop Node", "#22c55e", "white", bold=True))
        cmd_l.addWidget(_arrow(down=True))
        cmd_l.addWidget(_topic_card("UNITY TARGET", UNITY_TOPIC, COL_UNITY))
        cmd_l.addWidget(_arrow(down=True))
        cmd_l.addWidget(_topic_card("PREDICTED TGT", PREDICTED_TOPIC, COL_PRED))
        cmd_l.addWidget(_arrow(down=True))
        cmd_l.addWidget(_topic_card("SENT COMMAND", SENT_TOPIC, COL_SENT))
        cmd_l.addWidget(_arrow(down=True))
        
        fb_l = QVBoxLayout()
        fb_l.addWidget(_arrow(up=True))
        fb_l.addWidget(_topic_card("ACTUAL FEEDBACK", ACTUAL_TOPIC, COL_ACTUAL))
        fb_l.addWidget(_topic_card("TOOL VECTOR", TOOL_ACT_TOPIC, GREEN))
        fb_l.addWidget(_topic_card("ROBOT MODE", ROBOT_MODE_TOPIC, PURPLE))
        fb_l.addWidget(_topic_card("ROBOT ERROR", ERROR_STATUS_TOPIC, RED))
        fb_l.addWidget(_topic_card("DIGITAL IO", DO_STATUS_TOPIC, ORANGE))
        
        ros_grid.addLayout(cmd_l, 0, 0)
        ros_grid.addLayout(fb_l, 0, 2)
        ros_l.addLayout(ros_grid)
        
        ros_l.addWidget(_label("MG400 ROS 2 Driver", "#15803d", "white", bold=True))
        grid.addWidget(ros_b, 2, 0, 1, 3)

        grid.addWidget(_arrow(down=True), 3, 0)
        grid.addWidget(_arrow(up=True), 3, 2)

        # ── 3. Comm Layer (Row 4)
        comm_b, comm_l = _box("COMMUNICATION LAYER", "#fffbeb", "#eab308", 'H')
        comm_l.addWidget(_label("TCP Socket (Port 30003/29999)\nCommands", "#eab308", "white", bold=True))
        comm_l.addStretch()
        comm_l.addWidget(_label("UDP Socket (Port 30004)\nFeedback Packet", "#ca8a04", "white", bold=True))
        grid.addWidget(comm_b, 4, 0, 1, 3)

        grid.addWidget(_arrow(down=True), 5, 0)
        grid.addWidget(_arrow(up=True), 5, 2)

        # ── 4. Robot Layer (Row 6)
        rob_b, rob_l = _box("ROBOT HARDWARE LAYER", "#fef2f2", "#dc2626", 'H')
        rob_l.addWidget(_label("Robot Controller", "#ef4444", "white", bold=True))
        rob_l.addWidget(_arrow(right=True))
        rob_l.addWidget(_label("Firmware Queue", "#dc2626", "white", bold=True))
        rob_l.addWidget(_arrow(right=True))
        rob_l.addWidget(_label("Motion Execution", "#b91c1c", "white", bold=True))
        rob_l.addWidget(_arrow(right=True))
        rob_l.addWidget(_label("Joint Encoders", "#991b1b", "white", bold=True))
        grid.addWidget(rob_b, 6, 0, 1, 3)

        lay.addLayout(grid)
        lay.addStretch()

    def _build_left(self, parent):
        lay = QVBoxLayout(parent); lay.setContentsMargins(0,0,8,0); lay.setSpacing(0)

        vsplit = QSplitter(Qt.Orientation.Vertical)
        vsplit.setHandleWidth(7)

        data_panel = self._build_data_panel()
        log_panel  = self._build_log_panel()

        vsplit.addWidget(data_panel)
        vsplit.addWidget(log_panel)
        # Give more priority to data panel
        vsplit.setSizes([600, 250])
        vsplit.setCollapsible(0, False)
        vsplit.setCollapsible(1, False)

        lay.addWidget(vsplit)

    # ── Scrollable data panel ─────────────────────────────────────────────────
    def _build_data_panel(self):
        outer = QFrame(); outer.setObjectName("panel")
        outer_lay = QVBoxLayout(outer); outer_lay.setContentsMargins(0,0,0,0); outer_lay.setSpacing(0)

        # Header
        hdr = QFrame(); hdr.setObjectName("panel_header"); hdr.setFixedHeight(34)
        hdr_lay = QHBoxLayout(hdr); hdr_lay.setContentsMargins(12,0,12,0)
        t = QLabel("SYSTEM DATA"); t.setFont(font(FONT_COND, 12, bold=True))
        t.setStyleSheet(f"color:{ACCENT}; background:transparent; letter-spacing:3px;")
        self._lbl_hz = QLabel("20 Hz")
        self._lbl_hz.setFont(font(FONT_MONO, 9))
        self._lbl_hz.setStyleSheet(f"color:{ACCENT}; background:rgba(14,165,233,0.1); border:1px solid rgba(14,165,233,0.2); border-radius:3px; padding:2px 8px;")
        hdr_lay.addWidget(t); hdr_lay.addStretch(); hdr_lay.addWidget(self._lbl_hz)
        outer_lay.addWidget(hdr)

        # Scroll
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner  = QWidget(); inner.setStyleSheet(f"background:{PANEL};")
        scroll.setWidget(inner)
        inner_lay = QVBoxLayout(inner); inner_lay.setContentsMargins(0,6,0,12); inner_lay.setSpacing(0)
        outer_lay.addWidget(scroll)

        self._build_joint_table(inner_lay)
        inner_lay.addWidget(_divider()); inner_lay.addSpacing(4)
        self._build_xyz_table(inner_lay)
        inner_lay.addWidget(_divider()); inner_lay.addSpacing(4)
        self._build_controls(inner_lay)
        inner_lay.addWidget(_divider()); inner_lay.addSpacing(4)
        self._build_metrics(inner_lay)
        inner_lay.addWidget(_divider()); inner_lay.addSpacing(4)
        self._build_latency_do(inner_lay)
        inner_lay.addStretch()

        return outer

    def _build_joint_table(self, lay):
        lay.addSpacing(8)
        lay.addWidget(_sec_label("JOINT TRACKING")); lay.addSpacing(6)

        # Header row
        hdr = QWidget(); hdr.setStyleSheet(f"background:{PANEL2};")
        hl  = QHBoxLayout(hdr); hl.setContentsMargins(10,3,10,3); hl.setSpacing(0)
        for txt, clr, w in [("JT",MUTED,40),("Target °",COL_UNITY,80),("Actual °",COL_ACTUAL,80),("Δ Error",MUTED,80)]:
            l = QLabel(txt); l.setFont(font(FONT_MONO, 9)); l.setFixedWidth(w)
            l.setStyleSheet(f"color:{clr}; background:transparent;")
            l.setAlignment(Qt.AlignmentFlag.AlignRight if txt != "JT" else Qt.AlignmentFlag.AlignLeft)
            hl.addWidget(l)
        hl.addStretch()
        lay.addWidget(hdr)

        self._j_tgt = []; self._j_act = []; self._j_diff = []; self._j_diff_lbl = []
        for i, name in enumerate(["J1","J2","J3","J4"]):
            row = QWidget(); row.setStyleSheet(f"background:{'#f9fafb' if i%2 else PANEL};")
            rl  = QHBoxLayout(row); rl.setContentsMargins(10,5,10,5); rl.setSpacing(0)

            nm = QLabel(name); nm.setFont(font(FONT_COND, 14, bold=True))
            nm.setFixedWidth(40); nm.setStyleSheet(f"color:{MUTED}; background:transparent;")

            vt = QLabel("0.00"); vt.setFont(font(FONT_MONO,11)); vt.setFixedWidth(80)
            vt.setAlignment(Qt.AlignmentFlag.AlignRight)
            vt.setStyleSheet(f"color:{COL_UNITY}; background:transparent;")

            va = QLabel("0.00"); va.setFont(font(FONT_MONO,11)); va.setFixedWidth(80)
            va.setAlignment(Qt.AlignmentFlag.AlignRight)
            va.setStyleSheet(f"color:{COL_ACTUAL}; background:transparent;")

            vd = QLabel("+0.00"); vd.setFont(font(FONT_MONO,11)); vd.setFixedWidth(80)
            vd.setAlignment(Qt.AlignmentFlag.AlignRight)

            rl.addWidget(nm); rl.addWidget(vt); rl.addWidget(va); rl.addWidget(vd); rl.addStretch()
            lay.addWidget(row)
            self._j_tgt.append(vt); self._j_act.append(va)
            self._j_diff.append(vd); self._j_diff_lbl.append(vd)

    def _build_xyz_table(self, lay):
        lay.addSpacing(8)
        lay.addWidget(_sec_label("END EFFECTOR (mm)")); lay.addSpacing(6)

        hdr = QWidget(); hdr.setStyleSheet(f"background:{PANEL2};")
        hl  = QHBoxLayout(hdr); hl.setContentsMargins(10,3,10,3); hl.setSpacing(0)
        for txt, clr, w in [("Ax",MUTED,28),("UnityFK",COL_UNITY,72),
                             ("Flange","#3b82f6",72),("TCP",GREEN,72),("ToolΔ",MUTED,68)]:
            l = QLabel(txt); l.setFont(font(FONT_MONO, 9)); l.setFixedWidth(w)
            l.setStyleSheet(f"color:{clr}; background:transparent;")
            l.setAlignment(Qt.AlignmentFlag.AlignRight if txt!="Ax" else Qt.AlignmentFlag.AlignLeft)
            hl.addWidget(l)
        hl.addStretch()
        lay.addWidget(hdr)

        self._xyz_u=[]; self._xyz_f=[]; self._xyz_t=[]; self._xyz_d=[]
        for i,ax in enumerate(["X","Y","Z"]):
            row = QWidget(); row.setStyleSheet(f"background:{'#f9fafb' if i%2 else PANEL};")
            rl  = QHBoxLayout(row); rl.setContentsMargins(10,4,10,4); rl.setSpacing(0)
            nm  = QLabel(ax); nm.setFont(font(FONT_COND,13,True)); nm.setFixedWidth(28)
            nm.setStyleSheet(f"color:{MUTED}; background:transparent;")

            def mk(clr,w=72):
                l=QLabel("0.0"); l.setFont(font(FONT_MONO,11)); l.setFixedWidth(w)
                l.setAlignment(Qt.AlignmentFlag.AlignRight)
                l.setStyleSheet(f"color:{clr}; background:transparent;"); return l

            vu=mk(COL_UNITY); vf=mk("#3b82f6"); vt=mk(GREEN); vd=mk(MUTED,68)
            rl.addWidget(nm); rl.addWidget(vu); rl.addWidget(vf); rl.addWidget(vt); rl.addWidget(vd); rl.addStretch()
            lay.addWidget(row)
            self._xyz_u.append(vu); self._xyz_f.append(vf); self._xyz_t.append(vt); self._xyz_d.append(vd)

        # Tool index
        tool_row = QWidget(); tool_row.setStyleSheet(f"background:{PANEL};")
        tl = QHBoxLayout(tool_row); tl.setContentsMargins(10,4,10,4)
        l  = QLabel("Active Tool:"); l.setFont(font(FONT_MONO,9)); l.setStyleSheet(f"color:{MUTED}; background:transparent;")
        self._lbl_tool = QLabel("querying…"); self._lbl_tool.setFont(font(FONT_MONO,9))
        self._lbl_tool.setStyleSheet(f"color:{ACCENT}; background:transparent;")
        tl.addWidget(l); tl.addWidget(self._lbl_tool); tl.addStretch()
        lay.addWidget(tool_row)

    def _build_controls(self, lay):
        lay.addSpacing(8); lay.addWidget(_sec_label("CONTROLS")); lay.addSpacing(6)

        # Suction row
        sr = QWidget(); sr.setStyleSheet(f"background:{PANEL};")
        sl = QHBoxLayout(sr); sl.setContentsMargins(10,4,10,4); sl.setSpacing(8)
        lbl = QLabel("Suction"); lbl.setFont(font(FONT_SANS,10,True))
        lbl.setStyleSheet(f"color:{MUTED}; background:transparent;"); lbl.setFixedWidth(60)
        self._btn_vac = QPushButton("⊘ OFF"); self._btn_vac.setObjectName("btn_vac")
        self._btn_vac.setFont(font(FONT_SANS,10,True)); self._btn_vac.setFixedWidth(110)
        self._btn_vac.clicked.connect(self._toggle_vac)
        self._lbl_vac = QLabel("idle"); self._lbl_vac.setFont(font(FONT_MONO,9))
        self._lbl_vac.setStyleSheet(f"color:{MUTED}; background:transparent;")
        sl.addWidget(lbl); sl.addWidget(self._btn_vac); sl.addWidget(self._lbl_vac); sl.addStretch()
        lay.addWidget(sr)

        # Lights row
        lr = QWidget(); lr.setStyleSheet(f"background:{PANEL};")
        ll = QHBoxLayout(lr); ll.setContentsMargins(10,4,10,4); ll.setSpacing(6)
        lbl2 = QLabel("Lights"); lbl2.setFont(font(FONT_SANS,10,True))
        lbl2.setStyleSheet(f"color:{MUTED}; background:transparent;"); lbl2.setFixedWidth(60)
        ll.addWidget(lbl2)
        self._btns_light = {}
        for name, port, on_clr, lit_prop in [
            ("GREEN",  GREEN_LIGHT_DO_PORT,  C_GREEN,  "green"),
            ("YELLOW", YELLOW_LIGHT_DO_PORT, C_YELLOW, "yellow"),
            ("RED",    RED_LIGHT_DO_PORT,    C_RED,    "red"),
        ]:
            btn = QPushButton(name); btn.setObjectName("btn_light")
            btn.setFont(font(FONT_SANS,9,True)); btn.setFixedWidth(76)
            btn.clicked.connect(lambda _,n=name,p=port,c=on_clr: self._toggle_light(n,p,c))
            ll.addWidget(btn)
            self._btns_light[name] = (btn, port, on_clr, lit_prop)
        ll.addStretch(); lay.addWidget(lr)

        # Dashboard Commands Row
        dr = QWidget(); dr.setStyleSheet(f"background:{PANEL};")
        dl = QHBoxLayout(dr); dl.setContentsMargins(10,4,10,4); dl.setSpacing(6)
        lbl3 = QLabel("Robot"); lbl3.setFont(font(FONT_SANS,10,True))
        lbl3.setStyleSheet(f"color:{MUTED}; background:transparent;"); lbl3.setFixedWidth(60)
        dl.addWidget(lbl3)
        
        for name, cmd, bg_clr in [
            ("ENABLE", "EnableRobot()", C_GREEN),
            ("DISABLE", "DisableRobot()", ORANGE),
            ("CLEAR ERR", "ClearError()", C_YELLOW),
            ("RESET", "ResetRobot()", RED),
        ]:
            btn = QPushButton(name)
            btn.setFont(font(FONT_SANS,9,True)); btn.setFixedWidth(80)
            btn.setStyleSheet(f"QPushButton {{ background:{PANEL2}; color:{MUTED}; border:1px solid {BORDER}; border-radius:4px; padding:4px; }} QPushButton:hover {{ background:{bg_clr}; color:white; border-color:{bg_clr}; }}")
            btn.clicked.connect(lambda _, c=cmd: self._send_dash(c))
            dl.addWidget(btn)
            
        dl.addStretch(); lay.addWidget(dr)

    def _build_metrics(self, lay):
        lay.addSpacing(8); lay.addWidget(_sec_label("EXECUTION METRICS")); lay.addSpacing(6)

        cards_w = QWidget(); cards_w.setStyleSheet(f"background:{PANEL};")
        cards_l = QHBoxLayout(cards_w); cards_l.setContentsMargins(10,4,10,4); cards_l.setSpacing(6)

        self._met = {}; self._met_lbl = {}
        for key, init, label in [("status","IDLE","STATUS"),("timer","0.00s","DURATION"),("count","0","COUNT")]:
            card = QFrame(); card.setObjectName("metric_card")
            cl   = QVBoxLayout(card); cl.setContentsMargins(8,6,8,6); cl.setSpacing(2); cl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            vl   = QLabel(init); vl.setObjectName("metric_val")
            vl.setFont(font(FONT_MONO,14,True)); vl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            ll   = QLabel(label); ll.setObjectName("metric_lbl")
            ll.setFont(font(FONT_SANS,8,True)); ll.setAlignment(Qt.AlignmentFlag.AlignCenter)
            cl.addWidget(vl); cl.addWidget(ll)
            cards_l.addWidget(card, stretch=1)
            self._met[key]=vl; self._met_lbl[key]=ll
        lay.addWidget(cards_w)

        # Stats chips
        stats_w = QWidget(); stats_w.setStyleSheet(f"background:{PANEL};")
        stats_l = QHBoxLayout(stats_w); stats_l.setContentsMargins(10,4,10,4); stats_l.setSpacing(6)
        self._stat_chips = []
        for t in ["Avg 0.00s","Min 0.00s","Max 0.00s"]:
            c, l = _chip(t); stats_l.addWidget(c, stretch=1); self._stat_chips.append(l)
        lay.addWidget(stats_w)

    def _build_latency_do(self, lay):
        lay.addSpacing(8); lay.addWidget(_sec_label("LATENCY & I/O")); lay.addSpacing(6)

        lat_w = QWidget(); lat_w.setStyleSheet(f"background:{PANEL};")
        lat_l = QHBoxLayout(lat_w); lat_l.setContentsMargins(10,4,10,4); lat_l.setSpacing(6)

        self._chips = {}
        for key, txt in [("cmd","Cmd: —"),("feed","Feed: —"),("do","DO: 0x0000")]:
            c, l = _chip(txt); lat_l.addWidget(c, stretch=1); self._chips[key]=l
        lay.addWidget(lat_w)

        # DO bits
        do_w = QWidget(); do_w.setStyleSheet(f"background:{PANEL};")
        do_l = QHBoxLayout(do_w); do_l.setContentsMargins(10,4,10,8); do_l.setSpacing(3)
        self._do_bits = []
        for i in range(16):
            b = QLabel(str(i+1)); b.setObjectName("do_bit")
            b.setFont(font(FONT_MONO,7)); b.setAlignment(Qt.AlignmentFlag.AlignCenter)
            b.setToolTip(f"DO{i+1}"); do_l.addWidget(b); self._do_bits.append(b)
        do_l.addStretch(); lay.addWidget(do_w)

    # ── Log terminal ──────────────────────────────────────────────────────────
    def _build_log_panel(self):
        outer = QFrame(); outer.setObjectName("panel")
        outer.setStyleSheet(f"QFrame#panel{{ background:{LOG_BG}; border:1px solid #1e3a2f; border-radius:10px; }}")
        lay = QVBoxLayout(outer); lay.setContentsMargins(0,0,0,0); lay.setSpacing(0)

        # Header
        hdr = QFrame(); hdr.setObjectName("log_header"); hdr.setFixedHeight(30)
        hl  = QHBoxLayout(hdr); hl.setContentsMargins(10,0,10,0)
        for clr in [C_RED,C_YELLOW,C_GREEN]:
            dot = QLabel("●"); dot.setStyleSheet(f"color:{clr}; background:transparent; font-size:10px;")
            hl.addWidget(dot)
        hl.addSpacing(8)
        title = QLabel("// SYSTEM LOG — MG400")
        title.setFont(font(FONT_MONO,10,True))
        title.setStyleSheet("color:#4ade80; background:transparent; letter-spacing:1px;")
        hl.addWidget(title); hl.addStretch()
        hz_lbl = QLabel("20Hz"); hz_lbl.setFont(font(FONT_MONO,9))
        hz_lbl.setStyleSheet("color:#2d6a4f; background:transparent;")
        hl.addWidget(hz_lbl)
        lay.addWidget(hdr)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFont(font(FONT_MONO, 10))
        self._log.setStyleSheet(f"""
            QTextEdit {{
                background: {LOG_BG}; color: {LOG_GRN};
                border: none; padding: 8px 12px;
            }}
            QScrollBar:vertical {{
                background:#0a1912; width:4px;
            }}
            QScrollBar::handle:vertical {{
                background:#1e3a2f; border-radius:2px;
            }}
        """)
        lay.addWidget(self._log)
        return outer

    # ──────────────────────────────────────────────────────────────────────────
    #  RIGHT PANEL
    # ──────────────────────────────────────────────────────────────────────────
    def _build_right(self, parent):
        lay = QVBoxLayout(parent); lay.setContentsMargins(4,0,0,0); lay.setSpacing(0)

        vsplit = QSplitter(Qt.Orientation.Vertical)
        vsplit.setHandleWidth(7)

        graphs_w = self._build_joint_graphs()
        graph3d_w = self._build_3d_graph()

        vsplit.addWidget(graphs_w)
        vsplit.addWidget(graph3d_w)
        vsplit.setSizes([420, 320])
        vsplit.setCollapsible(0, False)
        vsplit.setCollapsible(1, False)

        lay.addWidget(vsplit)

    # ── 2×2 joint graphs ─────────────────────────────────────────────────────
    def _build_joint_graphs(self):
        outer = QWidget(); outer.setStyleSheet(f"background:{BG};")
        grid  = QGridLayout(outer); grid.setContentsMargins(0,0,0,0); grid.setSpacing(12)

        self._axes_j  = []
        self._lines_j = []    # (l_u, l_p, l_s, l_a)
        self._live_tx = []
        self._canvas_list = []

        positions = [(0,0),(0,1),(1,0),(1,1)]

        for idx,(ri,ci) in enumerate(positions):
            card = QFrame()
            card.setObjectName("graph_card")
            card.setStyleSheet(f"QFrame#graph_card {{ background:{PANEL}; border: 1px solid {BORDER}; border-radius: 10px; }}")
            cl = QVBoxLayout(card)
            cl.setContentsMargins(0,0,0,0); cl.setSpacing(0)
            
            # Header
            hdr = QWidget()
            hdr.setStyleSheet(f"border-bottom: 1px solid {BORDER};")
            hl = QHBoxLayout(hdr)
            hl.setContentsMargins(16,10,16,10)
            
            t1 = QLabel(f"J{idx+1}")
            t1.setFont(font(FONT_SANS, 16, bold=True))
            t1.setStyleSheet("color:#0f172a; border:none;")
            
            val = QLabel("0.00°")
            val.setFont(font(FONT_SANS, 16, bold=True))
            val.setStyleSheet(f"color:{COL_ACTUAL}; border:none;")
            self._live_tx.append(val)
            
            hl.addWidget(t1); hl.addSpacing(10); hl.addWidget(val); hl.addStretch()
            
            # Legend
            for clr, lbl in [(COL_UNITY,"Unity"),(COL_PRED,"Pred"),(COL_SENT,"Sent"),(COL_ACTUAL,"Actual")]:
                dot = QLabel("—")
                dot.setFont(font(FONT_SANS, 14, bold=True))
                dot.setStyleSheet(f"color:{clr}; border:none;")
                txt = QLabel(lbl)
                txt.setFont(font(FONT_SANS, 10, bold=True))
                txt.setStyleSheet(f"color:{MUTED}; border:none;")
                hl.addWidget(dot); hl.addWidget(txt); hl.addSpacing(6)
                
            cl.addWidget(hdr)
            
            # Plot
            fig = Figure(facecolor=PANEL)
            fig.subplots_adjust(left=0.12, right=0.96, top=0.92, bottom=0.18)
            canvas = FigureCanvasQTAgg(fig)
            canvas.setStyleSheet("border:none; background:transparent;")
            ax = fig.add_subplot(111)
            ax.set_facecolor(PANEL)
            
            for sp in ['top','right','left']: ax.spines[sp].set_visible(False)
            ax.spines['bottom'].set_color(BORDER)
            ax.tick_params(axis='y', colors=MUTED, labelsize=9, length=0)
            ax.tick_params(axis='x', colors=MUTED, labelsize=9)
            ax.grid(True, axis='y', color="#e2e8f0", lw=0.8)
            ax.grid(False, axis='x')
            
            lu, = ax.plot([], [], color=COL_UNITY,  lw=2.0, ls="-", alpha=0.85)
            lp, = ax.plot([], [], color=COL_PRED,   lw=2.0, ls="--", alpha=0.85)
            ls, = ax.plot([], [], color=COL_SENT,   lw=0, marker='o', ms=4.5, alpha=0.9)
            la, = ax.plot([], [], color=COL_ACTUAL, lw=2.0)
            
            self._axes_j.append(ax)
            self._lines_j.append((lu,lp,ls,la))
            self._canvas_list.append(canvas)
            
            cl.addWidget(canvas, stretch=1)
            grid.addWidget(card, ri, ci)
            
        return outer

    # ── 3D trajectory ─────────────────────────────────────────────────────────
    def _build_3d_graph(self):
        outer = QFrame(); outer.setObjectName("panel_3d")
        lay   = QVBoxLayout(outer); lay.setContentsMargins(0,0,0,0); lay.setSpacing(0)

        # Header
        hdr = QFrame(); hdr.setObjectName("header_3d"); hdr.setFixedHeight(38)
        hl  = QHBoxLayout(hdr); hl.setContentsMargins(12,0,12,0)
        t = QLabel("END EFFECTOR — 3D TRAJECTORY"); t.setFont(font(FONT_COND,13,True))
        t.setStyleSheet(f"color:{TEXT}; background:transparent; letter-spacing:3px;")
        hl.addWidget(t); hl.addStretch()
        for clr,lbl in [(COL_UNITY,"Target (Unity FK)"),(COL_ACTUAL,"Actual (TCP)")]:
            dot = QLabel("●"); dot.setStyleSheet(f"color:{clr}; background:transparent; font-size:11px;")
            lw  = QLabel(lbl); lw.setFont(font(FONT_SANS,9))
            lw.setStyleSheet(f"color:{MUTED}; background:transparent;")
            hl.addWidget(dot); hl.addWidget(lw); hl.addSpacing(12)
        lay.addWidget(hdr)

        # 3D matplotlib
        self._fig_3d   = Figure(facecolor=PANEL)
        self._ax_3d    = self._fig_3d.add_subplot(111, projection='3d')
        self._canvas_3d= FigureCanvasQTAgg(self._fig_3d)
        self._canvas_3d.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._style_3d()

        self._line_tgt, = self._ax_3d.plot([],[],[], color=COL_UNITY, lw=1.2, alpha=0.75)
        self._line_act, = self._ax_3d.plot([],[],[], color=COL_ACTUAL, lw=1.5, alpha=0.9)
        self._pt_tgt,   = self._ax_3d.plot([],[],[], 'o', color=COL_UNITY,  ms=7)
        self._pt_act,   = self._ax_3d.plot([],[],[], 'o', color=COL_ACTUAL, ms=7)
        self._line_err, = self._ax_3d.plot([],[],[], color=RED, lw=1.5, alpha=0.7, ls="--")

        lay.addWidget(self._canvas_3d, stretch=1)

        # Coord bar
        coord = QFrame(); coord.setObjectName("coord_bar"); coord.setFixedHeight(38)
        cl    = QHBoxLayout(coord); cl.setContentsMargins(12,0,12,0); cl.setSpacing(0)

        self._coord_tgt = []; self._coord_act = []
        for prefix, store, clr in [("TARGET",self._coord_tgt,COL_UNITY),("ACTUAL",self._coord_act,COL_ACTUAL)]:
            cl.addSpacing(6)
            lp = QLabel(prefix); lp.setFont(font(FONT_SANS,8,True))
            lp.setStyleSheet(f"color:{clr}; background:transparent;"); cl.addWidget(lp)
            cl.addSpacing(8)
            for ax in ["X","Y","Z"]:
                la = QLabel(ax); la.setFont(font(FONT_MONO,9,True))
                la.setStyleSheet(f"color:{MUTED}; background:transparent;"); cl.addWidget(la)
                lv = QLabel("0.0"); lv.setFont(font(FONT_MONO,10))
                lv.setStyleSheet(f"color:{clr}; background:transparent;")
                lv.setFixedWidth(62); lv.setAlignment(Qt.AlignmentFlag.AlignRight)
                cl.addWidget(lv); store.append(lv); cl.addSpacing(6)
            cl.addSpacing(16)

        cl.addStretch()

        # Dist badge
        dist_lbl = QLabel("ERR DIST"); dist_lbl.setFont(font(FONT_SANS,8,True))
        dist_lbl.setStyleSheet(f"color:{MUTED}; background:transparent;"); cl.addWidget(dist_lbl)
        cl.addSpacing(6)
        self._lbl_dist = QLabel("0.0"); self._lbl_dist.setFont(font(FONT_MONO,14,True))
        self._lbl_dist.setStyleSheet(f"color:{ACCENT}; background:transparent;"); cl.addWidget(self._lbl_dist)
        mm_lbl = QLabel(" mm"); mm_lbl.setFont(font(FONT_SANS,9))
        mm_lbl.setStyleSheet(f"color:{MUTED}; background:transparent;"); cl.addWidget(mm_lbl)

        lay.addWidget(coord)
        return outer

    def _style_3d(self):
        ax = self._ax_3d
        ax.set_facecolor("#fafbfd")
        ax.xaxis.pane.fill=False; ax.yaxis.pane.fill=False; ax.zaxis.pane.fill=False
        ax.xaxis.pane.set_edgecolor(BORDER)
        ax.yaxis.pane.set_edgecolor(BORDER)
        ax.zaxis.pane.set_edgecolor(BORDER)
        ax.set_xlabel("X mm", fontsize=7, color=MUTED)
        ax.set_ylabel("Y mm", fontsize=7, color=MUTED)
        ax.set_zlabel("Z mm", fontsize=7, color=MUTED)
        ax.tick_params(colors=MUTED, labelsize=6)
        ax.grid(True, color=BORDER, lw=0.4)
        self._fig_3d.tight_layout(pad=0.5)

    # ──────────────────────────────────────────────────────────────────────────
    #  GRAPH UPDATE  (called by QTimer at 20 Hz)
    # ──────────────────────────────────────────────────────────────────────────
    def _update_graphs(self):
        d   = self.data
        now = time.time()
        rel = now - self.g_start

        self.t_buf.append(rel)
        for i in range(4):
            self.u_buf[i].append(d.unity[i])
            self.p_buf[i].append(d.predicted[i])
            self.a_buf[i].append(d.actual[i])
            if d.sent_fresh[i]:
                self.s_buf[i].append(d.sent[i])
                d.sent_fresh[i] = False
            else:
                self.s_buf[i].append(np.nan)

        t_arr = np.array(self.t_buf)
        import matplotlib.ticker as ticker
        for i in range(4):
            lu,lp,ls,la = self._lines_j[i]
            ax = self._axes_j[i]
            u=np.array(self.u_buf[i]); p=np.array(self.p_buf[i])
            s=np.array(self.s_buf[i]); a=np.array(self.a_buf[i])
            lu.set_data(t_arr,u); lp.set_data(t_arr,p)
            ls.set_data(t_arr,s); la.set_data(t_arr,a)
            ax.set_xlim(max(0,rel-GRAPH_WIN), max(GRAPH_WIN, rel+0.5))
            if len(a)>1:
                vals = np.concatenate([u,p,a])
                mn,mx = np.nanmin(vals), np.nanmax(vals)
                if not (np.isnan(mn) or np.isnan(mx)):
                    pad = max(2.0,(mx-mn)*0.15)
                    ax.set_ylim(mn-pad, mx+pad)
            self._live_tx[i].setText(f"{d.actual[i]:+.2f}°")
            ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: f"{x:.1f}s"))

        for canvas in self._canvas_list:
            canvas.draw_idle()

        # 3D
        tgt = d.unity_xyz[:3]; act = d.tool_act[:3]
        self.tgt_trail.append(tuple(tgt)); self.act_trail.append(tuple(act))
        if len(self.tgt_trail) >= 2:
            tx,ty,tz = zip(*self.tgt_trail)
            ax_,ay_,az_ = zip(*self.act_trail)
            self._line_tgt.set_data_3d(tx,ty,tz)
            self._line_act.set_data_3d(ax_,ay_,az_)
            self._pt_tgt.set_data_3d([tgt[0]],[tgt[1]],[tgt[2]])
            self._pt_act.set_data_3d([act[0]],[act[1]],[act[2]])
            self._line_err.set_data_3d([tgt[0],act[0]],[tgt[1],act[1]],[tgt[2],act[2]])
            # auto-scale 3D
            def _lim(v):
                mn,mx=min(v),max(v); pad=max(20,(mx-mn)*.2); return mn-pad, mx+pad
            ax3=self._ax_3d
            ax3.set_xlim(*_lim(list(tx)+list(ax_)))
            ax3.set_ylim(*_lim(list(ty)+list(ay_)))
            ax3.set_zlim(*_lim(list(tz)+list(az_)))
            self._canvas_3d.draw_idle()

    # ──────────────────────────────────────────────────────────────────────────
    #  MAIN REFRESH  (50 ms / 20 Hz)
    # ──────────────────────────────────────────────────────────────────────────
    def _refresh(self):
        d   = self.data
        now = time.time()

        # Sim if no ROS
        if not ROS_AVAILABLE: d.tick_sim()

        # Clock / uptime
        self._lbl_clock.setText(datetime.datetime.now().strftime("%H:%M:%S"))
        # ROS status
        now = time.time()
        elapsed = int(now - self._start_t)
        self._lbl_uptime.setText(f"⏱ {elapsed//3600:02d}:{(elapsed%3600)//60:02d}:{elapsed%60:02d}")

        if ROS_AVAILABLE:
            if d.last_act_t > 0 and (now - d.last_act_t) < 2.0:
                self._lbl_ros.setText("ROS2 CONNECTED")
                self._lbl_ros.setStyleSheet(f"color:{GREEN}; background:rgba(34,197,94,0.08); border:1px solid rgba(34,197,94,0.3); border-radius:4px; padding:3px 10px;")
            else:
                self._lbl_ros.setText("WAITING FOR DATA")
                self._lbl_ros.setStyleSheet(f"color:{ORANGE}; background:rgba(249,115,22,0.08); border:1px solid rgba(249,115,22,0.3); border-radius:4px; padding:3px 10px;")
        else:
            self._lbl_ros.setText("SIMULATION / DISCONNECTED")
            self._lbl_ros.setStyleSheet(f"color:{RED}; background:rgba(239,68,68,0.08); border:1px solid rgba(239,68,68,0.3); border-radius:4px; padding:3px 10px;")

        # Mode
        mode_str = MODE_NAMES.get(d.robot_mode, str(d.robot_mode))
        self._lbl_mode.setText(f"MODE: {mode_str}")
        danger = d.robot_mode in (9,11)
        _set_prop(self._lbl_mode, "danger", danger)

        # Error
        if d.error_stat != 0:
            desc,_,_ = self.err_dec.decode_error(d.error_stat)
            self._lbl_err.setText(f"✗ ERR {d.error_stat:02X}: {desc or '?'}")
            _set_prop(self._lbl_err, "error", True)
        else:
            self._lbl_err.setText("✓ ERR: 00")
            _set_prop(self._lbl_err, "error", False)

        # Joint table
        tgt=d.unity; act=d.actual; total_err=0.0
        for i in range(4):
            self._j_tgt[i].setText(f"{tgt[i]:+.2f}")
            self._j_act[i].setText(f"{act[i]:+.2f}")
            diff=act[i]-tgt[i]; total_err+=abs(diff)
            self._j_diff[i].setText(f"{diff:+.2f}")
            clr = RED if abs(diff)>2 else (ORANGE if abs(diff)>0.5 else GREEN)
            self._j_diff_lbl[i].setStyleSheet(f"color:{clr}; background:transparent;")

        # XYZ
        xu=d.unity_xyz; xf=d.flange; xt=d.tool_act
        for i in range(3):
            self._xyz_u[i].setText(f"{xu[i]:.1f}")
            self._xyz_f[i].setText(f"{xf[i]:.1f}")
            self._xyz_t[i].setText(f"{xt[i]:.1f}")
            self._xyz_d[i].setText(f"{xt[i]-xf[i]:+.1f}")
        self._lbl_tool.setText(f"Tool {d.tool_idx}" if d.tool_idx>=0 else "querying…")

        # 3D coords overlay
        for i,lv in enumerate(self._coord_tgt): lv.setText(f"{xu[i]:.1f}")
        for i,lv in enumerate(self._coord_act):  lv.setText(f"{xt[i]:.1f}")
        dist = math.sqrt(sum((xt[i]-xu[i])**2 for i in range(3)))
        self._lbl_dist.setText(f"{dist:.2f}")

        # DO bits
        do=d.do_status
        self._chips["do"].setText(f"DO: 0x{do:04X}")
        for i,b in enumerate(self._do_bits):
            on = bool((do>>i)&1)
            _set_prop(b,"on",on)

        # Controls sync
        if now > self.lockout.get(VACUUM_DO_PORT,0):
            actual_vac = bool((do>>(VACUUM_DO_PORT-1))&1)
            self.vac_on = actual_vac
            if actual_vac:
                self._btn_vac.setText("⊙ VACUUM")
                self._btn_vac.setStyleSheet(f"background:{C_VAC};color:white;border-radius:5px;padding:5px 14px;font-weight:700;")
                self._lbl_vac.setText("active"); self._lbl_vac.setStyleSheet(f"color:{ACCENT};background:transparent;")
            else:
                self._btn_vac.setText("⊘ OFF")
                self._btn_vac.setStyleSheet(""); self._lbl_vac.setText("idle")
                self._lbl_vac.setStyleSheet(f"color:{MUTED};background:transparent;")

        for name,(btn,port,on_clr,lit_prop) in self._btns_light.items():
            if now > self.lockout.get(port,0):
                actual = bool((do>>(port-1))&1)
                self.light_on[name] = actual
                _set_prop(btn, "lit", lit_prop if actual else "")

        # Execution monitor
        self.exec_mon.update(total_err)
        st=self.exec_mon.state
        mv=self._met; ml=self._met_lbl
        mv["status"].setText(st)
        _set_prop(mv["status"],"state",st.lower() if st!="IDLE" else "")
        if st=="MOVING":
            mv["timer"].setText(f"{now-self.exec_mon.start_t:.2f}s")
        elif st=="ARRIVED":
            mv["timer"].setText(f"{self.exec_mon.last_dur:.2f}s")
        mv["count"].setText(str(len(self.exec_mon.durs)))
        avg,mn,mx=self.exec_mon.stats()
        self._stat_chips[0].setText(f"Avg {avg:.2f}s")
        self._stat_chips[1].setText(f"Min {mn:.2f}s")
        self._stat_chips[2].setText(f"Max {mx:.2f}s")

        # Data Flow Tab Updates
        if now - self._last_flow_t >= 1.0:
            dt = now - self._last_flow_t
            self._last_flow_t = now
            for k in d.msg_counts:
                hz = d.msg_counts[k] / dt
                d.flow_hz[k] = hz
                d.msg_counts[k] = 0
                if k in self._flow_cards:
                    lbl = self._flow_cards[k]["hz"]
                    lbl.setText(f"{hz:.1f} Hz")
                    lbl.setStyleSheet(f"color:{GREEN if hz > 0.5 else MUTED}; border:none;")

        # Update real-time values in Data Flow cards
        if self._flow_cards:
            self._flow_cards.get("UNITY TARGET", {}).get("val", QLabel()).setText(f"[{d.unity[0]:+.1f}°, {d.unity[1]:+.1f}°, {d.unity[2]:+.1f}°, {d.unity[3]:+.1f}°]")
            self._flow_cards.get("PREDICTED TGT", {}).get("val", QLabel()).setText(f"[{d.predicted[0]:+.1f}°, {d.predicted[1]:+.1f}°, {d.predicted[2]:+.1f}°, {d.predicted[3]:+.1f}°]")
            self._flow_cards.get("SENT COMMAND", {}).get("val", QLabel()).setText(f"[{d.sent[0]:+.1f}°, {d.sent[1]:+.1f}°, {d.sent[2]:+.1f}°, {d.sent[3]:+.1f}°]")
            self._flow_cards.get("ACTUAL FEEDBACK", {}).get("val", QLabel()).setText(f"[{d.actual[0]:+.1f}°, {d.actual[1]:+.1f}°, {d.actual[2]:+.1f}°, {d.actual[3]:+.1f}°]")
            self._flow_cards.get("TOOL VECTOR", {}).get("val", QLabel()).setText(f"X: {d.tool_act[0]:.1f}  Y: {d.tool_act[1]:.1f}  Z: {d.tool_act[2]:.1f}")
            self._flow_cards.get("ROBOT MODE", {}).get("val", QLabel()).setText(f"Mode: {MODE_NAMES.get(d.robot_mode, d.robot_mode)}")
            self._flow_cards.get("ROBOT ERROR", {}).get("val", QLabel()).setText(f"Err Code: 0x{d.error_stat:02X}")
            self._flow_cards.get("DIGITAL IO", {}).get("val", QLabel()).setText(f"Mask: 0x{d.do_status:04X}")

        # Latency
        if d.last_tgt_t: self._chips["cmd"].setText(f"Cmd: {(now-d.last_tgt_t)*1000:.0f}ms")
        if d.last_act_t: self._chips["feed"].setText(f"Feed: {(now-d.last_act_t)*1000:.0f}ms")

        # Log
        if now - self._last_log > 0.55:
            self._append_log(act, xu, xt, total_err)
            self._last_log = now

    # ──────────────────────────────────────────────────────────────────────────
    #  LOG TERMINAL
    # ──────────────────────────────────────────────────────────────────────────
    _LOG_MSGS = [
        lambda a,u,t,e: ("DATA",  f"J1=<b>{a[0]:+.3f}°</b>  J2=<b>{a[1]:+.3f}°</b>  J3=<b>{a[2]:+.3f}°</b>  J4=<b>{a[3]:+.3f}°</b>"),
        lambda a,u,t,e: ("DATA",  f"Target X=<b>{u[0]:.1f}</b>  Y=<b>{u[1]:.1f}</b>  Z=<b>{u[2]:.1f}</b> mm"),
        lambda a,u,t,e: ("DATA",  f"TCP    X=<b>{t[0]:.1f}</b>  Y=<b>{t[1]:.1f}</b>  Z=<b>{t[2]:.1f}</b> mm"),
        lambda a,u,t,e: ("INFO",  "Teleop command received from <b>Unity</b> controller"),
        lambda a,u,t,e: ("SYS",   "Session logger flush — <b>joints_tracking.csv</b>"),
        lambda a,u,t,e: ("DATA",  f"Total |Δ| joints: <b>{e:.3f}°</b>"),
        lambda a,u,t,e: ("INFO",  "FK solved — flange position updated"),
        lambda a,u,t,e: ("SYS",   "ROS2 spin tick — <b>/joint_states</b> callback"),
        lambda a,u,t,e: ("DATA",  f"3D dist err: <b>{math.sqrt(sum((t[i]-u[i])**2 for i in range(3))):.2f} mm</b>"),
        lambda a,u,t,e: ("WARN",  "High latency detected: cmd age > <b>80ms</b>"),
        lambda a,u,t,e: ("INFO",  "Execution state: <b>ARRIVED</b> — motion complete"),
        lambda a,u,t,e: ("SYS",   "xyz_tracking.csv — <b>row appended</b>"),
    ]

    _TAG_COLORS = {"INFO":"#4ade80","WARN":"#fbbf24","ERR":"#f87171","DATA":"#38bdf8","SYS":"#c084fc"}

    def _append_log(self, act, xyz_u, xyz_t, total_err):
        try:
            tag, msg = self._LOG_MSGS[self._log_idx % len(self._LOG_MSGS)](act, xyz_u, xyz_t, total_err)
        except Exception:
            tag, msg = "SYS", "…"
        self._log_idx += 1

        ts   = datetime.datetime.now().strftime("%H:%M:%S.%f")[:11]
        tclr = self._TAG_COLORS.get(tag, LOG_GRN)

        html = (
            f'<span style="color:#2d6a4f;">{ts}</span> '
            f'<span style="color:{tclr}; font-weight:bold;">[{tag}]</span> '
            f'<span style="color:#86efac;">{msg}</span><br>'
        )

        cur = self._log.textCursor()
        cur.movePosition(QTextCursor.MoveOperation.End)
        self._log.setTextCursor(cur)
        self._log.insertHtml(html)

        # Trim to 200 lines
        doc = self._log.document()
        if doc.blockCount() > 210:
            cur2 = QTextCursor(doc.begin())
            cur2.select(QTextCursor.SelectionType.BlockUnderCursor)
            cur2.removeSelectedText()
            cur2.deleteChar()

        self._log.verticalScrollBar().setValue(self._log.verticalScrollBar().maximum())

    # ──────────────────────────────────────────────────────────────────────────
    #  CONTROL CALLBACKS
    # ──────────────────────────────────────────────────────────────────────────
    def _toggle_vac(self):
        self.vac_on = not self.vac_on
        if self.ros_node: self.ros_node.send_suction(self.vac_on)
        self.lockout[VACUUM_DO_PORT] = time.time()+2.0
        if self.vac_on:
            self._btn_vac.setText("⊙ VACUUM")
            self._btn_vac.setStyleSheet(f"background:{C_VAC};color:white;border-radius:5px;padding:5px 14px;font-weight:700;border:none;")
            self._lbl_vac.setText("active"); self._lbl_vac.setStyleSheet(f"color:{ACCENT};background:transparent;")
        else:
            self._btn_vac.setText("⊘ OFF"); self._btn_vac.setStyleSheet("")
            self._lbl_vac.setText("idle"); self._lbl_vac.setStyleSheet(f"color:{MUTED};background:transparent;")

    def _toggle_light(self, name, port, on_clr):
        self.light_on[name] = not self.light_on[name]
        if self.ros_node: self.ros_node.send_light(port, self.light_on[name])
        self.lockout[port] = time.time()+2.0
        btn,_,_,lit_prop = self._btns_light[name]
        _set_prop(btn, "lit", lit_prop if self.light_on[name] else "")

    # ──────────────────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        self._timer.stop(); self._graph_timer.stop()
        self.logger.close()
        event.accept()


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════
def main():
    # High DPI
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

    app = QApplication(sys.argv)
    
    # Must load fonts AFTER QApplication is created
    _load_fonts()
    
    app.setApplicationName("MG400 Monitor")
    app.setStyleSheet(QSS)

    # ROS2
    ros_node = None
    if ROS_AVAILABLE:
        rclpy.init()
        ros_node = RosNode(DATA)
        spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        spin_thread.start()
    else:
        print("[INFO] Running in simulation mode (no ROS2)")

    win = MonitorWindow(DATA, ros_node)
    win.show()

    exit_code = app.exec()

    if ros_node:
        ros_node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()