import sys
from PyQt6.QtWidgets import QApplication, QWidget, QLabel
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QPainter, QPainterPath, QPen, QColor, QFont

class FlowCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.nodes = {}
        self.edges = []
        self.anim_offset = 0.0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._animate)
        self.timer.start(30)
        self.frequencies = {}
        self.setStyleSheet("background: #0f172a;") # Dark bg for testing

    def _animate(self):
        self.anim_offset -= 1.5
        self.update()

    def add_node(self, id_str, widget, rel_x, rel_y, w, h):
        widget.setParent(self)
        self.nodes[id_str] = {
            "widget": widget, "rx": rel_x, "ry": rel_y, "w": w, "h": h
        }

    def add_edge(self, p1_id, p2_id, color="#ffffff", key=None):
        self.edges.append((p1_id, p2_id, color, key))

    def set_freq(self, key, hz):
        self.frequencies[key] = hz

    def resizeEvent(self, e):
        super().resizeEvent(e)
        w, h = self.width(), self.height()
        for nid, n in self.nodes.items():
            nx = int(n["rx"] * w - n["w"]/2)
            ny = int(n["ry"] * h - n["h"]/2)
            n["widget"].setGeometry(nx, ny, n["w"], n["h"])

    def paintEvent(self, e):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        
        for p1_id, p2_id, color, key in self.edges:
            if p1_id not in self.nodes or p2_id not in self.nodes:
                continue
            n1, n2 = self.nodes[p1_id], self.nodes[p2_id]
            x1, y1 = n1["rx"] * w, n1["ry"] * h
            x2, y2 = n2["rx"] * w, n2["ry"] * h
            
            # Simple bounding box edge routing:
            # We want the line to connect the closest edges, but center-to-center works for now 
            # if we just draw it beneath the widgets (which paintEvent naturally does since widgets are children).
            path = QPainterPath()
            path.moveTo(x1, y1)
            
            dx, dy = x2 - x1, y2 - y1
            if abs(dx) > abs(dy):
                c1 = QPointF(x1 + dx/2, y1)
                c2 = QPointF(x2 - dx/2, y2)
            else:
                c1 = QPointF(x1, y1 + dy/2)
                c2 = QPointF(x2, y2 - dy/2)
            
            path.cubicTo(c1, c2, QPointF(x2, y2))
            
            c_base = QColor(color)
            c_base.setAlpha(40)
            pen_base = QPen(c_base)
            pen_base.setWidth(2)
            painter.setPen(pen_base)
            painter.drawPath(path)
            
            freq = self.frequencies.get(key, 10.0) # default positive for test
            if freq > 0:
                speed = min(freq / 5.0, 4.0) + 0.5
                c_anim = QColor(color)
                pen_anim = QPen(c_anim)
                pen_anim.setWidth(3)
                pen_anim.setStyle(Qt.PenStyle.DashLine)
                pen_anim.setDashPattern([6, 12])
                pen_anim.setDashOffset(self.anim_offset * speed)
                painter.setPen(pen_anim)
                painter.drawPath(path)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = FlowCanvas()
    
    l1 = QLabel("Node A")
    l1.setStyleSheet("background: #1e293b; color: white; border: 2px solid #3b82f6; border-radius: 8px;")
    l1.setAlignment(Qt.AlignmentFlag.AlignCenter)
    
    l2 = QLabel("Node B")
    l2.setStyleSheet("background: #1e293b; color: white; border: 2px solid #10b981; border-radius: 8px;")
    l2.setAlignment(Qt.AlignmentFlag.AlignCenter)
    
    win.add_node("A", l1, 0.2, 0.5, 100, 50)
    win.add_node("B", l2, 0.8, 0.5, 100, 50)
    win.add_edge("A", "B", "#3b82f6", "test_flow")
    
    win.set_freq("test_flow", 20.0)
    
    win.resize(800, 600)
    win.show()
    # sys.exit(app.exec())
