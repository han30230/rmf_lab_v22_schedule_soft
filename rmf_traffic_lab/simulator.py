#!/usr/bin/env python3
"""Interactive PySide6 editor and viewer for the real RMF Traffic lab runner."""

from __future__ import annotations

import copy
from datetime import datetime
import json
import math
import os
import sys
from pathlib import Path

try:
    from PySide6.QtCore import QPointF, QRectF, QProcess, QSettings, QTimer, Qt
    from PySide6.QtGui import (
        QAction, QColor, QBrush, QFont, QFontDatabase, QKeySequence, QPainterPath,
        QPen, QPolygonF, QTextCursor,
    )
    from PySide6.QtWidgets import (
        QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
        QFormLayout, QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem,
        QGraphicsItemGroup, QGraphicsPathItem, QGraphicsPolygonItem,
        QGraphicsRectItem, QGraphicsScene, QGraphicsSimpleTextItem, QGraphicsView, QGroupBox,
        QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow, QMessageBox,
        QInputDialog,
        QPushButton, QScrollArea, QSlider, QSpinBox, QSplitter, QTableWidget,
        QTableWidgetItem, QTabWidget, QTextEdit, QToolBar, QVBoxLayout, QWidget,
        QSizePolicy,
    )
except ImportError as exc:  # pragma: no cover - exercised on machines without Qt
    print("PySide6가 필요합니다: python3 -m pip install -r requirements-gui.txt", file=sys.stderr)
    raise SystemExit(2) from exc

from tools.scenario_templates import builtin_scenarios
from tools.building_map_import import available_levels, convert_building_map_yaml
from tools.setup_after_core import (
    after_core_patch_status,
    prepare_after_core as install_after_lane_penalty_core,
)
from tools.setup_after_nego_core import prepare_after_nego_core
from tools.setup_schedule_soft_core import prepare_schedule_soft_core
from tools.event_explainer import (
    astar_guide_text, classify_negotiation_message, decision_records,
    diagnosis_text, explain_event, explain_runtime_output,
    failure_summary_text, failure_trace_records,
    rmf_object_guide_text, schedule_guide_text, schedule_model_text,
    summarize_jsonl,
)


ROOT = Path(__file__).resolve().parent
SCALE = 70.0
ROBOT_COLORS = ["#ef4444", "#22c55e", "#38bdf8", "#f59e0b", "#a78bfa", "#ec4899"]

APP_STYLE = """
QMainWindow, QWidget {
  background-color: #f4f7fb;
  color: #172033;
}
QToolBar#mainToolbar {
  background-color: #ffffff;
  border: 0;
  border-bottom: 1px solid #d8e0eb;
  spacing: 7px;
  padding: 7px;
}
QLabel#toolbarSection, QLabel#graphBadge, QLabel#coreBadge {
  color: #1e40af;
  font-weight: 700;
}
QLabel#graphBadge, QLabel#coreBadge {
  background-color: #eff6ff;
  border: 1px solid #93c5fd;
  border-radius: 6px;
  padding: 5px 9px;
}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit, QTableWidget {
  background-color: #ffffff;
  color: #172033;
  border: 1px solid #cbd5e1;
  border-radius: 5px;
  selection-background-color: #2563eb;
  selection-color: #ffffff;
}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
  min-height: 27px;
  padding: 2px 6px;
}
QPushButton {
  background-color: #ffffff;
  color: #1e293b;
  border: 1px solid #94a3b8;
  border-radius: 6px;
  padding: 6px 11px;
}
QPushButton:hover { background-color: #eff6ff; border-color: #3b82f6; }
QPushButton:pressed { background-color: #dbeafe; }
QPushButton:disabled { color: #94a3b8; background-color: #f1f5f9; border-color: #cbd5e1; }
QPushButton#primaryRun {
  background-color: #2563eb;
  color: #ffffff;
  border: 2px solid #1d4ed8;
  font-weight: 700;
  padding: 9px 17px;
}
QPushButton#primaryRun:hover { background-color: #1d4ed8; }
QPushButton#dangerButton { background-color: #fee2e2; color: #991b1b; border-color: #f87171; font-weight: 700; }
QPushButton#accentButton { background-color: #dcfce7; color: #166534; border-color: #4ade80; font-weight: 700; }
QTabWidget::pane { border: 1px solid #cbd5e1; background-color: #ffffff; }
QTabBar::tab {
  background-color: #eef2f7;
  color: #475569;
  border: 1px solid #cbd5e1;
  padding: 7px 11px;
}
QTabBar::tab:selected { background-color: #1d4ed8; color: #ffffff; font-weight: 700; }
QHeaderView::section {
  background-color: #e8eef7;
  color: #1e3a5f;
  border: 0;
  border-right: 1px solid #cbd5e1;
  border-bottom: 1px solid #94a3b8;
  padding: 6px;
  font-weight: 700;
}
QTableWidget { alternate-background-color: #f8fafc; gridline-color: #dbe3ed; }
QLabel#liveDecision {
  background-color: #eff6ff;
  color: #172554;
  border: 2px solid #2563eb;
  border-radius: 8px;
  padding: 9px 12px;
  font-weight: 600;
}
QLabel#sectionTitle { color: #1d4ed8; font-weight: 700; padding: 3px; }
QSplitter::handle { background-color: #d8e0eb; }
QSplitter::handle:hover { background-color: #93c5fd; }
QSplitter#mapOutputSplitter::handle:vertical {
  background-color: #bfdbfe;
  border-top: 2px solid #60a5fa;
  border-bottom: 2px solid #60a5fa;
}
QSplitter#mapOutputSplitter::handle:vertical:hover { background-color: #60a5fa; }
QStatusBar { background-color: #ffffff; color: #475569; border-top: 1px solid #d8e0eb; }
QCheckBox { spacing: 6px; }
QSlider::groove:horizontal { height: 6px; background: #cbd5e1; border-radius: 3px; }
QSlider::handle:horizontal { width: 16px; margin: -5px 0; background: #2563eb; border-radius: 8px; }
"""


class CopyableTableWidget(QTableWidget):
    """Table with predictable TSV copy for analysis data."""

    def _range_text(self, all_rows: bool) -> str:
        if all_rows:
            top, bottom = 0, self.rowCount() - 1
            left, right = 0, self.columnCount() - 1
        else:
            ranges = self.selectedRanges()
            if not ranges:
                return ""
            top = min(selection.topRow() for selection in ranges)
            bottom = max(selection.bottomRow() for selection in ranges)
            left = min(selection.leftColumn() for selection in ranges)
            right = max(selection.rightColumn() for selection in ranges)

        rows: list[str] = []
        if all_rows:
            rows.append("\t".join(
                self.horizontalHeaderItem(column).text()
                if self.horizontalHeaderItem(column) else ""
                for column in range(left, right + 1)
            ))
        for row in range(top, bottom + 1):
            rows.append("\t".join(
                self.item(row, column).text() if self.item(row, column) else ""
                for column in range(left, right + 1)
            ))
        return "\n".join(rows)

    def copy_selection(self) -> bool:
        text = self._range_text(False)
        if not text:
            return False
        QApplication.clipboard().setText(text)
        return True

    def copy_all(self) -> bool:
        text = self._range_text(True)
        if not text:
            return False
        QApplication.clipboard().setText(text)
        return True

    def keyPressEvent(self, event) -> None:
        modifiers = event.modifiers()
        if event.key() == Qt.Key.Key_C and (
            modifiers & Qt.KeyboardModifier.ControlModifier
            and modifiers & Qt.KeyboardModifier.ShiftModifier
        ):
            self.copy_all()
            event.accept()
            return
        if event.matches(QKeySequence.StandardKey.Copy):
            self.copy_selection()
            event.accept()
            return
        super().keyPressEvent(event)


class RobotItem(QGraphicsItemGroup):
    """A differential-drive robot marker with a visible +X heading nose."""

    def __init__(self, name: str, color: str):
        super().__init__()
        accent = QColor(color)
        dark = QColor("#111827")
        light = QColor("#f8fafc")

        shadow = QGraphicsEllipseItem(-17, -10, 38, 25)
        shadow_color = QColor("#64748b"); shadow_color.setAlpha(85)
        shadow.setBrush(QBrush(shadow_color)); shadow.setPen(QPen(Qt.PenStyle.NoPen))
        shadow.setPos(2, 4); self.addToGroup(shadow)

        body_path = QPainterPath()
        body_path.addRoundedRect(QRectF(-18, -13, 36, 26), 7, 7)
        body = QGraphicsPathItem(body_path)
        body.setBrush(QBrush(accent)); body.setPen(QPen(light, 2))
        self.addToGroup(body)

        cabin = QGraphicsRectItem(-7, -8, 15, 16)
        cabin_color = QColor(dark); cabin_color.setAlpha(185)
        cabin.setBrush(QBrush(cabin_color)); cabin.setPen(QPen(Qt.PenStyle.NoPen))
        self.addToGroup(cabin)

        for x, y, w, h in ((-12, -17, 11, 5), (4, -17, 11, 5),
                           (-12, 12, 11, 5), (4, 12, 11, 5)):
            wheel = QGraphicsRectItem(x, y, w, h)
            wheel.setBrush(QBrush(dark)); wheel.setPen(QPen(Qt.PenStyle.NoPen))
            self.addToGroup(wheel)

        nose = QGraphicsPolygonItem(QPolygonF([
            QPointF(12, -8), QPointF(29, 0), QPointF(12, 8),
        ]))
        nose.setBrush(QBrush(light)); nose.setPen(QPen(accent, 2))
        self.addToGroup(nose)

        heading = QGraphicsLineItem(18, 0, 38, 0)
        heading_pen = QPen(dark, 3); heading_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        heading.setPen(heading_pen); self.addToGroup(heading)

        self.label = QGraphicsSimpleTextItem(name)
        self.label.setBrush(QBrush(dark)); self.label.setPos(-18, -40)
        self.addToGroup(self.label)
        self.setZValue(8)

    def set_pose(self, x: float, y: float, yaw_rad: float, state: str) -> None:
        degrees = math.degrees(yaw_rad)
        self.setPos(x * SCALE, -y * SCALE)
        self.setRotation(-degrees)
        self.label.setText(f"{self.data(0)} · {degrees:+.0f}° · {state}")
        self.label.setRotation(degrees)

    def set_name(self, name: str) -> None:
        self.setData(0, name)


def configure_korean_font(app: QApplication) -> tuple[bool, str]:
    """Load the bundled Korean font before any widgets are constructed."""
    candidates = [
        ROOT / "assets" / "fonts" / "NotoSansKR-Regular.woff2",
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf"),
    ]
    for path in candidates:
        if not path.is_file():
            continue
        font_id = QFontDatabase.addApplicationFont(str(path))
        if font_id < 0:
            continue
        families = QFontDatabase.applicationFontFamilies(font_id)
        if families:
            family = families[0]
            app.setFont(QFont(family, 10))
            return True, family

    installed = set(QFontDatabase.families())
    for family in ("Noto Sans CJK KR", "Noto Sans KR", "NanumGothic", "Malgun Gothic"):
        if family in installed:
            app.setFont(QFont(family, 10))
            return True, family
    return False, app.font().family()


class NodeItem(QGraphicsEllipseItem):
    def __init__(self, editor: "MapScene", index: int, node: dict):
        super().__init__(-11, -11, 22, 22)
        self.editor = editor
        self.index = index
        self.setPos(node["x"] * SCALE, -node["y"] * SCALE)
        self.setBrush(QBrush(QColor("#f8fafc")))
        self.setPen(QPen(QColor("#2563eb"), 2))
        self.setFlags(
            QGraphicsItem.GraphicsItemFlag.ItemIsMovable
            | QGraphicsItem.GraphicsItemFlag.ItemIsSelectable
            | QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )
        self.setZValue(3)
        label = QGraphicsSimpleTextItem(f"{index} · {node['name']}", self)
        label.setBrush(QBrush(QColor("#1e293b")))
        label.setPos(14, -22)

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self.editor.node_moved(self)
        return super().itemChange(change, value)


class LaneItem(QGraphicsLineItem):
    def __init__(self, editor: "MapScene", index: int, lane: dict):
        super().__init__()
        self.editor = editor
        self.index = index
        self.runtime_penalty = 0.0
        self.runtime_occupancy = 0.0
        self.corridor_id = ""
        self.corridor_state = "FREE"
        self.corridor_overlay_visible = True
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setZValue(1)
        self.refresh_style(lane)

    def refresh_style(self, lane: dict) -> None:
        corridor_color = {
            "RESERVED": "#f59e0b",
            "OCCUPIED": "#dc2626",
            "UNKNOWN_HOLD": "#991b1b",
            "FREE": "#0ea5e9",
        }.get(self.corridor_state, "#0ea5e9")
        color = "#ef4444" if lane.get("closed") else (
            corridor_color if self.corridor_overlay_visible and self.corridor_id else
            "#059669" if self.runtime_penalty > 0 else
            "#7c3aed" if float(lane.get("after_penalty", 0) or 0) > 0 else
            "#f59e0b" if lane.get("mutex_group") else "#64748b"
        )
        emphasized = (
            lane.get("mutex_group")
            or float(lane.get("after_penalty", 0) or 0) > 0
            or self.runtime_penalty > 0
            or (self.corridor_overlay_visible and bool(self.corridor_id))
        )
        pen = QPen(QColor(color), 5 if emphasized else 3)
        if lane.get("closed"):
            pen.setStyle(Qt.PenStyle.DashLine)
        self.setPen(pen)
        if self.corridor_overlay_visible and self.corridor_id:
            self.setToolTip(
                f"Physical corridor={self.corridor_id} · state={self.corridor_state} · "
                f"policy source=POLICY_DERIVED")
        elif self.runtime_penalty > 0:
            self.setToolTip(
                f"AFTER 자동 penalty={self.runtime_penalty:g} · "
                f"예상 통로 수요={self.runtime_occupancy:g}대")
        else:
            self.setToolTip("")

    def set_corridor_state(
        self, lane: dict, corridor_id: str, state: str, visible: bool,
    ) -> None:
        self.corridor_id = corridor_id
        self.corridor_state = state or "FREE"
        self.corridor_overlay_visible = visible
        self.refresh_style(lane)

    def set_runtime_penalty(
        self, lane: dict, penalty: float = 0.0, occupancy: float = 0.0,
    ) -> None:
        self.runtime_penalty = max(0.0, float(penalty))
        self.runtime_occupancy = max(0.0, float(occupancy))
        self.refresh_style(lane)


class MapScene(QGraphicsScene):
    def __init__(self, selection_callback):
        super().__init__(-560, -400, 1120, 800)
        self.document: dict = {}
        self.node_items: list[NodeItem] = []
        self.lane_items: list[LaneItem] = []
        self.robot_items: dict[str, RobotItem] = {}
        self.robot_path_items: list[QGraphicsItem] = []
        self.corridor_overlay_visible = True
        self.corridor_states: dict[str, str] = {}
        self.selectionChanged.connect(selection_callback)
        self.setBackgroundBrush(QBrush(QColor("#f8fafc")))

    def load_document(self, document: dict) -> None:
        self.clear()
        self.document = document
        self.node_items = [NodeItem(self, i, node) for i, node in enumerate(document["nodes"])]
        for item in self.node_items:
            self.addItem(item)
        self.lane_items = [LaneItem(self, i, lane) for i, lane in enumerate(document["lanes"])]
        for item in self.lane_items:
            self.addItem(item)
        self.robot_items = {}
        self.robot_path_items = []
        self.update_lines()
        self.refresh_corridor_overlay()
        bounds = self.itemsBoundingRect()
        self.setSceneRect(bounds.adjusted(-120, -120, 120, 120))

    def node_moved(self, item: NodeItem) -> None:
        if not self.document or item.index >= len(self.document.get("nodes", [])):
            return
        self.document["nodes"][item.index]["x"] = round(item.pos().x() / SCALE, 4)
        self.document["nodes"][item.index]["y"] = round(-item.pos().y() / SCALE, 4)
        self.update_lines()

    def update_lines(self) -> None:
        for item, lane in zip(self.lane_items, self.document.get("lanes", [])):
            a, b = lane.get("from", -1), lane.get("to", -1)
            if 0 <= a < len(self.node_items) and 0 <= b < len(self.node_items):
                item.setLine(self.node_items[a].pos().x(), self.node_items[a].pos().y(),
                             self.node_items[b].pos().x(), self.node_items[b].pos().y())
                item.refresh_style(lane)

    def set_runtime_penalties(
        self, directed_penalties: dict[int, float],
        directed_occupancy: dict[int, float],
    ) -> None:
        """Project compiled directed-lane evidence back onto source map lanes."""
        directed_id = 0
        for source_index, (item, lane) in enumerate(zip(
            self.lane_items, self.document.get("lanes", []),
        )):
            ids = [directed_id]
            directed_id += 1
            if bool(lane.get("bidirectional", True)):
                ids.append(directed_id)
                directed_id += 1
            penalty = max((directed_penalties.get(i, 0.0) for i in ids), default=0.0)
            occupancy = max((directed_occupancy.get(i, 0.0) for i in ids), default=0.0)
            item.set_runtime_penalty(lane, penalty, occupancy)

    def set_corridor_overlay(
        self, visible: bool, states: dict[str, str] | None = None,
    ) -> None:
        self.corridor_overlay_visible = bool(visible)
        if states is not None:
            self.corridor_states = dict(states)
        self.refresh_corridor_overlay()

    def refresh_corridor_overlay(self) -> None:
        edge_to_corridor: dict[tuple[int, int], str] = {}
        for corridor in self.document.get("corridors", []):
            corridor_id = str(corridor.get("id", ""))
            for edge in corridor.get("forward_edges", []):
                if isinstance(edge, list) and len(edge) == 2:
                    edge_to_corridor[(int(edge[0]), int(edge[1]))] = corridor_id
            for edge in corridor.get("reverse_edges", []):
                if isinstance(edge, list) and len(edge) == 2:
                    edge_to_corridor[(int(edge[0]), int(edge[1]))] = corridor_id
        for item, lane in zip(self.lane_items, self.document.get("lanes", [])):
            corridor_id = edge_to_corridor.get(
                (int(lane.get("from", -1)), int(lane.get("to", -1))), "")
            if not corridor_id and bool(lane.get("bidirectional", True)):
                corridor_id = edge_to_corridor.get(
                    (int(lane.get("to", -1)), int(lane.get("from", -1))), "")
            item.set_corridor_state(
                lane, corridor_id, self.corridor_states.get(corridor_id, "FREE"),
                self.corridor_overlay_visible)

    def selected_node_indexes(self) -> list[int]:
        return [item.index for item in self.selectedItems() if isinstance(item, NodeItem)]

    def clear_robot_overlay(self) -> None:
        for item in self.robot_items.values():
            self.removeItem(item)
        self.robot_items = {}

        for item in self.robot_path_items:
            self.removeItem(item)
        self.robot_path_items = []

    def set_robot_trajectory(self, points: list[dict], color: str) -> None:
        if not points:
            return
        path = QPainterPath()
        path.moveTo(float(points[0]["x"]) * SCALE, -float(points[0]["y"]) * SCALE)
        for point in points[1:]:
            path.lineTo(float(point["x"]) * SCALE, -float(point["y"]) * SCALE)
        item = QGraphicsPathItem(path)
        path_color = QColor(color); path_color.setAlpha(150)
        pen = QPen(path_color, 3); pen.setStyle(Qt.PenStyle.DashLine)
        item.setPen(pen); item.setZValue(4)
        self.addItem(item); self.robot_path_items.append(item)

        goal = points[-1]
        goal_item = QGraphicsEllipseItem(-8, -8, 16, 16)
        goal_item.setPos(float(goal["x"]) * SCALE, -float(goal["y"]) * SCALE)
        goal_item.setBrush(QBrush(QColor(color)))
        goal_item.setPen(QPen(QColor("#1e293b"), 2))
        goal_item.setZValue(5)
        self.addItem(goal_item); self.robot_path_items.append(goal_item)

    def set_robot_pose(
        self, name: str, x: float, y: float, yaw_rad: float,
        state: str, color: str,
    ) -> None:
        if name not in self.robot_items:
            marker = RobotItem(name, color)
            marker.set_name(name)
            self.addItem(marker)
            self.robot_items[name] = marker
        self.robot_items[name].set_pose(x, y, yaw_rad, state)


class MapView(QGraphicsView):
    """Cursor-centered wheel zoom plus middle-button panning."""

    def __init__(self, scene: QGraphicsScene, zoom_callback):
        super().__init__(scene)
        self.zoom_callback = zoom_callback
        self._panning = False
        self._pan_position = None
        self.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)

    def zoom_percent(self) -> int:
        return int(round(self.transform().m11() * 100))

    def _zoom(self, factor: float) -> None:
        current = self.transform().m11()
        target = current * factor
        if target < 0.08 or target > 8.0:
            return
        self.scale(factor, factor)
        self.zoom_callback(self.zoom_percent())

    def zoom_in(self) -> None:
        self._zoom(1.25)

    def zoom_out(self) -> None:
        self._zoom(0.8)

    def reset_zoom(self) -> None:
        self.resetTransform()
        self.zoom_callback(100)

    def fit_content(self) -> None:
        if self.scene() and self.scene().items():
            self.fitInView(
                self.scene().itemsBoundingRect().adjusted(-50, -50, 50, 50),
                Qt.AspectRatioMode.KeepAspectRatio,
            )
            self.zoom_callback(self.zoom_percent())

    def wheelEvent(self, event) -> None:
        self._zoom(1.2 if event.angleDelta().y() > 0 else 1 / 1.2)
        event.accept()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._pan_position = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._panning and self._pan_position is not None:
            delta = event.position() - self._pan_position
            self._pan_position = event.position()
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - int(delta.x()))
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - int(delta.y()))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.MiddleButton and self._panning:
            self._panning = False
            self._pan_position = None
            self.unsetCursor()
            event.accept()
            return
        super().mouseReleaseEvent(event)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RMF Traffic Core Desktop Simulator")
        self.setMinimumSize(1200, 760)
        self.resize(1840, 1120)
        self.settings = QSettings("RMF Traffic Lab", "Desktop Simulator")
        self.start_maximized = True
        self.templates = builtin_scenarios()
        for scenario_file in sorted((ROOT / "scenarios").glob("*.json")):
            try:
                payload = json.loads(scenario_file.read_text(encoding="utf-8"))
                self.templates[f"example · {scenario_file.stem}"] = payload
            except (OSError, json.JSONDecodeError):
                pass
        self.document: dict = {}
        self.process: QProcess | None = None
        self.regression_process: QProcess | None = None
        self.regression_output_text = ""
        self.regression_summary_path: Path | None = None
        self.events: list[dict] = []
        self.runtime_output_text = ""
        self.schedule_event_rows: dict[QTableWidget, list[dict]] = {}
        self.object_event_rows: dict[QTableWidget, list[dict]] = {}
        self.astar_event_rows: list[dict] = []
        self.decision_rows: list[dict] = []
        self.decision_row_by_seq: dict[object, int] = {}
        self.animation_decisions: list[dict] = []
        self.live_robot_context: dict[str, dict] = {}
        self.last_animation_decision_seq: object = None
        self.trajectories: dict[str, list[dict]] = {}
        self.animation_time = 0.0
        self.animation_end = 0.0
        self.playback_speed = 1.0
        self.animation_timer = QTimer(self)
        self.animation_timer.setInterval(33)
        self.animation_timer.timeout.connect(self.advance_animation)
        self.jsonl_timer = QTimer(self)
        self.jsonl_timer.setInterval(250)
        self.jsonl_timer.timeout.connect(self.refresh_live_jsonl)
        self.result_path = ROOT / "results" / "gui_baseline.jsonl"
        self.last_jsonl_content = ""

        self.scene = MapScene(self.on_selection_changed)
        self.view = MapView(self.scene, self.on_zoom_changed)

        self._build_toolbar()
        self._build_ui()
        self.restore_workspace_layout()
        self.load_named_scenario("single_lane_bidirectional")
        self.update_comparison()

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("실행", self)
        toolbar.setObjectName("mainToolbar")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        scenario_label = QLabel("시나리오")
        scenario_label.setObjectName("toolbarSection")
        toolbar.addWidget(scenario_label)
        self.scenario_combo = QComboBox()
        self.scenario_combo.setMinimumWidth(200)
        self.scenario_combo.addItems(self.templates.keys())
        self.scenario_combo.currentTextChanged.connect(self.load_named_scenario)
        toolbar.addWidget(self.scenario_combo)
        toolbar.addSeparator()
        open_action = QAction("JSON 열기", self)
        open_action.triggered.connect(self.open_json)
        toolbar.addAction(open_action)
        open_yaml_action = QAction("YAML 맵 열기", self)
        open_yaml_action.triggered.connect(self.open_yaml)
        toolbar.addAction(open_yaml_action)
        save_action = QAction("JSON 저장", self)
        save_action.triggered.connect(self.save_json)
        toolbar.addAction(save_action)
        toolbar.addSeparator()
        core_label = QLabel("RMF 코어")
        core_label.setObjectName("toolbarSection")
        toolbar.addWidget(core_label)
        self.core_profile_combo = QComboBox()
        self.core_profile_combo.addItem("BASELINE · Stock RMF", "baseline")
        self.core_profile_combo.addItem("OLD_SOFT · 기존 Schedule-aware 비용", "soft")
        self.core_profile_combo.addItem("SCHEDULE_SOFT · Snapshot overlap only", "schedule_soft")
        self.core_profile_combo.addItem("HYBRID · Soft + Admission", "hybrid")
        self.core_profile_combo.addItem(
            "HYBRID + NEGO · Admission + 신규협상", "hybrid_nego")
        self.core_profile_combo.currentIndexChanged.connect(self.core_profile_changed)
        toolbar.addWidget(self.core_profile_combo)
        self.core_status_label = QLabel("BASELINE")
        self.core_status_label.setObjectName("coreBadge")
        toolbar.addWidget(self.core_status_label)
        setup_label = QLabel("setup.bash")
        setup_label.setObjectName("toolbarSection")
        toolbar.addWidget(setup_label)
        self.setup_edit = QLineEdit("~/rmf_ws/install/setup.bash")
        self.setup_edit.setMinimumWidth(240)
        toolbar.addWidget(self.setup_edit)
        self.build_check = QCheckBox("변경사항 다시 빌드")
        self.build_check.setChecked(True)
        self.build_check.setToolTip(
            "체크하면 rmf_core_lab C++ 실행 파일을 먼저 빌드합니다. "
            "소스나 RMF 코어를 바꾼 뒤에는 체크하세요.")
        self.build_check.toggled.connect(self.update_run_button_text)
        toolbar.addWidget(self.build_check)
        timeout_label = QLabel("실행 제한")
        timeout_label.setObjectName("toolbarSection")
        toolbar.addWidget(timeout_label)
        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(1, 3600)
        self.timeout_spin.setValue(60)
        self.timeout_spin.setSuffix(" s")
        toolbar.addWidget(self.timeout_spin)
        self.run_button = QPushButton()
        self.run_button.setObjectName("primaryRun")
        self.run_button.setMinimumWidth(220)
        self.run_button.setToolTip(
            "편집한 시나리오를 실제 C++ rmf_traffic Planner/Negotiation/Database로 분석합니다. "
            "체크 상태에 따라 rmf_core_lab 실행 파일을 먼저 다시 빌드합니다.")
        self.run_button.clicked.connect(self.run_rmf)
        toolbar.addWidget(self.run_button)
        self.stop_button = QPushButton("■ 중지")
        self.stop_button.setObjectName("dangerButton")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_process)
        toolbar.addWidget(self.stop_button)
        help_button = QPushButton("사용법")
        help_button.clicked.connect(self.show_usage)
        toolbar.addWidget(help_button)
        self.update_run_button_text()

    def update_run_button_text(self, *_args) -> None:
        if not hasattr(self, "run_button"):
            return
        self.run_button.setText(
            "▶ 변경사항 빌드 후 RMF 분석"
            if self.build_check.isChecked()
            else "▶ 빌드된 RMF로 계획 분석"
        )

    def _build_ui(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        edit_bar = QHBoxLayout()
        edit_title = QLabel("맵 편집")
        edit_title.setObjectName("sectionTitle")
        edit_bar.addWidget(edit_title)
        for label, callback in (
            ("노드 추가", self.add_node), ("선택 노드 양방향 Lane", self.add_lane),
            ("선택 삭제", self.delete_selected), ("−", self.view.zoom_out),
            ("+", self.view.zoom_in), ("100%", self.view.reset_zoom),
            ("화면 맞춤", self.fit_map),
        ):
            button = QPushButton(label)
            button.clicked.connect(callback)
            edit_bar.addWidget(button)
        self.zoom_label = QLabel("100%")
        edit_bar.addWidget(self.zoom_label)
        edit_bar.addStretch()
        self.graph_summary = QLabel()
        self.graph_summary.setObjectName("graphBadge")
        edit_bar.addWidget(self.graph_summary)
        outer.addLayout(edit_bar)

        layout_bar = QHBoxLayout()
        resize_hint = QLabel("파란 분할선 드래그 = 지도·속성·로그 크기 조절")
        resize_hint.setToolTip(
            "지도와 오른쪽 속성 패널 사이, 지도와 하단 결과 사이의 분할선을 드래그하세요.")
        layout_bar.addWidget(resize_hint)
        layout_bar.addStretch()
        for label, callback in (
            ("지도 넓게", self.focus_map_layout),
            ("하단 결과 접기/펼치기", self.toggle_output_panel),
            ("기본 배치", self.reset_workspace_layout),
            ("창 최대화/복원", self.toggle_window_size),
        ):
            button = QPushButton(label)
            button.clicked.connect(callback)
            layout_bar.addWidget(button)
        outer.addLayout(layout_bar)

        map_panel = QWidget(); map_layout = QVBoxLayout(map_panel)
        map_panel.setMinimumWidth(700)
        map_panel.setMinimumHeight(180)
        map_panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Ignored)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_header = QHBoxLayout()
        map_title = QLabel("2D RMF 실행 뷰")
        map_title.setObjectName("sectionTitle")
        map_header.addWidget(map_title)
        map_header.addStretch()
        self.corridor_overlay_check = QCheckBox("Corridor 상태 표시")
        self.corridor_overlay_check.setChecked(True)
        self.corridor_overlay_check.toggled.connect(
            lambda checked: self.scene.set_corridor_overlay(checked))
        map_header.addWidget(self.corridor_overlay_check)
        map_header.addWidget(QLabel(
            "흰색 노즈=전진 · 점선=최종 계획 · 하늘=FREE · 주황=RESERVED · 빨강=OCCUPIED"))
        map_layout.addLayout(map_header)
        map_layout.addWidget(self.view)
        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.main_splitter.setObjectName("mapEditorSplitter")
        self.main_splitter.setMinimumHeight(180)
        self.main_splitter.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Ignored)
        self.main_splitter.setHandleWidth(8)
        self.main_splitter.setChildrenCollapsible(False)
        editor_panel = self._make_editor_panel()
        editor_panel.setMinimumWidth(410)
        self.main_splitter.addWidget(map_panel)
        self.main_splitter.addWidget(editor_panel)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 0)
        self.main_splitter.setSizes([1380, 460])

        self.output_tabs = QTabWidget()
        self.output_tabs.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.runtime_log = QTextEdit(); self.runtime_log.setReadOnly(True)
        self.runtime_log_korean = QTextEdit(); self.runtime_log_korean.setReadOnly(True)
        self.raw_log = QTextEdit(); self.raw_log.setReadOnly(True)
        self.jsonl_summary = QTextEdit(); self.jsonl_summary.setReadOnly(True)
        self.diagnosis = QTextEdit(); self.diagnosis.setReadOnly(True)
        self.diagnosis_raw = QTextEdit(); self.diagnosis_raw.setReadOnly(True)

        schedule_panel = QWidget(); schedule_layout = QVBoxLayout(schedule_panel)
        schedule_panel.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        schedule_layout.setContentsMargins(4, 4, 4, 4)
        schedule_copy_bar = QHBoxLayout()
        schedule_copy_selected = QPushButton("현재 표 선택 복사")
        schedule_copy_selected.clicked.connect(lambda: self.copy_current_schedule(False))
        schedule_copy_all = QPushButton("현재 표 전체 복사")
        schedule_copy_all.clicked.connect(lambda: self.copy_current_schedule(True))
        schedule_copy_bar.addWidget(schedule_copy_selected)
        schedule_copy_bar.addWidget(schedule_copy_all)
        schedule_copy_bar.addWidget(QLabel("Ctrl+C: 선택 셀 · Ctrl+Shift+C: 전체 표"))
        schedule_copy_bar.addStretch()
        schedule_layout.addLayout(schedule_copy_bar)
        self.schedule_tabs = QTabWidget()
        self.schedule_tabs.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.schedule_guide = QTextEdit(); self.schedule_guide.setReadOnly(True)
        self.schedule_guide.setPlainText(schedule_guide_text())
        self.schedule_state_table = self.make_table(
            ["seq", "저장 시점", "DB 버전", "참가자 수", "저장 Route 수",
             "DB class", "View class", "실제 읽기 API"])
        self.schedule_operation_table = self.make_table(
            ["seq", "작업", "실제 API", "이전 DB 버전", "이후 DB 버전", "참가자 ID", "결과"])
        self.schedule_participant_table = self.make_table(
            ["저장 시점", "ID", "로봇 이름", "owner", "responsive", "profile",
             "itinerary 버전", "progress 버전", "plan ID", "Route 수", "궤적점 수",
             "cumulative delay(s)", "reached checkpoint", "runtime source", "실제 읽기 API"])
        self.schedule_route_table = self.make_table(
            ["저장 시점", "참가자 ID", "로봇", "plan ID", "route ID", "map",
             "시작(s)", "종료(s)", "기간(s)", "궤적점 수", "associated corridor",
             "direction", "corridor enter(s)", "corridor exit(s)", "analysis source",
             "RMF 객체 경로"])
        self.schedule_point_table = self.make_table(
            ["저장 시점", "참가자", "로봇", "plan", "route", "점 순번", "시각(s)",
             "x", "y", "yaw", "vx", "vy", "yaw rate", "RMF 객체 경로"])
        self.schedule_tabs.addTab(self.schedule_state_table, "DB 상태")
        self.schedule_tabs.addTab(self.schedule_operation_table, "쓰기/읽기 API")
        self.schedule_tabs.addTab(self.schedule_participant_table, "Participants")
        self.schedule_tabs.addTab(self.schedule_route_table, "Itineraries")
        self.schedule_tabs.addTab(self.schedule_point_table, "Trajectory 원본")
        self.schedule_explanation = QTextEdit(); self.schedule_explanation.setReadOnly(True)
        self.schedule_explanation.setPlainText(
            "왼쪽 표에서 행을 선택하면 이곳에 해당 DB 값의 의미를 표시합니다.")
        schedule_right_tabs = QTabWidget()
        schedule_right_tabs.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.schedule_model = QTextEdit(); self.schedule_model.setReadOnly(True)
        self.schedule_model.setPlainText(schedule_model_text())
        schedule_right_tabs.addTab(self.schedule_explanation, "선택 행 해석")
        schedule_right_tabs.addTab(self.schedule_model, "실제 RMF 객체 구조")
        schedule_right_tabs.addTab(self.schedule_guide, "용어·버전 가이드")
        self.schedule_split = QSplitter(Qt.Orientation.Horizontal)
        self.schedule_split.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.schedule_split.setChildrenCollapsible(False)
        self.schedule_split.addWidget(self.schedule_tabs)
        self.schedule_split.addWidget(schedule_right_tabs)
        self.schedule_split.setStretchFactor(0, 3)
        self.schedule_split.setStretchFactor(1, 1)
        self.schedule_split.setSizes([1080, 430])
        schedule_layout.addWidget(self.schedule_split, 1)
        for table in (
            self.schedule_state_table, self.schedule_operation_table,
            self.schedule_participant_table, self.schedule_route_table,
            self.schedule_point_table,
        ):
            table.itemSelectionChanged.connect(
                lambda selected_table=table: self.show_schedule_row_explanation(selected_table))

        astar_panel = QWidget(); astar_layout = QVBoxLayout(astar_panel)
        astar_panel.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        astar_layout.setContentsMargins(4, 4, 4, 4)
        astar_copy_bar = QHBoxLayout()
        astar_copy_selected = QPushButton("선택 복사")
        astar_copy_selected.clicked.connect(lambda: self.copy_table(self.astar_table, False))
        astar_copy_all = QPushButton("전체 A* 복사")
        astar_copy_all.clicked.connect(lambda: self.copy_table(self.astar_table, True))
        astar_copy_bar.addWidget(astar_copy_selected); astar_copy_bar.addWidget(astar_copy_all)
        astar_copy_bar.addWidget(QLabel("행을 누르면 실제 g/h/f 기준 설명 표시"))
        astar_copy_bar.addStretch(); astar_layout.addLayout(astar_copy_bar)
        self.astar_table = CopyableTableWidget(0, 25)
        self.astar_table.setHorizontalHeaderLabels(
            ["seq", "robot", "event", "step", "node", "parent", "waypoint",
             "g", "Δg", "구간시간", "이동시간", "회전시간", "대기시간", "이동거리",
             "회전각", "g 미노출차", "h", "그래프거리", "그래프주행시간",
             "첫회전시간", "h-그래프시간", "f", "queue", "next f", "selection basis"])
        for table in (self.astar_table,):
            table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
            table.verticalHeader().setDefaultSectionSize(25)
            table.verticalHeader().setVisible(False)
            table.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            table.setMinimumHeight(0)
            table.setAlternatingRowColors(True)
        self.astar_table.itemSelectionChanged.connect(self.show_astar_row_explanation)
        self.policy_astar_table = self.make_table([
            "seq", "candidate", "parent", "participant", "current wp", "target wp",
            "lane ids", "corridor", "direction", "enter(s)", "exit(s)",
            "parent_g", "approach", "RMF alt", "movement", "rotation", "event", "waiting", "static",
            "shared traffic", "same", "opposite", "occupancy", "no escape",
            "total policy", "new_g", "h", "f", "decision", "Schedule overlaps", "source",
        ])
        self.policy_astar_table.itemSelectionChanged.connect(
            self.show_policy_astar_explanation)
        self.astar_explanation = QTextEdit(); self.astar_explanation.setReadOnly(True)
        self.astar_explanation.setPlainText("왼쪽 A* 행을 선택하면 실제 g/h/f 근거를 표시합니다.")
        self.astar_guide = QTextEdit(); self.astar_guide.setReadOnly(True)
        self.astar_guide.setPlainText(astar_guide_text())
        astar_right_tabs = QTabWidget()
        astar_right_tabs.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        astar_right_tabs.addTab(self.astar_explanation, "선택 단계 해석")
        astar_right_tabs.addTab(self.astar_guide, "g/h/f 가이드")
        self.astar_split = QSplitter(Qt.Orientation.Horizontal)
        self.astar_split.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.astar_split.setChildrenCollapsible(False)
        astar_data_tabs = QTabWidget()
        astar_data_tabs.addTab(self.astar_table, "RMF Planner Debug g/h/f")
        astar_data_tabs.addTab(self.policy_astar_table, "Corridor policy expansion")
        self.astar_split.addWidget(astar_data_tabs)
        self.astar_split.addWidget(astar_right_tabs)
        self.astar_split.setStretchFactor(0, 3)
        self.astar_split.setStretchFactor(1, 1)
        self.astar_split.setSizes([1090, 420])
        astar_layout.addWidget(self.astar_split, 1)

        decision_panel = QWidget(); decision_layout = QVBoxLayout(decision_panel)
        decision_panel.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        decision_layout.setContentsMargins(4, 4, 4, 4)
        decision_copy_bar = QHBoxLayout()
        decision_copy_selected = QPushButton("선택 복사")
        decision_copy_selected.clicked.connect(lambda: self.copy_table(self.decision_table, False))
        decision_copy_all = QPushButton("전체 판단 타임라인 복사")
        decision_copy_all.clicked.connect(lambda: self.copy_table(self.decision_table, True))
        decision_copy_bar.addWidget(decision_copy_selected); decision_copy_bar.addWidget(decision_copy_all)
        decision_copy_bar.addWidget(QLabel("재생 중 현재 시간의 판단 단계와 자동 동기화"))
        decision_copy_bar.addStretch(); decision_layout.addLayout(decision_copy_bar)
        self.decision_table = self.make_table(
            ["seq", "단계", "로봇", "무엇을 결정", "왜", "실제 근거값", "결과"])
        self.decision_table.itemSelectionChanged.connect(self.show_decision_row_explanation)
        self.decision_explanation = QTextEdit(); self.decision_explanation.setReadOnly(True)
        self.decision_explanation.setPlainText(
            "실행 후 행을 선택하면 해당 순간의 판단과 실제 JSONL 근거를 한글로 설명합니다."
        )
        self.decision_guide = QTextEdit(); self.decision_guide.setReadOnly(True)
        self.decision_guide.setPlainText(
            "판단 타임라인 읽는 순서\n\n"
            "1. Planner 요청\n2. A* frontier 선택과 후보 생성\n3. 경로 후보 비용 비교\n"
            "4. 협상 table·plan 제출·거부\n5. 연속시간 충돌검사\n6. Schedule DB 반영\n\n"
            "재생 중에는 현재 이동 목표에 해당하는 plan_waypoint 행을 자동 선택합니다."
        )
        decision_right_tabs = QTabWidget()
        decision_right_tabs.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        decision_right_tabs.addTab(self.decision_explanation, "선택 판단 해석")
        decision_right_tabs.addTab(self.decision_guide, "전체 흐름 가이드")
        self.decision_split = QSplitter(Qt.Orientation.Horizontal)
        self.decision_split.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.decision_split.setChildrenCollapsible(False)
        self.decision_split.addWidget(self.decision_table)
        self.decision_split.addWidget(decision_right_tabs)
        self.decision_split.setStretchFactor(0, 3)
        self.decision_split.setStretchFactor(1, 1)
        self.decision_split.setSizes([1070, 440])
        decision_layout.addWidget(self.decision_split, 1)

        object_panel = QWidget(); object_layout = QVBoxLayout(object_panel)
        object_panel.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        object_layout.setContentsMargins(4, 4, 4, 4)
        object_copy_bar = QHBoxLayout()
        object_copy_selected = QPushButton("현재 표 선택 복사")
        object_copy_selected.clicked.connect(
            lambda: self.copy_current_object_table(False))
        object_copy_all = QPushButton("현재 표 전체 복사")
        object_copy_all.clicked.connect(
            lambda: self.copy_current_object_table(True))
        object_copy_bar.addWidget(object_copy_selected)
        object_copy_bar.addWidget(object_copy_all)
        object_copy_bar.addWidget(QLabel(
            "Graph→Start/Goal→Validator→Plan/Itinerary→Proposal→협상→Commit 실제 이벤트"))
        object_copy_bar.addStretch()
        object_layout.addLayout(object_copy_bar)

        self.object_tabs = QTabWidget()
        self.object_tabs.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.supergraph_table = self.make_table([
            "seq", "Graph class", "Graph API", "waypoints", "directed lanes",
            "Supergraph class", "public API", "실제 관찰 범위",
        ])
        self.graph_node_table = self.make_table([
            "seq", "id", "name", "map", "x", "y", "holding", "parking",
            "passthrough", "mutex", "outgoing lanes", "incoming lanes",
        ])
        self.graph_lane_table = self.make_table([
            "seq", "id", "entry", "exit", "length(m)", "speed limit", "effective speed",
            "mutex", "closed",
        ])
        self.start_goal_table = self.make_table([
            "seq", "robot", "mode", "Start type", "start node", "start time(s)",
            "yaw", "Goal type", "goal node", "goal orientation", "any orientation",
            "dynamic insertion(s)",
        ])
        self.validator_table = self.make_table([
            "seq", "phase", "stage", "Planner RouteValidator", "객체 공개",
            "Schedule 인지", "DB version", "post validator", "목적·한계",
        ])
        self.itinerary_table = self.make_table([
            "seq", "robot", "phase", "object", "Route 수", "source API",
            "Schedule commit 여부", "의미",
        ])
        self.route_object_table = self.make_table([
            "seq", "robot", "phase", "route index", "map", "Trajectory points",
            "start(s)", "finish(s)", "duration(s)", "source API",
        ])
        self.trajectory_object_table = self.make_table([
            "seq", "robot", "phase", "route", "point", "time(s)", "x", "y", "yaw",
            "vx", "vy", "yaw rate", "source API",
        ])
        self.proposal_table = self.make_table([
            "seq", "event", "phase", "stage", "robot", "participant", "present",
            "Plan 수", "cost", "waypoints", "Route 수", "points", "validated",
            "accepted", "committed", "action/reason",
        ])
        self.negotiation_timeline_table = self.make_table([
            "seq", "event", "phase", "stage", "robot/참가자", "조치",
            "결과", "실제 API·원문·근거",
        ])
        self.negotiation_process_table = self.make_table([
            "seq", "stage", "action", "한글 의미", "실제 RMF 원문", "source API",
        ])
        self.reject_forfeit_table = self.make_table([
            "seq", "source", "stage", "action", "한글 의미", "accepted", "committed",
            "실제 원문·판정 이유",
        ])
        for table, label in (
            (self.supergraph_table, "Graph·Supergraph"),
            (self.graph_node_table, "Graph Waypoints"),
            (self.graph_lane_table, "Graph Lanes"),
            (self.start_goal_table, "Start·Goal"),
            (self.validator_table, "Validator"),
            (self.itinerary_table, "Itinerary"),
            (self.route_object_table, "Route"),
            (self.trajectory_object_table, "Trajectory"),
            (self.proposal_table, "Proposal"),
            (self.negotiation_timeline_table, "협상 전체 시퀀스"),
            (self.negotiation_process_table, "협상 원문 과정"),
            (self.reject_forfeit_table, "Reject·Forfeit"),
        ):
            self.object_tabs.addTab(table, label)
            table.itemSelectionChanged.connect(
                lambda selected_table=table: self.show_object_row_explanation(
                    selected_table))

        self.object_explanation = QTextEdit()
        self.object_explanation.setReadOnly(True)
        self.object_explanation.setPlainText(
            "왼쪽 표에서 행을 선택하면 실제 C++ 객체/API와 해석 한계를 표시합니다.")
        self.object_guide = QTextEdit(); self.object_guide.setReadOnly(True)
        self.object_guide.setPlainText(rmf_object_guide_text())
        object_right_tabs = QTabWidget()
        object_right_tabs.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        object_right_tabs.addTab(self.object_explanation, "선택 행 상세")
        object_right_tabs.addTab(self.object_guide, "객체·협상 흐름 가이드")
        self.object_split = QSplitter(Qt.Orientation.Horizontal)
        self.object_split.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.object_split.setChildrenCollapsible(False)
        self.object_split.addWidget(self.object_tabs)
        self.object_split.addWidget(object_right_tabs)
        self.object_split.setStretchFactor(0, 3)
        self.object_split.setStretchFactor(1, 1)
        self.object_split.setSizes([1100, 410])
        object_layout.addWidget(self.object_split, 1)

        corridor_panel = QWidget(); corridor_layout = QVBoxLayout(corridor_panel)
        corridor_bar = QHBoxLayout()
        corridor_bar.addWidget(QLabel(
            "Physical corridor 정의, Schedule snapshot, admission/soft-cost 판정을 source와 함께 표시합니다."))
        corridor_bar.addStretch()
        corridor_copy = QPushButton("현재 Corridor 표 전체 복사")
        corridor_copy.clicked.connect(
            lambda: self.copy_table(self.corridor_tabs.currentWidget(), True))
        corridor_bar.addWidget(corridor_copy)
        corridor_layout.addLayout(corridor_bar)
        self.corridor_tabs = QTabWidget()
        self.corridor_definition_table = self.make_table([
            "seq", "corridor", "forward lanes", "reverse lanes", "capacity",
            "passing", "hard opposite", "entry A", "entry B", "base penalty", "source",
        ])
        self.corridor_snapshot_table = self.make_table([
            "seq", "mode", "Schedule version", "participant", "planning t(s)",
            "corridors", "intervals", "reason", "query API", "source",
        ])
        self.corridor_interval_table = self.make_table([
            "seq", "snapshot", "Schedule version", "corridor", "participant",
            "plan", "route", "direction", "enter(s)", "exit(s)", "state", "owner",
            "responsive", "itinerary version", "trajectory source", "state source",
        ])
        self.corridor_decision_table = self.make_table([
            "seq", "participant", "corridor", "direction", "enter(s)", "exit(s)",
            "entry", "decision", "static", "same", "opposite", "occupancy",
            "no escape", "total", "overlap robots", "source",
        ])
        self.corridor_runtime_table = self.make_table([
            "seq", "event", "corridor", "state", "direction", "owner", "occupants",
            "reserved", "wait same", "wait opposite", "interval", "last update(s)",
            "release condition", "passing", "capacity", "robot", "value(s)",
            "Schedule changed/version", "source",
        ])
        self.route_validator_result_table = self.make_table([
            "seq", "phase", "validator", "participant", "candidate route", "decision",
            "reason", "blocker participant", "plan", "route", "conflict t(s)", "source",
        ])
        for table, label in (
            (self.corridor_definition_table, "Lane ↔ Corridor"),
            (self.corridor_snapshot_table, "Planning Snapshot"),
            (self.corridor_interval_table, "Schedule Intervals"),
            (self.corridor_decision_table, "Admission / Soft Cost"),
            (self.corridor_runtime_table, "Reservation / Delay"),
            (self.route_validator_result_table, "RMF Validator 결과"),
        ):
            self.corridor_tabs.addTab(table, label)
        corridor_layout.addWidget(self.corridor_tabs, 1)

        compare_panel = QWidget(); compare_layout = QVBoxLayout(compare_panel)
        compare_bar = QHBoxLayout()
        compare_bar.addWidget(QLabel(
            "동일 Scenario SHA에서 경로 변경·우회·시간조정·협상 성공 여부를 비교합니다."))
        compare_selected = QPushButton("Compare All Versions")
        compare_selected.setObjectName("accentButton")
        compare_selected.clicked.connect(lambda: self.start_regression(False))
        compare_bar.addWidget(compare_selected)
        run_all = QPushButton("Run All Scenarios")
        run_all.clicked.connect(lambda: self.start_regression(True))
        compare_bar.addWidget(run_all)
        self.regression_stop_button = QPushButton("Regression 중지")
        self.regression_stop_button.setObjectName("dangerButton")
        self.regression_stop_button.setEnabled(False)
        self.regression_stop_button.clicked.connect(self.stop_regression)
        compare_bar.addWidget(self.regression_stop_button)
        compare_bar.addStretch()
        compare_copy = QPushButton("비교 결과 전체 복사")
        compare_copy.clicked.connect(lambda: self.copy_table(self.compare_table, True))
        compare_bar.addWidget(compare_copy)
        compare_layout.addLayout(compare_bar)
        self.regression_status_label = QLabel(
            "선택 Scenario 또는 전체 Scenario를 실제 5개 RMF 코어로 순차 실행합니다.")
        self.regression_status_label.setWordWrap(True)
        compare_layout.addWidget(self.regression_status_label)
        self.regression_table = self.make_table([
            "Scenario", "Version", "Result", "Baseline 비교", "Conflict",
            "Deadlock", "Travel(s)", "Wait(s)", "Distance(m)", "Detour",
            "Planning(ms)", "Expanded", "Negotiations", "Rounds",
            "Validator Reject", "Penalty", "Termination reason", "입력 동일",
        ])
        self.regression_table.setMaximumHeight(260)
        compare_layout.addWidget(self.regression_table)
        self.compare_table = self.make_table(
            ["비교 항목", "BASELINE", "OLD_SOFT", "SCHEDULE_SOFT", "HYBRID", "HYBRID+NEGO", "BASELINE→권장모드"])
        self.compare_explanation = QTextEdit(); self.compare_explanation.setReadOnly(True)
        compare_split = QSplitter(Qt.Orientation.Horizontal)
        compare_split.addWidget(self.compare_table)
        compare_split.addWidget(self.compare_explanation)
        compare_split.setSizes([1020, 490])
        compare_layout.addWidget(compare_split)

        failure_panel = QWidget(); failure_layout = QVBoxLayout(failure_panel)
        failure_layout.setContentsMargins(4, 4, 4, 4)
        failure_header = QLabel(
            "실제 JSONL 이벤트만 사용해 Planner → Schedule → Conflict → Negotiation → Final을 추적합니다. "
            "공개 API에 없는 값은 UNKNOWN으로 표시합니다.")
        failure_header.setWordWrap(True)
        failure_layout.addWidget(failure_header)
        self.failure_summary = QTextEdit(); self.failure_summary.setReadOnly(True)
        self.failure_summary.setMaximumHeight(260)
        self.failure_summary.setPlainText("RMF 실행 후 실패 원인 요약이 표시됩니다.")
        failure_layout.addWidget(self.failure_summary)
        self.failure_trace_table = self.make_table([
            "seq", "stage", "robot/pair", "event", "status", "location",
            "time", "actual evidence", "source API",
        ])
        failure_layout.addWidget(self.failure_trace_table, 1)

        self.output_tabs.addTab(self.runtime_log, "실행 로그")
        self.output_tabs.addTab(self.runtime_log_korean, "실행 로그 요약")
        self.output_tabs.addTab(self.raw_log, "원본 JSONL")
        self.output_tabs.addTab(self.jsonl_summary, "JSONL 요약")
        self.output_tabs.addTab(failure_panel, "실패 원인 추적")
        self.output_tabs.addTab(self.diagnosis, "진단 요약")
        self.output_tabs.addTab(self.diagnosis_raw, "진단 원본")
        self.output_tabs.addTab(schedule_panel, "Schedule Database")
        self.output_tabs.addTab(corridor_panel, "Corridor / Reservation")
        self.output_tabs.addTab(astar_panel, "A* 내부 과정")
        self.output_tabs.addTab(decision_panel, "스텝별 판단 근거")
        self.output_tabs.addTab(object_panel, "RMF 객체·협상 원문")
        self.compare_panel = compare_panel
        self.output_tabs.addTab(compare_panel, "5-Mode 비교 / Regression")

        bottom = QWidget()
        bottom.setMinimumHeight(0)
        bottom.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        bottom_layout = QVBoxLayout(bottom)
        controls = QHBoxLayout()
        playback_title = QLabel("궤적 재생")
        playback_title.setObjectName("sectionTitle")
        controls.addWidget(playback_title)
        self.play_button = QPushButton("▶ 계획 재생")
        self.play_button.setObjectName("accentButton")
        self.play_button.clicked.connect(self.toggle_animation)
        controls.addWidget(self.play_button)
        controls.addWidget(QLabel("배속"))
        self.playback_speed_combo = QComboBox()
        for label, value in (
            ("0.25x", 0.25), ("0.5x", 0.5), ("1x", 1.0),
            ("2x", 2.0), ("4x", 4.0), ("8x", 8.0),
        ):
            self.playback_speed_combo.addItem(label, value)
        self.playback_speed_combo.setCurrentIndex(2)
        self.playback_speed_combo.setToolTip(
            "RMF가 계산한 궤적 시간축은 유지하고 화면 재생 속도만 바꿉니다.")
        self.playback_speed_combo.currentIndexChanged.connect(
            self.playback_speed_changed)
        controls.addWidget(self.playback_speed_combo)
        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_slider.setRange(0, 1000)
        self.time_slider.valueChanged.connect(self.slider_changed)
        controls.addWidget(self.time_slider)
        self.time_label = QLabel("0.00 / 0.00 s")
        controls.addWidget(self.time_label)
        bottom_layout.addLayout(controls)
        bottom_layout.addWidget(self.output_tabs, 1)

        self.vertical_splitter = QSplitter(Qt.Orientation.Vertical)
        self.vertical_splitter.setObjectName("mapOutputSplitter")
        self.vertical_splitter.setHandleWidth(14)
        self.vertical_splitter.setOpaqueResize(True)
        self.vertical_splitter.setChildrenCollapsible(True)
        self.vertical_splitter.addWidget(self.main_splitter)
        self.vertical_splitter.addWidget(bottom)
        self.vertical_splitter.setCollapsible(0, False)
        self.vertical_splitter.setCollapsible(1, True)
        self.vertical_splitter.setStretchFactor(0, 3)
        self.vertical_splitter.setStretchFactor(1, 2)
        self.last_output_height = 320
        self.vertical_splitter.setSizes([800, 270])
        outer.addWidget(self.vertical_splitter)
        self.setCentralWidget(central)
        self.statusBar().showMessage("시나리오를 편집한 뒤 ‘변경사항 빌드 후 RMF 분석’을 누르세요")

    @staticmethod
    def _splitter_total(splitter: QSplitter, fallback: int) -> int:
        total = sum(splitter.sizes())
        return total if total > 0 else fallback

    def focus_map_layout(self) -> None:
        """Give the map most of the workspace while keeping editors usable."""
        horizontal = self._splitter_total(
            self.main_splitter, max(self.width() - 30, 1200))
        editor_width = min(450, max(410, horizontal // 4))
        self.main_splitter.setSizes(
            [max(700, horizontal - editor_width), editor_width])
        vertical = self._splitter_total(
            self.vertical_splitter, max(self.height() - 120, 700))
        current = self.vertical_splitter.sizes()
        if len(current) > 1 and current[1] > 0:
            self.last_output_height = current[1]
        self.vertical_splitter.setSizes([vertical, 0])
        self.statusBar().showMessage(
            "지도 최대 배치 · 하단 결과를 완전히 접었습니다. 오른쪽 판단은 유지됩니다",
            4000,
        )

    def toggle_output_panel(self) -> None:
        total = self._splitter_total(
            self.vertical_splitter, max(self.height() - 120, 700))
        current = self.vertical_splitter.sizes()
        if len(current) < 2:
            return
        if current[1] > 110:
            self.last_output_height = current[1]
            self.vertical_splitter.setSizes([total, 0])
            self.statusBar().showMessage(
                "하단 결과를 접었습니다 · 같은 버튼이나 분할선을 사용해 다시 펼칠 수 있습니다",
                3500,
            )
        else:
            output_height = max(220, min(self.last_output_height, int(total * 0.45)))
            self.vertical_splitter.setSizes([total - output_height, output_height])
            self.statusBar().showMessage("하단 결과를 펼쳤습니다", 2500)

    def reset_workspace_layout(self) -> None:
        """Restore a balanced, large-screen workspace split."""
        horizontal = self._splitter_total(
            self.main_splitter, max(self.width() - 30, 1200))
        editor_width = min(500, max(430, int(horizontal * 0.25)))
        self.main_splitter.setSizes(
            [max(700, horizontal - editor_width), editor_width])
        vertical = self._splitter_total(
            self.vertical_splitter, max(self.height() - 120, 700))
        map_height = max(390, int(vertical * 0.68))
        self.vertical_splitter.setSizes(
            [map_height, max(220, vertical - map_height)])
        self.statusBar().showMessage("기본 작업 배치로 복원했습니다", 3000)

    def toggle_window_size(self) -> None:
        if self.isMaximized():
            self.showNormal()
            if self.width() < 1500 or self.height() < 900:
                self.resize(1700, 1000)
            self.statusBar().showMessage(
                "일반 창 · 테두리를 드래그해 원하는 크기로 조절하세요", 3500)
        else:
            self.showMaximized()
            self.statusBar().showMessage("창을 최대화했습니다", 2500)

    def restore_workspace_layout(self) -> None:
        geometry = self.settings.value("window/geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        main_state = self.settings.value("splitter/map_editor_v2")
        if main_state is not None:
            self.main_splitter.restoreState(main_state)
        vertical_state = self.settings.value("splitter/map_output_v2")
        if vertical_state is not None:
            self.vertical_splitter.restoreState(vertical_state)
        self.start_maximized = self.settings.value(
            "window/maximized", True, type=bool)

    @staticmethod
    def make_table(headers: list[str]) -> CopyableTableWidget:
        table = CopyableTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        table.verticalHeader().setDefaultSectionSize(25)
        table.verticalHeader().setVisible(False)
        table.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        table.setMinimumHeight(0)
        table.setAlternatingRowColors(True)
        return table

    def copy_table(self, table: CopyableTableWidget, all_rows: bool) -> None:
        copied = table.copy_all() if all_rows else table.copy_selection()
        self.statusBar().showMessage(
            "표 전체를 TSV로 복사했습니다" if copied and all_rows else
            "선택 셀을 TSV로 복사했습니다" if copied else
            "복사할 셀을 먼저 선택하세요",
            3500,
        )

    def copy_current_schedule(self, all_rows: bool) -> None:
        current = self.schedule_tabs.currentWidget()
        if isinstance(current, CopyableTableWidget):
            self.copy_table(current, all_rows)
            return
        if isinstance(current, QTextEdit):
            QApplication.clipboard().setText(current.toPlainText())
            self.statusBar().showMessage("Schedule DB 해석 가이드를 복사했습니다", 3500)

    def copy_current_object_table(self, all_rows: bool) -> None:
        current = self.object_tabs.currentWidget()
        if isinstance(current, CopyableTableWidget):
            self.copy_table(current, all_rows)

    def show_object_row_explanation(self, table: QTableWidget) -> None:
        row = table.currentRow()
        events = self.object_event_rows.get(table, [])
        if 0 <= row < len(events):
            self.object_explanation.setPlainText(explain_event(events[row]))

    def show_schedule_row_explanation(self, table: QTableWidget) -> None:
        row = table.currentRow()
        events = self.schedule_event_rows.get(table, [])
        if 0 <= row < len(events):
            self.schedule_explanation.setPlainText(explain_event(events[row]))

    def show_astar_row_explanation(self) -> None:
        row = self.astar_table.currentRow()
        if 0 <= row < len(self.astar_event_rows):
            self.astar_explanation.setPlainText(explain_event(self.astar_event_rows[row]))
        else:
            self.astar_explanation.setPlainText(
                "왼쪽 A* 행을 선택하면 실제 g/h/f 근거를 표시합니다.")

    def show_policy_astar_explanation(self) -> None:
        row = self.policy_astar_table.currentRow()
        if not 0 <= row < len(getattr(self, "corridor_decision_events", [])):
            return
        event = self.corridor_decision_events[row]
        overlaps = event.get("overlaps", [])
        overlap_lines = []
        for overlap in overlaps:
            overlap_lines.append(
                "- participant={participant_id}, plan={plan_id}, route={route_id}, "
                "방향={direction}({relation}), 점유={occupancy_enter}~{occupancy_exit}s, "
                "실제겹침={overlap_duration}s, admission창겹침="
                "{admission_overlap_duration}s, state={state}, source={source}".format(
                    **{key: overlap.get(key, "") for key in (
                        "participant_id", "plan_id", "route_id", "direction",
                        "relation", "occupancy_enter", "occupancy_exit",
                        "overlap_duration", "admission_overlap_duration",
                        "state", "source")}
                ))
        decision = str(event.get("decision", ""))
        if decision == "HARD_CORRIDOR_BLOCK":
            conclusion = (
                "Corridor Admission의 HARD POLICY BLOCK입니다. 기존 RMF validator가 "
                "막은 것으로 표시하지 않습니다.")
        elif decision == "SOFT_PENALIZED":
            conclusion = (
                "child는 제거되지 않았습니다. policy cost가 g에만 더해져 다른 후보보다 "
                "순위가 낮아질 수 있습니다.")
        else:
            conclusion = "Corridor policy는 이 child를 허용했고 custom cost도 없습니다."
        candidate_lanes = [int(value) for value in event.get("lane_ids", [])]
        participant_name = next((
            item.get("name") for item in reversed(self.events)
            if item.get("event") == "schedule_participant"
            and item.get("participant_id") == event.get("participant_id")
        ), "")
        chosen_lanes: list[int] = []
        for item in reversed(self.events):
            if item.get("event") == "plan_summary" and (
                not participant_name or item.get("robot") == participant_name
            ):
                chosen_lanes = [int(value) for value in item.get("used_lanes", [])]
                if chosen_lanes:
                    break
        selected_text = (
            "최종 Plan의 used_lanes와 겹칩니다."
            if candidate_lanes and set(candidate_lanes) & set(chosen_lanes)
            else "최종 Plan의 used_lanes에는 포함되지 않았습니다."
            if chosen_lanes else
            "최종 Plan lane 정보가 아직 없어 선택 여부를 확정할 수 없습니다."
        )
        self.astar_explanation.setPlainText(
            f"선택 Candidate #{event.get('candidate_id', event.get('seq'))} "
            f"(parent #{event.get('parent_id')})\n\n"
            f"1. 노드/Lane: waypoint {event.get('current_waypoint')} → "
            f"{event.get('target_waypoint')}, lanes={candidate_lanes}\n"
            f"2. 고려한 물리 통로: {event.get('corridor_id')} / 방향 {event.get('direction')}\n"
            f"3. 예상 통과 시간: {event.get('predicted_enter_time')}~"
            f"{event.get('predicted_exit_time')} s "
            f"({event.get('interval_basis')})\n"
            f"4. RMF 원래 비용: parent_g={event.get('parent_g')}, "
            f"approach={event.get('approach_cost')}, "
            f"RMF alt={event.get('rmf_core_alt_cost')}. "
            f"trajectory 분해는 "
            f"move={event.get('base_move_cost')}, rotation={event.get('rotation_cost')}, "
            f"event={event.get('event_cost')}, wait={event.get('wait_cost')}\n"
            f"5. Soft 비용: static={event.get('static_penalty')}, "
            f"same={event.get('same_direction_penalty')}, "
            f"opposite={event.get('opposite_direction_penalty')}, "
            f"occupancy={event.get('corridor_occupancy_penalty')}, "
            f"no_escape={event.get('no_escape_penalty')}, "
            f"합계={event.get('total_policy_penalty')}\n"
            f"6. 최종 ranking 값: new_g={event.get('final_g')}, "
            f"h={event.get('h')} (dynamic penalty 미포함), f={event.get('f')}\n"
            f"7. 판정: {decision} / {event.get('reason_code')}\n"
            f"8. 해석: {conclusion}\n"
            f"9. 최종 경로 대조: {selected_text} chosen_lanes={chosen_lanes}\n\n"
            "참고한 Schedule participant/plan/route\n"
            + ("\n".join(overlap_lines) if overlap_lines else "- 시간 중첩 없음")
            + "\n\nSource 경계\n"
            f"- candidate cost/timestamp: {event.get('source')}\n"
            f"- exact g/h/f: {event.get('exact_cost_source')}\n"
            f"- move/rotation/wait 분해: {event.get('motion_breakdown_source')}\n"
            f"- Schedule row: {event.get('schedule_source')}\n"
            f"- corridor association/penalty: {event.get('analysis_source')}\n"
            f"- participant context: {event.get('participant_context_source')}\n"
            f"- cost detail: {event.get('cost_component_note')}"
        )

    def show_decision_row_explanation(self) -> None:
        row = self.decision_table.currentRow()
        if 0 <= row < len(self.decision_rows):
            self.decision_explanation.setPlainText(self.decision_rows[row]["detail"])

    def on_zoom_changed(self, percent: int) -> None:
        if hasattr(self, "zoom_label"):
            self.zoom_label.setText(f"{percent}%")

    def show_usage(self) -> None:
        QMessageBox.information(
            self,
            "RMF Traffic Simulator 사용법",
            "1. 상단 시나리오에서 기준 맵을 선택합니다.\n\n"
            "2. 노드는 마우스로 드래그합니다. 새 노드는 ‘노드 추가’를 누릅니다.\n"
            "   Lane은 Ctrl+클릭으로 노드 2개를 선택한 뒤 ‘양방향 Lane’을 누릅니다.\n\n"
            "3. 오른쪽 속성 탭에서 holding, mutex, speed limit, lane 폐쇄를 바꾸고\n"
            "   ‘선택 항목에 적용’을 누릅니다. Corridor 탭에서는 1>2,2>3 형식으로\n"
            "   한 물리 통로의 정/역방향 edge를 묶고, Delay/Replan 탭에서 runtime event를 편집합니다.\n\n"
            "4. 로봇 탭에서 name, start/goal, yaw, 계획 시작과 동적 투입 시각을 입력합니다.\n"
            "   계획 시작만 8초면 처음부터 알려진 지연 출발입니다. 동적 투입도 8초이면\n"
            "   앞 stage가 DB에 commit된 뒤 그 로봇 participant를 실제로 등록합니다.\n"
            "   ‘신규 로봇 동적 투입’은 현재 마지막 투입보다 5초 뒤 행을 추가합니다.\n\n"
            "5. setup.bash 경로를 확인합니다. 최초 실행이나 C++/RMF 소스 변경 뒤에는\n"
            "   ‘변경사항 다시 빌드’를 체크하고 ‘변경사항 빌드 후 RMF 분석’을 누릅니다.\n"
            "   실행 파일이 최신이면 체크를 끄고 ‘빌드된 RMF로 계획 분석’을 누릅니다.\n\n"
            "6. 실행 중 ‘실행 로그 요약’과 ‘JSONL 요약’에서 주요 값을 확인합니다.\n"
            "   Schedule DB·A*·스텝별 판단 근거 표는 Ctrl+C로 선택 복사,\n"
            "   Ctrl+Shift+C로 전체 표를 복사할 수 있습니다.\n\n"
            "   ‘RMF 객체·협상 원문’ 탭에서는 실제 Graph/Supergraph 노출 범위,\n"
            "   Start·Goal·Validator, Plan의 Itinerary·Route·Trajectory, Proposal과\n"
            "   CentralizedNegotiation raw log의 Submit·Reject·Forfeit 과정을 확인합니다.\n"
            "   Reject/Forfeit 분류는 원문을 보존한 상태의 가독성용 분류입니다.\n\n"
            "7. 실행이 끝나면 하단 ‘계획 재생’을 눌러 실제 RMF trajectory를 봅니다.\n"
            "   배속은 0.25x~8x이며 화면 재생 속도만 바꾸고 RMF 궤적의 원래 시각은 유지합니다.\n"
            "   로봇 앞쪽 흰색 노즈가 yaw 방향이며 이동·회전·대기·도착 상태가 표시됩니다.\n"
            "   후진은 금지되며 반대 방향은 제자리 회전 후 전진합니다. 재생 시간은\n"
            "   스텝별 판단 근거의 plan waypoint와 자동 동기화됩니다.\n\n"
            "8. 상단 모드는 BASELINE/SOFT/HYBRID/HYBRID+NEGO입니다. 수정 모드는\n"
            "   ‘After 코어 준비’를 먼저 누릅니다. SOFT는 Schedule-aware g-cost만,\n"
            "   HYBRID는 반대 방향 새 ENTRY admission까지, HYBRID+NEGO는 동적 신규 batch\n"
            "   협상 정책까지 포함합니다. 네 결과는 서로 다른 build/JSONL에 저장됩니다.\n\n"
            "화면은 휠로 확대/축소하고 가운데 버튼 드래그로 이동합니다.\n"
            "지도-속성, 지도-로그 사이의 파란 분할선을 드래그하면 각 영역 크기가 바뀝니다.\n"
            "‘지도 넓게’는 오른쪽과 하단을 줄이고, ‘하단 결과 접기/펼치기’는\n"
            "로그 전체를 접어 지도를 창 아래까지 확장합니다. 실시간 판단은 오른쪽의\n"
            "‘판단 접기/펼치기’ 버튼으로 별도 조절합니다. 하단 높이를 늘리면 표도 같이\n"
            "늘어나 더 많은 행이 보입니다.\n"
            "‘기본 배치’는 비율을 복원합니다.\n"
            "‘창 최대화/복원’ 또는 창 테두리 드래그로 전체 창 크기도 조절할 수 있습니다.\n\n"
            "중요: 맵 편집과 화면은 Python이지만 Planner, Negotiation, Schedule DB,\n"
            "DetectConflict는 실제 C++ rmf_traffic을 호출합니다.",
        )

    def _make_editor_panel(self) -> QWidget:
        container = QWidget()
        container.setMinimumHeight(0)
        container.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Ignored)
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(0, 0, 0, 0)
        live_header = QHBoxLayout()
        live_title = QLabel("실시간 RMF 판단")
        live_title.setObjectName("sectionTitle")
        live_header.addWidget(live_title)
        live_header.addStretch()
        self.live_decision_toggle = QPushButton("판단 접기")
        self.live_decision_toggle.setToolTip(
            "도달 목표·이동 근거·경로·협상·Schedule DB 실시간 설명을 열거나 닫습니다.")
        self.live_decision_toggle.clicked.connect(self.toggle_live_decision_panel)
        live_header.addWidget(self.live_decision_toggle)
        container_layout.addLayout(live_header)
        self.current_decision_label = QLabel(
            "실행 후 재생하면 도달 목표·이동 근거·경로 근거·협상·Schedule DB가 표시됩니다.")
        self.current_decision_label.setObjectName("liveDecision")
        self.current_decision_label.setWordWrap(True)
        self.current_decision_label.setMinimumHeight(205)
        self.current_decision_label.setMaximumHeight(285)
        self.current_decision_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        container_layout.addWidget(self.current_decision_label)
        live_collapsed = self.settings.value(
            "panels/live_decision_collapsed", False, type=bool)
        self._set_live_decision_visible(not live_collapsed)

        tabs = QTabWidget()
        tabs.setMinimumHeight(0)
        tabs.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Ignored)
        tabs.setUsesScrollButtons(True)
        tabs.setElideMode(Qt.TextElideMode.ElideNone)
        props = QWidget(); form = QFormLayout(props)
        self.selection_label = QLabel("노드 또는 lane을 선택하세요")
        form.addRow(self.selection_label)
        self.name_edit = QLineEdit()
        self.x_spin = QDoubleSpinBox(); self.y_spin = QDoubleSpinBox()
        for spin in (self.x_spin, self.y_spin):
            spin.setRange(-1000, 1000); spin.setDecimals(3)
        self.holding_check = QCheckBox(); self.parking_check = QCheckBox()
        self.passthrough_check = QCheckBox(); self.mutex_edit = QLineEdit()
        self.bidirectional_check = QCheckBox(); self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0, 100); self.speed_spin.setDecimals(3)
        self.speed_spin.setSpecialValueText("제한 없음")
        self.closed_check = QCheckBox()
        self.after_penalty_spin = QDoubleSpinBox()
        self.after_penalty_spin.setRange(0, 1000000)
        self.after_penalty_spin.setDecimals(3)
        self.after_penalty_spin.setSingleStep(10.0)
        self.after_penalty_spin.setSpecialValueText("사용 안 함")
        self.after_penalty_spin.setToolTip(
            "AFTER 수동 모드에서 이 원본 Lane의 양방향 directed lane g-cost에 더할 값입니다.")
        form.addRow("이름", self.name_edit); form.addRow("X", self.x_spin); form.addRow("Y", self.y_spin)
        form.addRow("Holding", self.holding_check); form.addRow("Parking", self.parking_check)
        form.addRow("Passthrough", self.passthrough_check); form.addRow("Mutex group", self.mutex_edit)
        form.addRow("양방향", self.bidirectional_check); form.addRow("Speed limit", self.speed_spin)
        form.addRow("Lane 폐쇄", self.closed_check)
        form.addRow("AFTER 우회 penalty", self.after_penalty_spin)
        apply_button = QPushButton("선택 항목에 적용")
        apply_button.clicked.connect(self.apply_properties)
        form.addRow(apply_button)
        tabs.addTab(props, "노드/Lane")

        robots = QWidget(); robot_layout = QVBoxLayout(robots)
        self.robot_table = QTableWidget(0, 6)
        self.robot_table.setHorizontalHeaderLabels(
            ["name", "start node", "goal node", "yaw(rad)", "계획 시작(s)", "동적 투입(s)"])
        self.robot_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        robot_layout.addWidget(self.robot_table)
        robot_buttons = QHBoxLayout()
        add_robot = QPushButton("로봇 추가"); add_robot.clicked.connect(self.add_robot)
        add_dynamic_robot = QPushButton("신규 로봇 동적 투입")
        add_dynamic_robot.setObjectName("accentButton")
        add_dynamic_robot.clicked.connect(self.add_dynamic_robot)
        delete_robot = QPushButton("선택 로봇 삭제"); delete_robot.clicked.connect(self.delete_robot)
        robot_buttons.addWidget(add_robot); robot_buttons.addWidget(add_dynamic_robot)
        robot_buttons.addWidget(delete_robot)
        robot_layout.addLayout(robot_buttons)
        robot_layout.addWidget(QLabel(
            "계획 시작은 궤적의 earliest time, 동적 투입은 Schedule DB에 참가자를 실제로 "
            "등록하는 시각입니다. 동적 투입이 0보다 크면 기존 itinerary를 유지한 채 신규 "
            "로봇만 협상합니다. 시작 노드는 병목 밖 staging/parking을 권장합니다."))
        tabs.addTab(robots, "로봇")

        corridors = QWidget(); corridor_editor_layout = QVBoxLayout(corridors)
        self.corridor_editor_table = QTableWidget(0, 9)
        self.corridor_editor_table.setHorizontalHeaderLabels([
            "id", "forward edges", "reverse edges", "capacity", "passing",
            "hard opposite", "entry A", "entry B", "base penalty",
        ])
        self.corridor_editor_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents)
        self.corridor_editor_table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeMode.Stretch)
        self.corridor_editor_table.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeMode.Stretch)
        corridor_editor_layout.addWidget(self.corridor_editor_table)
        corridor_buttons = QHBoxLayout()
        add_corridor = QPushButton("선택 노드로 Corridor 추가")
        add_corridor.clicked.connect(self.add_corridor)
        apply_corridors = QPushButton("Corridor 표 적용")
        apply_corridors.setObjectName("accentButton")
        apply_corridors.clicked.connect(self.apply_corridor_table)
        delete_corridor = QPushButton("선택 Corridor 삭제")
        delete_corridor.clicked.connect(self.delete_corridor)
        corridor_buttons.addWidget(add_corridor)
        corridor_buttons.addWidget(apply_corridors)
        corridor_buttons.addWidget(delete_corridor)
        corridor_editor_layout.addLayout(corridor_buttons)
        corridor_help = QLabel(
            "edge 형식: 1>2,2>3. 한 물리 통로의 정방향/역방향 directed edge를 "
            "같은 ID로 묶습니다. passing=false·hard opposite=true이면 HYBRID에서 "
            "반대 방향의 새 ENTRY만 차단합니다.")
        corridor_help.setWordWrap(True)
        corridor_editor_layout.addWidget(corridor_help)
        tabs.addTab(corridors, "Corridor")

        runtime_events = QWidget(); runtime_layout = QVBoxLayout(runtime_events)
        self.runtime_event_table = QTableWidget(0, 7)
        self.runtime_event_table.setHorizontalHeaderLabels([
            "type", "robot", "at(s)", "value(s)", "corridor/reason",
            "trigger/confirmed", "source",
        ])
        self.runtime_event_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        runtime_layout.addWidget(self.runtime_event_table)
        runtime_buttons = QHBoxLayout()
        for label, callback in (
            ("Delay 추가", self.add_delay_event),
            ("통신 끊김 추가", self.add_communication_loss_event),
            ("Exit 확인 추가", self.add_checkpoint_release_event),
            ("선택 이벤트 삭제", self.delete_runtime_event),
            ("이벤트 표 적용", self.apply_runtime_event_table),
        ):
            button = QPushButton(label)
            button.clicked.connect(callback)
            runtime_buttons.addWidget(button)
        runtime_layout.addLayout(runtime_buttons)
        runtime_note = QLabel(
            "Delay는 가능한 RMF 빌드에서 실제 Participant::delay(Duration)를 호출합니다. "
            "통신 끊김/Exit 확인은 Fleet Adapter가 없는 실험기의 명시적 SIMULATION_EVENT이며, "
            "예상 exit 시각만으로 Corridor를 자동 FREE 처리하지 않습니다.")
        runtime_note.setWordWrap(True)
        runtime_layout.addWidget(runtime_note)
        tabs.addTab(runtime_events, "Delay / Replan")

        core = QWidget(); core_form = QFormLayout(core)
        self.before_setup_edit = QLineEdit("~/rmf_ws/install/setup.bash")
        self.before_source_edit = QLineEdit("~/rmf_ws/src/rmf_traffic")
        self.after_workspace_edit = QLineEdit("~/rmf_ws_modified")
        self.after_setup_edit = QLineEdit("~/rmf_ws_modified/install/setup.bash")
        self.after_source_edit = QLineEdit("~/rmf_ws_modified/src/rmf_traffic")
        self.schedule_soft_workspace_edit = QLineEdit("~/rmf_ws_schedule_soft")
        self.schedule_soft_setup_edit = QLineEdit("~/rmf_ws_schedule_soft/install/setup.bash")
        self.schedule_soft_source_edit = QLineEdit("~/rmf_ws_schedule_soft/src/rmf_traffic")
        self.schedule_soft_label_edit = QLineEdit("schedule_soft_baseline_derived")
        self.after_nego_workspace_edit = QLineEdit("~/rmf_ws_after_nego")
        self.after_nego_setup_edit = QLineEdit(
            "~/rmf_ws_after_nego/install/setup.bash")
        self.after_nego_source_edit = QLineEdit(
            "~/rmf_ws_after_nego/src/rmf_traffic")
        self.base_ros_setup_edit = QLineEdit("/opt/ros/jazzy/setup.bash")
        self.after_label_edit = QLineEdit("after_schedule_corridor_policy")
        self.after_nego_label_edit = QLineEdit("after_nego_newcomer_detour")
        self.rebuild_after_check = QCheckBox(
            "SOFT/HYBRID 실행 전에 rmf_traffic을 colcon build")
        self.rebuild_after_check.setChecked(True)
        self.rebuild_schedule_soft_check = QCheckBox(
            "SCHEDULE_SOFT 실행 전에 rmf_traffic을 colcon build")
        self.rebuild_schedule_soft_check.setChecked(True)
        self.rebuild_after_nego_check = QCheckBox(
            "HYBRID+NEGO 실행 전에 rmf_traffic을 colcon build")
        self.rebuild_after_nego_check.setChecked(True)
        self.after_penalty_mode_combo = QComboBox()
        self.after_penalty_mode_combo.addItem(
            "Legacy V2 · 예상 점유 로봇 수", "shared_corridor")
        self.after_penalty_mode_combo.addItem(
            "자동 우회 · 기존 최단경로 회피", "shortest_path")
        self.after_penalty_mode_combo.addItem(
            "수동 · Lane 속성의 AFTER penalty", "manual")
        self.after_penalty_mode_combo.addItem("사용 안 함 · 다른 코어 수정 실험", "off")
        self.after_penalty_value_spin = QDoubleSpinBox()
        self.after_penalty_value_spin.setRange(0.1, 100000.0)
        self.after_penalty_value_spin.setDecimals(1)
        self.after_penalty_value_spin.setSingleStep(10.0)
        self.after_penalty_value_spin.setValue(60.0)
        self.after_penalty_value_spin.setSuffix(" cost/lane")
        self.after_penalty_value_spin.setToolTip(
            "혼잡 모드는 예상경로가 겹치는 물리 통로의 수요가 1대를 넘을 때, "
            "초과 로봇 수 × 이 값을 실제 A* g에 더합니다. 우회 모드는 기존 "
            "최단경로 directed lane마다 이 값을 더합니다. 60부터 시작하세요.")
        self.policy_same_weight = QDoubleSpinBox()
        self.policy_opposite_weight = QDoubleSpinBox()
        self.policy_occupied_weight = QDoubleSpinBox()
        self.policy_future_weight = QDoubleSpinBox()
        self.policy_no_escape_weight = QDoubleSpinBox()
        self.policy_static_weight = QDoubleSpinBox()
        self.policy_overlap_margin = QDoubleSpinBox()
        self.schedule_soft_lambda = QDoubleSpinBox()
        self.schedule_soft_max_penalty = QDoubleSpinBox()
        self.schedule_soft_same_weight = QDoubleSpinBox()
        self.schedule_soft_opposite_weight = QDoubleSpinBox()
        self.schedule_soft_enabled = QCheckBox(
            "SCHEDULE_SOFT 활성 · OFF이면 lambda=0으로 Baseline 동일성 검증")
        self.schedule_soft_enabled.setChecked(True)
        policy_defaults = (
            (self.policy_same_weight, 0.25, " cost/s"),
            (self.policy_opposite_weight, 8.0, " cost/s"),
            (self.policy_occupied_weight, 1.5, " cost/s"),
            (self.policy_future_weight, 0.6, " cost/s"),
            (self.policy_no_escape_weight, 25.0, " cost"),
            (self.policy_static_weight, 0.0, " cost"),
            (self.policy_overlap_margin, 0.25, " s"),
            (self.schedule_soft_lambda, 0.25, " λ"),
            (self.schedule_soft_max_penalty, 10.0, " cost"),
            (self.schedule_soft_same_weight, 0.5, " weight"),
            (self.schedule_soft_opposite_weight, 1.5, " weight"),
        )
        for spin, value, suffix in policy_defaults:
            spin.setRange(0.0, 100000.0)
            spin.setDecimals(3)
            spin.setValue(value)
            spin.setSuffix(suffix)
        core_form.addRow("BASELINE setup", self.before_setup_edit)
        core_form.addRow("BASELINE source", self.before_source_edit)
        core_form.addRow("OLD_SOFT/HYBRID workspace", self.after_workspace_edit)
        core_form.addRow("OLD_SOFT/HYBRID setup", self.after_setup_edit)
        core_form.addRow("OLD_SOFT/HYBRID source", self.after_source_edit)
        core_form.addRow("SCHEDULE_SOFT workspace", self.schedule_soft_workspace_edit)
        core_form.addRow("SCHEDULE_SOFT setup", self.schedule_soft_setup_edit)
        core_form.addRow("SCHEDULE_SOFT source", self.schedule_soft_source_edit)
        core_form.addRow("SCHEDULE_SOFT label", self.schedule_soft_label_edit)
        core_form.addRow("HYBRID+NEGO workspace", self.after_nego_workspace_edit)
        core_form.addRow("HYBRID+NEGO setup", self.after_nego_setup_edit)
        core_form.addRow("HYBRID+NEGO source", self.after_nego_source_edit)
        core_form.addRow("Base ROS setup", self.base_ros_setup_edit)
        core_form.addRow("OLD_SOFT/HYBRID label", self.after_label_edit)
        core_form.addRow("HYBRID+NEGO label", self.after_nego_label_edit)
        core_form.addRow("Legacy V2 우회", self.after_penalty_mode_combo)
        core_form.addRow("Legacy/newcomer penalty", self.after_penalty_value_spin)
        core_form.addRow("같은 방향 overlap", self.policy_same_weight)
        core_form.addRow("반대 방향 overlap", self.policy_opposite_weight)
        core_form.addRow("현재 점유 overlap", self.policy_occupied_weight)
        core_form.addRow("미래 예약 overlap", self.policy_future_weight)
        core_form.addRow("회피공간 없음", self.policy_no_escape_weight)
        core_form.addRow("Corridor static", self.policy_static_weight)
        core_form.addRow("시간 overlap margin", self.policy_overlap_margin)
        core_form.addRow(self.schedule_soft_enabled)
        core_form.addRow("SCHEDULE_SOFT lambda", self.schedule_soft_lambda)
        core_form.addRow("SCHEDULE_SOFT max penalty", self.schedule_soft_max_penalty)
        core_form.addRow("SCHEDULE_SOFT same weight", self.schedule_soft_same_weight)
        core_form.addRow("SCHEDULE_SOFT opposite weight", self.schedule_soft_opposite_weight)
        penalty_note = QLabel(
            "위 Legacy V2 항목은 이전 실험 재현용입니다. 현재 SOFT/HYBRID는 아래 "
            "같은/반대 방향·점유·미래예약 weight로 실제 Schedule snapshot과 candidate "
            "trajectory 시간 중첩을 계산해 DifferentialDrivePlanner의 A* g에만 더합니다.")
        penalty_note.setWordWrap(True)
        core_form.addRow(penalty_note)
        core_form.addRow(self.rebuild_after_check)
        core_form.addRow(self.rebuild_schedule_soft_check)
        core_form.addRow(self.rebuild_after_nego_check)
        prepare_after = QPushButton("Schedule-aware SOFT/HYBRID 코어 준비")
        prepare_after.setObjectName("accentButton")
        prepare_after.setMinimumHeight(38)
        prepare_after.setToolTip(
            "원본 rmf_traffic을 격리 workspace로 복사하고 실제 Schedule-aware "
            "Corridor policy hook을 준비합니다.")
        prepare_after.clicked.connect(self.prepare_after_core_from_ui)
        prepare_schedule_soft = QPushButton("SCHEDULE_SOFT 코어 준비 (BASELINE에서 분기)")
        prepare_schedule_soft.setObjectName("accentButton")
        prepare_schedule_soft.setMinimumHeight(38)
        prepare_schedule_soft.setToolTip(
            "BASELINE rmf_traffic source에서 독립 workspace를 만들고 Schedule DB 실제 interval 기반 "
            "bounded soft cost만 활성화합니다.")
        prepare_schedule_soft.clicked.connect(self.prepare_schedule_soft_core_from_ui)
        prepare_after_nego = QPushButton("HYBRID + NEGO 코어 준비")
        prepare_after_nego.setObjectName("accentButton")
        prepare_after_nego.setMinimumHeight(38)
        prepare_after_nego.setToolTip(
            "원본 rmf_traffic을 별도 workspace로 복사하고 Schedule-aware Corridor "
            "policy와 newcomer-only negotiation 비교 환경을 준비합니다.")
        prepare_after_nego.clicked.connect(self.prepare_after_nego_core_from_ui)
        apply_core = QPushButton("선택한 코어 경로 적용")
        apply_core.clicked.connect(self.core_profile_changed)
        compare_refresh = QPushButton("5-Mode 결과 다시 읽기")
        compare_refresh.clicked.connect(self.update_comparison)
        core_form.addRow(apply_core)
        core_form.addRow(prepare_after)
        core_form.addRow(prepare_schedule_soft)
        core_form.addRow(prepare_after_nego)
        core_form.addRow(compare_refresh)
        core_form.addRow(QLabel(
            "정확한 비교를 위해 BASELINE과 수정 코어 install을 분리하세요. "
            "SOFT/HYBRID는 같은 수정 코어에서 policy mode만 바꿀 수 있습니다."
        ))
        core_scroll = QScrollArea()
        core_scroll.setWidgetResizable(True)
        core_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        core_scroll.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        core_scroll.setWidget(core)
        tabs.addTab(core_scroll, "RMF Core")
        container_layout.addWidget(tabs, 1)
        return container

    def _set_live_decision_visible(self, visible: bool) -> None:
        self.current_decision_label.setVisible(visible)
        self.live_decision_toggle.setText("판단 접기" if visible else "판단 펼치기")

    def toggle_live_decision_panel(self) -> None:
        visible = not self.current_decision_label.isVisible()
        self._set_live_decision_visible(visible)
        self.settings.setValue("panels/live_decision_collapsed", not visible)
        self.statusBar().showMessage(
            "실시간 RMF 판단을 펼쳤습니다" if visible else "실시간 RMF 판단을 접었습니다",
            2500,
        )

    def prepare_after_core_from_ui(self) -> None:
        try:
            result = install_after_lane_penalty_core(
                Path(self.before_source_edit.text().strip()),
                Path(self.after_workspace_edit.text().strip()),
            )
        except (OSError, RuntimeError, ValueError) as exc:
            QMessageBox.critical(self, "Schedule-aware 코어 준비 실패", str(exc))
            return

        self.after_source_edit.setText(str(result["after_source"]))
        after_setup = Path(result["after_workspace"]) / "install" / "setup.bash"
        self.after_setup_edit.setText(str(after_setup))
        self.after_label_edit.setText("after_schedule_corridor_policy")
        self.core_profile_combo.setCurrentIndex(1)
        self.rebuild_after_check.setChecked(True)
        self.build_check.setChecked(True)
        action = "새로 복사하고 패치했습니다" if result["copied"] else (
            "기존 소스에 패치했습니다" if result["patched"] else
            "이미 패치되어 있어 그대로 사용합니다")
        QMessageBox.information(
            self,
            "Schedule-aware 코어 준비 완료",
            f"{action}.\n\n"
            f"수정 파일:\n{result['planner']}\n{result['negotiator']}\n"
            f"{result['policy_header']}\n\n"
            "이제 SOFT 또는 HYBRID를 선택한 상태에서 실행하세요. "
            "rmf_traffic colcon build 후 수정 라이브러리로 시나리오를 분석합니다.",
        )

    def prepare_schedule_soft_core_from_ui(self) -> None:
        try:
            result = prepare_schedule_soft_core(
                Path(self.before_source_edit.text().strip()),
                Path(self.schedule_soft_workspace_edit.text().strip()),
            )
        except (OSError, RuntimeError, ValueError) as exc:
            QMessageBox.critical(self, "SCHEDULE_SOFT 코어 준비 실패", str(exc))
            return
        self.schedule_soft_source_edit.setText(str(result["after_source"]))
        setup = Path(result["after_workspace"]) / "install" / "setup.bash"
        self.schedule_soft_setup_edit.setText(str(setup))
        self.schedule_soft_label_edit.setText("schedule_soft_baseline_derived")
        self.core_profile_combo.setCurrentIndex(2)
        self.rebuild_schedule_soft_check.setChecked(True)
        self.build_check.setChecked(True)
        QMessageBox.information(
            self, "SCHEDULE_SOFT 코어 준비 완료",
            "BASELINE source에서 독립 workspace를 준비했습니다.\n\n"
            f"Planner: {result['planner']}\n"
            f"Policy header: {result['policy_header']}\n\n"
            "OLD_SOFT의 POLICY_DERIVED/static/no_escape 비용은 schedule_soft 모드에서 사용하지 않습니다.")

    def prepare_after_nego_core_from_ui(self) -> None:
        try:
            result = prepare_after_nego_core(
                Path(self.before_source_edit.text().strip()),
                Path(self.after_nego_workspace_edit.text().strip()),
            )
        except (OSError, RuntimeError, ValueError) as exc:
            QMessageBox.critical(self, "HYBRID+NEGO 코어 준비 실패", str(exc))
            return

        self.after_nego_source_edit.setText(str(result["after_source"]))
        setup = Path(result["after_workspace"]) / "install" / "setup.bash"
        self.after_nego_setup_edit.setText(str(setup))
        self.after_nego_label_edit.setText("after_nego_newcomer_detour")
        self.core_profile_combo.setCurrentIndex(4)
        self.rebuild_after_nego_check.setChecked(True)
        self.build_check.setChecked(True)
        action = "새로 복사하고 패치했습니다" if result["copied"] else (
            "기존 소스에 패치했습니다" if result["patched"] else
            "이미 실제 A* g-cost 패치가 있어 그대로 사용합니다")
        QMessageBox.information(
            self,
            "HYBRID+NEGO 코어 준비 완료",
            f"{action}.\n\n수정 파일:\n{result['planner']}\n"
            f"{result.get('negotiator', '')}\n{result.get('policy_header', '')}\n\n"
            "동적 시나리오에서는 하나의 실제 Schedule Database를 유지하고, "
            "이미 commit된 로봇은 고정한 채 신규 투입 로봇만 협상합니다. "
            "신규 로봇 A*에는 기존 plan이 사용한 통로와 mutex group의 soft penalty가 들어갑니다.",
        )

    def core_profile_changed(self, *_args) -> None:
        if not hasattr(self, "before_setup_edit"):
            return
        profile = self.core_profile_combo.currentData()
        setup = (
            self.before_setup_edit.text() if profile == "baseline" else
            self.schedule_soft_setup_edit.text() if profile == "schedule_soft" else
            self.after_setup_edit.text() if profile in {"soft", "hybrid"} else
            self.after_nego_setup_edit.text())
        self.setup_edit.setText(setup)
        self.build_check.setChecked(True)
        if hasattr(self, "core_status_label"):
            self.core_status_label.setText(
                "BASELINE · Stock RMF" if profile == "baseline" else
                "OLD_SOFT · 기존 Schedule-aware g" if profile == "soft" else
                "SCHEDULE_SOFT · 최신 Snapshot overlap" if profile == "schedule_soft" else
                "HYBRID · Soft + Hard admission" if profile == "hybrid" else
                "HYBRID + NEGO · 신규협상 포함")
            self.core_status_label.setStyleSheet(
                "background:#dbeafe; border:1px solid #60a5fa; color:#1e3a8a;"
                if profile == "baseline" else
                "background:#ffedd5; border:1px solid #fb923c; color:#9a3412;"
                if profile == "soft" else
                "background:#e0f2fe; border:1px solid #38bdf8; color:#075985;"
                if profile == "schedule_soft" else
                "background:#fef3c7; border:1px solid #f59e0b; color:#92400e;"
                if profile == "hybrid" else
                "background:#dcfce7; border:1px solid #4ade80; color:#166534;")
        self.statusBar().showMessage(
            "BASELINE/OLD_SOFT/SCHEDULE_SOFT/HYBRID/HYBRID+NEGO 결과를 서로 다른 JSONL로 저장합니다"
        )

    def load_named_scenario(self, name: str) -> None:
        if name not in self.templates:
            return
        self.load_document(copy.deepcopy(self.templates[name]))

    def load_document(self, document: dict) -> None:
        self.document = document
        self.document.setdefault("corridors", [])
        self.document.setdefault("runtime_events", [])
        self.scene.load_document(self.document)
        self.refresh_robot_table()
        self.refresh_corridor_table()
        self.refresh_runtime_event_table()
        self.refresh_summary()
        self.show_robot_starts()
        QTimer.singleShot(0, self.fit_map)

    def show_robot_starts(self) -> None:
        self.scene.clear_robot_overlay()
        nodes = self.document.get("nodes", [])
        for index, robot in enumerate(self.document.get("robots", [])):
            start = robot.get("start", -1)
            if not isinstance(start, int) or not 0 <= start < len(nodes):
                continue
            node = nodes[start]
            start_time_s = float(robot.get("start_time_s", robot.get("start_time", 0.0)) or 0.0)
            insertion_time_s = float(robot.get("insertion_time_s", 0.0) or 0.0)
            state = (
                f"신규 투입 예약 t={insertion_time_s:g}s"
                if insertion_time_s > 0 else
                f"출발 예약 t={start_time_s:g}s" if start_time_s > 0 else "초기 참가자")
            self.scene.set_robot_pose(
                str(robot.get("name", f"R{index}")), float(node["x"]), float(node["y"]),
                float(robot.get("yaw", 0.0)), state,
                ROBOT_COLORS[index % len(ROBOT_COLORS)],
            )

    def refresh_summary(self) -> None:
        start_times = [
            float(robot.get("start_time_s", robot.get("start_time", 0.0)) or 0.0)
            for robot in self.document.get("robots", [])
        ]
        departure = (
            f" · 출발 {min(start_times):g}~{max(start_times):g}s"
            if start_times else "")
        insertion_times = [
            float(robot.get("insertion_time_s", 0.0) or 0.0)
            for robot in self.document.get("robots", [])
        ]
        dynamic = (
            f" · 동적 투입 {sum(value > 0 for value in insertion_times)}대"
            if any(value > 0 for value in insertion_times) else "")
        penalty_count = sum(
            1 for lane in self.document.get("lanes", [])
            if float(lane.get("after_penalty", 0) or 0) > 0)
        penalty = f" · 수동 penalty Lane {penalty_count}" if penalty_count else ""
        self.graph_summary.setText(
            f"노드 {len(self.document.get('nodes', []))} · 원본 lane {len(self.document.get('lanes', []))} "
            f"· 로봇 {len(self.document.get('robots', []))} "
            f"· Corridor {len(self.document.get('corridors', []))} "
            f"· Runtime event {len(self.document.get('runtime_events', []))}"
            f"{departure}{dynamic}{penalty}")

    def refresh_robot_table(self) -> None:
        robots = self.document.get("robots", [])
        self.robot_table.setRowCount(len(robots))
        for row, robot in enumerate(robots):
            values = (
                robot.get("name", f"R{row}"), robot.get("start", 0),
                robot.get("goal", 0), robot.get("yaw", 0.0),
                robot.get("start_time_s", robot.get("start_time", 0.0)),
                robot.get("insertion_time_s", robot.get("dispatch_time_s", 0.0)),
            )
            for column, value in enumerate(values):
                self.robot_table.setItem(row, column, QTableWidgetItem(str(value)))

    def sync_robots(self) -> None:
        robots = []
        for row in range(self.robot_table.rowCount()):
            try:
                start_time_s = float(self.robot_table.item(row, 4).text())
                if start_time_s < 0 or not math.isfinite(start_time_s):
                    raise ValueError("출발 시각은 0 이상의 유한한 초 값이어야 합니다")
                insertion_time_s = float(self.robot_table.item(row, 5).text())
                if insertion_time_s < 0 or not math.isfinite(insertion_time_s):
                    raise ValueError("동적 투입 시각은 0 이상의 유한한 초 값이어야 합니다")
                robots.append({
                    "name": self.robot_table.item(row, 0).text().strip(),
                    "start": int(self.robot_table.item(row, 1).text()),
                    "goal": int(self.robot_table.item(row, 2).text()),
                    "yaw": float(self.robot_table.item(row, 3).text()),
                    "start_time_s": start_time_s,
                    "insertion_time_s": insertion_time_s,
                })
            except (AttributeError, ValueError) as exc:
                raise ValueError(f"로봇 표 {row + 1}행의 값을 확인하세요") from exc
        self.document["robots"] = robots
        self.document["dynamic_insertion"] = any(
            robot["insertion_time_s"] > 0 for robot in robots)

    @staticmethod
    def _table_text(table: QTableWidget, row: int, column: int) -> str:
        item = table.item(row, column)
        return item.text().strip() if item else ""

    @staticmethod
    def _parse_bool_text(value: str) -> bool:
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y", "on"}:
            return True
        if normalized in {"false", "0", "no", "n", "off"}:
            return False
        raise ValueError(f"boolean 값이 아닙니다: {value}")

    @staticmethod
    def _format_edges(edges: object) -> str:
        if not isinstance(edges, list):
            return ""
        return ",".join(
            f"{edge[0]}>{edge[1]}" for edge in edges
            if isinstance(edge, list) and len(edge) == 2)

    @staticmethod
    def _parse_edges(value: str) -> list[list[int]]:
        output: list[list[int]] = []
        for raw in value.split(","):
            token = raw.strip()
            if not token:
                continue
            fields = token.split(">")
            if len(fields) != 2:
                raise ValueError(f"edge는 1>2 형식이어야 합니다: {token}")
            output.append([int(fields[0].strip()), int(fields[1].strip())])
        if not output:
            raise ValueError("Corridor에는 forward edge가 하나 이상 필요합니다")
        return output

    def refresh_corridor_table(self) -> None:
        corridors = self.document.get("corridors", [])
        self.corridor_editor_table.setRowCount(len(corridors))
        for row, corridor in enumerate(corridors):
            values = (
                corridor.get("id", f"C{row + 1}"),
                self._format_edges(corridor.get("forward_edges", [])),
                self._format_edges(corridor.get("reverse_edges", [])),
                corridor.get("capacity", 1),
                str(bool(corridor.get("passing_allowed", False))).lower(),
                str(bool(corridor.get(
                    "hard_opposite_direction_block", True))).lower(),
                "" if corridor.get("holding_entry_a") is None
                else corridor.get("holding_entry_a"),
                "" if corridor.get("holding_entry_b") is None
                else corridor.get("holding_entry_b"),
                corridor.get("base_penalty", 0.0),
            )
            for column, value in enumerate(values):
                self.corridor_editor_table.setItem(
                    row, column, QTableWidgetItem(str(value)))

    def sync_corridors(self) -> None:
        corridors: list[dict] = []
        node_count = len(self.document.get("nodes", []))
        seen: set[str] = set()
        for row in range(self.corridor_editor_table.rowCount()):
            corridor_id = self._table_text(
                self.corridor_editor_table, row, 0)
            if not corridor_id or corridor_id in seen:
                raise ValueError(f"Corridor {row + 1}행 ID가 비었거나 중복입니다")
            seen.add(corridor_id)
            forward = self._parse_edges(self._table_text(
                self.corridor_editor_table, row, 1))
            reverse_text = self._table_text(self.corridor_editor_table, row, 2)
            reverse = self._parse_edges(reverse_text) if reverse_text else [
                [edge[1], edge[0]] for edge in reversed(forward)]
            for edge in forward + reverse:
                if min(edge) < 0 or max(edge) >= node_count:
                    raise ValueError(
                        f"Corridor {corridor_id} edge {edge}의 node가 범위를 벗어났습니다")
            entry_a_text = self._table_text(self.corridor_editor_table, row, 6)
            entry_b_text = self._table_text(self.corridor_editor_table, row, 7)
            corridors.append({
                "id": corridor_id,
                "forward_edges": forward,
                "reverse_edges": reverse,
                "capacity": int(self._table_text(
                    self.corridor_editor_table, row, 3) or "1"),
                "passing_allowed": self._parse_bool_text(self._table_text(
                    self.corridor_editor_table, row, 4) or "false"),
                "hard_opposite_direction_block": self._parse_bool_text(
                    self._table_text(self.corridor_editor_table, row, 5) or "true"),
                "holding_entry_a": int(entry_a_text) if entry_a_text else None,
                "holding_entry_b": int(entry_b_text) if entry_b_text else None,
                "base_penalty": float(self._table_text(
                    self.corridor_editor_table, row, 8) or "0"),
            })
        self.document["corridors"] = corridors

    def add_corridor(self) -> None:
        selected = self.scene.selected_node_indexes()
        if len(selected) != 2:
            QMessageBox.information(
                self, "Corridor 추가",
                "입구와 출구 노드 두 개를 Ctrl+클릭으로 선택하세요.")
            return
        row = self.corridor_editor_table.rowCount()
        self.corridor_editor_table.insertRow(row)
        a, b = selected
        defaults = (
            f"C{row + 1}", f"{a}>{b}", f"{b}>{a}", "1", "false",
            "true", str(a), str(b), "0.0")
        for column, value in enumerate(defaults):
            self.corridor_editor_table.setItem(
                row, column, QTableWidgetItem(value))
        self.apply_corridor_table()

    def delete_corridor(self) -> None:
        rows = sorted({
            index.row() for index in self.corridor_editor_table.selectedIndexes()
        }, reverse=True)
        for row in rows:
            self.corridor_editor_table.removeRow(row)
        self.apply_corridor_table()

    def apply_corridor_table(self) -> None:
        try:
            self.sync_corridors()
        except ValueError as exc:
            QMessageBox.critical(self, "Corridor 입력 오류", str(exc))
            return
        self.scene.refresh_corridor_overlay()
        self.refresh_summary()

    def refresh_runtime_event_table(self) -> None:
        events = self.document.get("runtime_events", [])
        self.runtime_event_table.setRowCount(len(events))
        for row, event in enumerate(events):
            event_type = str(event.get("type", "delay"))
            if event_type == "delay":
                value = event.get("delay_s", 0.0)
                detail = event.get("reason", "explicit_replan")
                flag = event.get("trigger_replan", True)
            elif event_type == "communication_loss":
                value = event.get("duration_s", 0.0)
                detail = ""
                flag = event.get("release_on_timeout", False)
            else:
                value = 0.0
                detail = event.get("corridor", "")
                flag = event.get("checkpoint_confirmed", True)
            values = (
                event_type, event.get("robot", ""), event.get("at_s", 0.0),
                value, detail, str(bool(flag)).lower(), "SIMULATION_EVENT",
            )
            for column, value in enumerate(values):
                self.runtime_event_table.setItem(
                    row, column, QTableWidgetItem(str(value)))

    def sync_runtime_events(self) -> None:
        events: list[dict] = []
        valid_robots = {
            str(robot.get("name", "")) for robot in self.document.get("robots", [])}
        valid_corridors = {
            str(corridor.get("id", "")) for corridor in self.document.get("corridors", [])}
        for row in range(self.runtime_event_table.rowCount()):
            event_type = self._table_text(self.runtime_event_table, row, 0)
            robot = self._table_text(self.runtime_event_table, row, 1)
            at_s = float(self._table_text(self.runtime_event_table, row, 2) or "0")
            value = float(self._table_text(self.runtime_event_table, row, 3) or "0")
            detail = self._table_text(self.runtime_event_table, row, 4)
            flag = self._parse_bool_text(
                self._table_text(self.runtime_event_table, row, 5) or "false")
            if robot not in valid_robots:
                raise ValueError(f"Runtime event {row + 1}행의 robot이 없습니다: {robot}")
            if at_s < 0 or value < 0:
                raise ValueError("Runtime event 시간은 0 이상이어야 합니다")
            if event_type == "delay":
                events.append({
                    "type": event_type, "robot": robot, "at_s": at_s,
                    "delay_s": value, "reason": detail or "explicit_replan",
                    "trigger_replan": flag,
                })
            elif event_type == "communication_loss":
                events.append({
                    "type": event_type, "robot": robot, "at_s": at_s,
                    "duration_s": value, "release_on_timeout": flag,
                })
            elif event_type == "checkpoint_release":
                if detail not in valid_corridors:
                    raise ValueError(
                        f"Runtime event {row + 1}행의 Corridor가 없습니다: {detail}")
                events.append({
                    "type": event_type, "robot": robot, "at_s": at_s,
                    "corridor": detail, "checkpoint_confirmed": flag,
                })
            else:
                raise ValueError(f"지원하지 않는 runtime event type: {event_type}")
        self.document["runtime_events"] = events

    def _append_runtime_event(self, event_type: str) -> None:
        robots = self.document.get("robots", [])
        if not robots:
            QMessageBox.information(self, "Runtime event", "로봇을 먼저 추가하세요.")
            return
        corridors = self.document.get("corridors", [])
        row = self.runtime_event_table.rowCount()
        self.runtime_event_table.insertRow(row)
        robot = str(robots[0].get("name", "R0"))
        if event_type == "delay":
            values = (event_type, robot, "10", "10", "maximum_delay_exceeded", "true", "SIMULATION_EVENT")
        elif event_type == "communication_loss":
            values = (event_type, robot, "10", "20", "", "false", "SIMULATION_EVENT")
        else:
            if not corridors:
                self.runtime_event_table.removeRow(row)
                QMessageBox.information(
                    self, "Exit 확인", "Corridor를 먼저 추가하세요.")
                return
            values = (
                event_type, robot, "20", "0", corridors[0].get("id", "C1"),
                "true", "SIMULATION_EVENT")
        for column, value in enumerate(values):
            self.runtime_event_table.setItem(
                row, column, QTableWidgetItem(str(value)))

    def add_delay_event(self) -> None:
        self._append_runtime_event("delay")

    def add_communication_loss_event(self) -> None:
        self._append_runtime_event("communication_loss")

    def add_checkpoint_release_event(self) -> None:
        self._append_runtime_event("checkpoint_release")

    def delete_runtime_event(self) -> None:
        rows = sorted({
            index.row() for index in self.runtime_event_table.selectedIndexes()
        }, reverse=True)
        for row in rows:
            self.runtime_event_table.removeRow(row)
        self.apply_runtime_event_table()

    def apply_runtime_event_table(self) -> None:
        try:
            self.sync_runtime_events()
        except ValueError as exc:
            QMessageBox.critical(self, "Runtime event 입력 오류", str(exc))
            return
        self.refresh_summary()

    def add_node(self) -> None:
        index = len(self.document["nodes"])
        self.document["nodes"].append({
            "name": f"N{index}", "x": index * 0.5, "y": 1.0,
            "holding": True, "parking": False, "passthrough": False,
        })
        self.scene.load_document(self.document); self.refresh_summary(); self.show_robot_starts()

    def add_lane(self) -> None:
        selected = self.scene.selected_node_indexes()
        if len(selected) != 2:
            QMessageBox.information(self, "Lane 추가", "연결할 노드 두 개를 Ctrl+클릭으로 선택하세요.")
            return
        self.document["lanes"].append({"from": selected[0], "to": selected[1], "bidirectional": True})
        self.scene.load_document(self.document); self.refresh_summary(); self.show_robot_starts()

    def delete_selected(self) -> None:
        lane_indexes = {i.index for i in self.scene.selectedItems() if isinstance(i, LaneItem)}
        node_indexes = {i.index for i in self.scene.selectedItems() if isinstance(i, NodeItem)}
        if lane_indexes:
            self.document["lanes"] = [lane for i, lane in enumerate(self.document["lanes"]) if i not in lane_indexes]
        if node_indexes:
            mapping = {}; new_nodes = []
            for old, node in enumerate(self.document["nodes"]):
                if old not in node_indexes:
                    mapping[old] = len(new_nodes); new_nodes.append(node)
            self.document["nodes"] = new_nodes
            self.document["lanes"] = [
                {**lane, "from": mapping[lane["from"]], "to": mapping[lane["to"]]}
                for lane in self.document["lanes"]
                if lane["from"] in mapping and lane["to"] in mapping
            ]
            self.document["robots"] = [
                robot for robot in self.document["robots"]
                if robot["start"] in mapping and robot["goal"] in mapping
            ]
            for robot in self.document["robots"]:
                robot["start"] = mapping[robot["start"]]; robot["goal"] = mapping[robot["goal"]]
            updated_corridors = []
            for corridor in self.document.get("corridors", []):
                forward = [
                    [mapping[edge[0]], mapping[edge[1]]]
                    for edge in corridor.get("forward_edges", [])
                    if len(edge) == 2 and edge[0] in mapping and edge[1] in mapping]
                reverse = [
                    [mapping[edge[0]], mapping[edge[1]]]
                    for edge in corridor.get("reverse_edges", [])
                    if len(edge) == 2 and edge[0] in mapping and edge[1] in mapping]
                if not forward:
                    continue
                updated = {**corridor, "forward_edges": forward, "reverse_edges": reverse}
                for field in ("holding_entry_a", "holding_entry_b"):
                    value = corridor.get(field)
                    updated[field] = mapping.get(value) if value is not None else None
                updated_corridors.append(updated)
            removed_corridors = {
                corridor.get("id") for corridor in self.document.get("corridors", [])
            } - {corridor.get("id") for corridor in updated_corridors}
            self.document["corridors"] = updated_corridors
            self.document["runtime_events"] = [
                event for event in self.document.get("runtime_events", [])
                if event.get("corridor") not in removed_corridors]
        self.scene.load_document(self.document)
        self.refresh_robot_table(); self.refresh_corridor_table()
        self.refresh_runtime_event_table(); self.refresh_summary(); self.show_robot_starts()

    def on_selection_changed(self) -> None:
        selected = self.scene.selectedItems()
        if len(selected) != 1:
            self.selection_label.setText("하나의 노드 또는 lane을 선택하세요")
            return
        item = selected[0]
        if isinstance(item, NodeItem):
            node = self.document["nodes"][item.index]
            self.selection_label.setText(f"Node {item.index}")
            self.name_edit.setText(node.get("name", "")); self.x_spin.setValue(node["x"]); self.y_spin.setValue(node["y"])
            self.holding_check.setChecked(node.get("holding", False)); self.parking_check.setChecked(node.get("parking", False))
            self.passthrough_check.setChecked(node.get("passthrough", False)); self.mutex_edit.setText(node.get("mutex_group", ""))
            self.after_penalty_spin.setValue(0.0)
        elif isinstance(item, LaneItem):
            lane = self.document["lanes"][item.index]
            self.selection_label.setText(f"Lane {item.index}: {lane['from']} → {lane['to']}")
            self.mutex_edit.setText(lane.get("mutex_group", "")); self.bidirectional_check.setChecked(lane.get("bidirectional", True))
            self.speed_spin.setValue(float(lane.get("speed_limit", 0) or 0)); self.closed_check.setChecked(lane.get("closed", False))
            self.after_penalty_spin.setValue(float(lane.get("after_penalty", 0) or 0))

    def apply_properties(self) -> None:
        selected = self.scene.selectedItems()
        if len(selected) != 1:
            return
        item = selected[0]
        if isinstance(item, NodeItem):
            node = self.document["nodes"][item.index]
            node.update({"name": self.name_edit.text().strip() or f"N{item.index}",
                         "x": self.x_spin.value(), "y": self.y_spin.value(),
                         "holding": self.holding_check.isChecked(), "parking": self.parking_check.isChecked(),
                         "passthrough": self.passthrough_check.isChecked()})
            mutex = self.mutex_edit.text().strip()
            if mutex: node["mutex_group"] = mutex
            else: node.pop("mutex_group", None)
        elif isinstance(item, LaneItem):
            lane = self.document["lanes"][item.index]
            lane["bidirectional"] = self.bidirectional_check.isChecked(); lane["closed"] = self.closed_check.isChecked()
            speed = self.speed_spin.value()
            if speed: lane["speed_limit"] = speed
            else: lane.pop("speed_limit", None)
            mutex = self.mutex_edit.text().strip()
            if mutex: lane["mutex_group"] = mutex
            else: lane.pop("mutex_group", None)
            penalty = self.after_penalty_spin.value()
            if penalty > 0: lane["after_penalty"] = penalty
            else: lane.pop("after_penalty", None)
        self.scene.load_document(self.document); self.refresh_summary(); self.show_robot_starts()

    def add_robot(self) -> None:
        row = self.robot_table.rowCount(); self.robot_table.insertRow(row)
        defaults = (
            f"R{row}", "0", str(max(0, len(self.document["nodes"]) - 1)),
            "0.0", "0.0", "0.0")
        for column, value in enumerate(defaults): self.robot_table.setItem(row, column, QTableWidgetItem(value))
        self.sync_robots(); self.refresh_summary(); self.show_robot_starts()

    def add_dynamic_robot(self) -> None:
        current_times = []
        for row in range(self.robot_table.rowCount()):
            try:
                current_times.append(float(self.robot_table.item(row, 5).text()))
            except (AttributeError, ValueError):
                pass
        insertion = max(current_times, default=0.0) + 5.0
        row = self.robot_table.rowCount()
        self.robot_table.insertRow(row)
        defaults = (
            f"R_NEW_{row}", "0", str(max(0, len(self.document["nodes"]) - 1)),
            "0.0", f"{insertion:g}", f"{insertion:g}")
        for column, value in enumerate(defaults):
            self.robot_table.setItem(row, column, QTableWidgetItem(value))
        self.document["dynamic_insertion"] = True
        self.sync_robots(); self.refresh_summary(); self.show_robot_starts()

    def delete_robot(self) -> None:
        rows = sorted({index.row() for index in self.robot_table.selectedIndexes()}, reverse=True)
        removed_names = {
            self._table_text(self.robot_table, row, 0) for row in rows}
        for row in rows: self.robot_table.removeRow(row)
        self.document["runtime_events"] = [
            event for event in self.document.get("runtime_events", [])
            if event.get("robot") not in removed_names]
        self.sync_robots(); self.refresh_summary(); self.show_robot_starts()
        self.refresh_runtime_event_table()

    def open_json(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "시나리오 JSON 열기", str(ROOT / "scenarios"), "JSON (*.json)")
        if not path: return
        try:
            self.load_document(json.loads(Path(path).read_text(encoding="utf-8")))
        except (OSError, json.JSONDecodeError, KeyError) as exc:
            QMessageBox.critical(self, "열기 실패", str(exc))

    def open_yaml(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "RMF building-map YAML 열기", str(ROOT / "scenarios"),
            "YAML (*.yaml *.yml)")
        if not path:
            return
        try:
            text = Path(path).read_text(encoding="utf-8")
            levels = available_levels(text)
            selected = levels[0]
            if len(levels) > 1:
                selected, accepted = QInputDialog.getItem(
                    self, "YAML 레벨 선택", "불러올 level", levels, 0, False)
                if not accepted:
                    return
            result = convert_building_map_yaml(text, selected)
            self.load_document(result["document"])
            metadata = result["metadata"]
            QMessageBox.information(
                self,
                "YAML 맵 가져오기 완료",
                f"{metadata['building_name']} / {metadata['selected_level']}\n"
                f"노드 {metadata['node_count']} · 원본 Lane {metadata['source_lane_count']} "
                f"· 방향 Lane {metadata['directed_lane_count']}\n\n"
                "YAML에는 로봇 요청이 없으므로 로봇 탭에서 로봇을 추가하고 "
                "start/goal node index를 지정하세요.\n\n"
                + "\n".join(f"• {item}" for item in metadata["warnings"]),
            )
        except (OSError, ValueError, RuntimeError) as exc:
            QMessageBox.critical(self, "YAML 맵 열기 실패", str(exc))

    def save_json(self) -> None:
        try:
            self.sync_robots(); self.sync_corridors(); self.sync_runtime_events()
        except ValueError as exc: QMessageBox.critical(self, "입력 오류", str(exc)); return
        path, _ = QFileDialog.getSaveFileName(self, "시나리오 JSON 저장", f"{self.document.get('name', 'scenario')}.json", "JSON (*.json)")
        if path: Path(path).write_text(json.dumps(self.document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def fit_map(self) -> None:
        self.view.fit_content()

    def _regression_profiles_ready(self) -> bool:
        for label, source_text in (
            ("OLD_SOFT/HYBRID", self.after_source_edit.text().strip()),
            ("SCHEDULE_SOFT", self.schedule_soft_source_edit.text().strip()),
            ("HYBRID+NEGO", self.after_nego_source_edit.text().strip()),
        ):
            patched, planner = after_core_patch_status(Path(source_text))
            if not patched:
                QMessageBox.critical(
                    self, "Regression 코어 준비 필요",
                    f"{label} 수정 코어 패치를 찾지 못했습니다.\n\n"
                    "RMF Core 탭에서 코어 준비 버튼을 먼저 누르세요.\n\n"
                    f"소스: {Path(source_text).expanduser()}\n"
                    f"Planner: {planner or '찾지 못함'}")
                return False
        return True

    def _regression_config(self, scenarios: list[dict]) -> tuple[Path, str]:
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        request_dir = ROOT / "results" / "regression_requests"
        request_dir.mkdir(parents=True, exist_ok=True)
        config_path = request_dir / f"{run_id}.json"
        config = {
            "run_id": run_id,
            "random_seed": 0,
            "timeout_s": self.timeout_spin.value(),
            "base_ros_setup": self.base_ros_setup_edit.text().strip(),
            "newcomer_penalty": self.after_penalty_value_spin.value(),
            "weights": {
                "same": self.policy_same_weight.value(),
                "opposite": self.policy_opposite_weight.value(),
                "occupied": self.policy_occupied_weight.value(),
                "future": self.policy_future_weight.value(),
                "no_escape": self.policy_no_escape_weight.value(),
                "static": self.policy_static_weight.value(),
                "overlap_margin": self.policy_overlap_margin.value(),
                "schedule_soft_lambda": (
                    self.schedule_soft_lambda.value() if self.schedule_soft_enabled.isChecked() else 0.0),
                "schedule_soft_max_penalty": self.schedule_soft_max_penalty.value(),
                "schedule_soft_same_weight": self.schedule_soft_same_weight.value(),
                "schedule_soft_opposite_weight": self.schedule_soft_opposite_weight.value(),
            },
            "profiles": {
                "baseline": {
                    "setup": self.before_setup_edit.text().strip(),
                    "source": self.before_source_edit.text().strip(),
                    "workspace": "", "rebuild_workspace": False,
                },
                "soft": {
                    "setup": self.after_setup_edit.text().strip(),
                    "source": self.after_source_edit.text().strip(),
                    "workspace": self.after_workspace_edit.text().strip(),
                    "rebuild_workspace": self.rebuild_after_check.isChecked(),
                },
                "schedule_soft": {
                    "setup": self.schedule_soft_setup_edit.text().strip(),
                    "source": self.schedule_soft_source_edit.text().strip(),
                    "workspace": self.schedule_soft_workspace_edit.text().strip(),
                    "rebuild_workspace": self.rebuild_schedule_soft_check.isChecked(),
                },
                "hybrid": {
                    "setup": self.after_setup_edit.text().strip(),
                    "source": self.after_source_edit.text().strip(),
                    "workspace": self.after_workspace_edit.text().strip(),
                    "rebuild_workspace": self.rebuild_after_check.isChecked(),
                },
                "hybrid_nego": {
                    "setup": self.after_nego_setup_edit.text().strip(),
                    "source": self.after_nego_source_edit.text().strip(),
                    "workspace": self.after_nego_workspace_edit.text().strip(),
                    "rebuild_workspace": self.rebuild_after_nego_check.isChecked(),
                },
            },
            "scenarios": scenarios,
        }
        config_path.write_text(
            json.dumps(config, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        return config_path, run_id

    def start_regression(self, run_all: bool) -> None:
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            QMessageBox.warning(self, "실행 중", "현재 Scenario 실행을 먼저 종료하세요.")
            return
        if (self.regression_process
                and self.regression_process.state() != QProcess.ProcessState.NotRunning):
            return
        try:
            self.sync_robots(); self.sync_corridors(); self.sync_runtime_events()
        except ValueError as exc:
            QMessageBox.critical(self, "입력 오류", str(exc)); return
        if not self._regression_profiles_ready():
            return
        if run_all:
            scenarios = [
                {"name": name, "document": copy.deepcopy(document)}
                for name, document in self.templates.items()
                if document.get("robots")]
        else:
            document = copy.deepcopy(self.document)
            scenarios = [{
                "name": str(document.get("name", "gui_current_scenario")),
                "document": document,
            }]
        config_path, run_id = self._regression_config(scenarios)
        self.regression_output_text = ""
        self.regression_summary_path = (
            ROOT / "results" / "regression" / run_id / "summary.json")
        self.regression_table.setRowCount(0)
        self.regression_status_label.setText(
            f"Regression 실행 중 · Scenario {len(scenarios)}개 × 5개 코어")
        self.regression_process = QProcess(self)
        self.regression_process.setWorkingDirectory(str(ROOT))
        self.regression_process.setProcessChannelMode(
            QProcess.ProcessChannelMode.MergedChannels)
        self.regression_process.readyReadStandardOutput.connect(
            self.read_regression_output)
        self.regression_process.finished.connect(self.regression_finished)
        self.regression_process.start(
            sys.executable,
            [str(ROOT / "regression_runner.py"), "--config", str(config_path)])
        self.regression_stop_button.setEnabled(True)
        self.output_tabs.setCurrentWidget(self.compare_panel)

    def read_regression_output(self) -> None:
        if not self.regression_process:
            return
        text = bytes(
            self.regression_process.readAllStandardOutput()).decode(errors="replace")
        self.regression_output_text += text
        lines = [line for line in self.regression_output_text.splitlines() if line.strip()]
        if lines:
            self.regression_status_label.setText(lines[-1])

    def stop_regression(self) -> None:
        if self.regression_process:
            self.regression_process.terminate()

    def regression_finished(self, exit_code: int, _status) -> None:
        self.read_regression_output()
        self.regression_stop_button.setEnabled(False)
        if self.regression_summary_path and self.regression_summary_path.is_file():
            self.load_regression_summary(self.regression_summary_path)
            self.regression_status_label.setText(
                f"Regression 완료 · {self.regression_summary_path}")
        else:
            self.regression_status_label.setText(
                f"Regression 실패 · 종료 코드 {exit_code}\n"
                + self.regression_output_text[-1200:])

    def load_regression_summary(self, path: Path) -> None:
        report = json.loads(path.read_text(encoding="utf-8"))
        rows: list[list[object]] = []
        for scenario in report.get("scenarios", []):
            for profile in report.get("profile_order", []):
                result = scenario.get("results", {}).get(profile, {})
                rows.append([
                    scenario.get("scenario"), profile, result.get("result"),
                    result.get("comparison"), result.get("conflict"),
                    result.get("deadlock"), result.get("travel_time_s"),
                    result.get("wait_time_s"), result.get("distance_m"),
                    result.get("detour"), result.get("planning_time_ms"),
                    result.get("expanded_nodes"), result.get("negotiation_count"),
                    result.get("negotiation_rounds"), result.get("validator_rejects"),
                    result.get("observed_penalty_sum"),
                    result.get("termination_reason"),
                    scenario.get("identical_input"),
                ])
        self.set_table_rows(self.regression_table, rows)
        totals = report.get("totals", {})
        profile_lines = []
        for profile in report.get("profile_order", []):
            item = totals.get("profiles", {}).get(profile, {})
            profile_lines.append(
                f"{profile}: SUCCESS {item.get('success', 0)} · "
                f"NO_SOLUTION {item.get('no_solution', 0)} · "
                f"Deadlock {item.get('deadlock', 0)} · Conflict {item.get('conflict', 0)}")
        self.compare_explanation.setPlainText(
            f"Regression 전체 요약\n\nTotal Scenarios: {totals.get('scenario_count', 0)}\n"
            + "\n".join(profile_lines)
            + f"\n\nBaseline → Modified Regression: {totals.get('regressions', 0)}"
            + f"\nBaseline → Modified Improvement: {totals.get('improvements', 0)}"
            + f"\n입력 SHA 불일치: {totals.get('input_identity_failures', 0)}"
            + f"\nRMF core provenance 불일치: {totals.get('core_provenance_failures', 0)}"
            + f"\n\n결과: {path.parent}\nsummary.json / summary.csv / profile별 JSONL·로그")

    def run_rmf(self) -> None:
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning: return
        try:
            self.sync_robots(); self.sync_corridors(); self.sync_runtime_events()
        except ValueError as exc: QMessageBox.critical(self, "입력 오류", str(exc)); return
        if not self.document.get("robots"):
            QMessageBox.warning(
                self, "로봇 필요",
                "YAML 맵에는 로봇 요청이 없습니다. 로봇 탭에서 로봇을 한 대 이상 "
                "추가하고 start/goal node index를 지정하세요.")
            return
        build = ROOT / "build"; build.mkdir(exist_ok=True)
        scenario_path = build / "gui_current_scenario.json"
        document = copy.deepcopy(self.document)
        document["name"] = "gui_current_scenario"
        scenario_path.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        profile = self.core_profile_combo.currentData()
        has_dynamic_newcomer = any(
            float(robot.get("insertion_time_s", 0) or 0) > 0
            for robot in document.get("robots", []))
        result_name = {
            "baseline": "gui_baseline",
            "soft": "gui_soft",
            "schedule_soft": "gui_schedule_soft",
            "hybrid": "gui_hybrid",
            "hybrid_nego": "gui_hybrid_nego",
        }[str(profile)]
        if profile == "baseline":
            core_label = "baseline_stock_rmf"
        elif profile == "schedule_soft":
            core_label = (
                self.schedule_soft_label_edit.text().strip() or "schedule_soft_baseline_derived")
        elif profile in {"soft", "hybrid"}:
            core_label = (
                self.after_label_edit.text().strip() or "schedule_corridor_policy")
        else:
            core_label = (
                self.after_nego_label_edit.text().strip() or "hybrid_nego")
        build_dir = ROOT / "build" / str(profile)
        source_text = (
            self.before_source_edit.text().strip()
            if profile == "baseline" else
            self.schedule_soft_source_edit.text().strip()
            if profile == "schedule_soft" else
            self.after_source_edit.text().strip()
            if profile in {"soft", "hybrid"} else
            self.after_nego_source_edit.text().strip())
        if (
            profile in {"soft", "schedule_soft", "hybrid", "hybrid_nego"}
        ):
            patched, planner_path = after_core_patch_status(Path(source_text))
            if not patched:
                QMessageBox.critical(
                    self,
                    "수정 코어 패치 확인 필요",
                    "선택한 rmf_traffic 소스에서 schedule corridor policy 패치를 "
                    "찾지 못했습니다.\n\n"
                    "RMF Core 탭에서 해당 코어 준비 버튼을 먼저 "
                    "누르세요.\n\n"
                    f"확인 소스: {Path(source_text).expanduser()}\n"
                    f"Planner: {planner_path or '찾지 못함'}",
                )
                return
        schedule_soft_lambda = (
            self.schedule_soft_lambda.value() if self.schedule_soft_enabled.isChecked() else 0.0)
        args = [str(ROOT / "run.py"), "--scenario-file", str(scenario_path),
                "--timeout", str(self.timeout_spin.value()), "--no-html",
                "--build-dir", str(build_dir), "--result-name", result_name,
                "--core-label", core_label,
                "--traffic-mode", str(profile),
                "--same-direction-weight", str(self.policy_same_weight.value()),
                "--opposite-direction-weight", str(self.policy_opposite_weight.value()),
                "--occupied-weight", str(self.policy_occupied_weight.value()),
                "--future-reservation-weight", str(self.policy_future_weight.value()),
                "--no-escape-weight", str(self.policy_no_escape_weight.value()),
                "--static-policy-weight", str(self.policy_static_weight.value()),
                "--overlap-margin", str(self.policy_overlap_margin.value()),
                "--schedule-soft-lambda", str(schedule_soft_lambda),
                "--schedule-soft-max-penalty", str(self.schedule_soft_max_penalty.value()),
                "--schedule-soft-same-weight", str(self.schedule_soft_same_weight.value()),
                "--schedule-soft-opposite-weight", str(self.schedule_soft_opposite_weight.value())]
        setup = os.path.expanduser(self.setup_edit.text().strip())
        if setup: args.extend(["--setup", setup])
        if source_text:
            args.extend(["--rmf-source", os.path.expanduser(source_text)])
        if profile == "hybrid_nego" and has_dynamic_newcomer:
            args.extend([
                "--dynamic-insertion-policy", "after_nego",
                "--lane-penalty-value", str(self.after_penalty_value_spin.value()),
            ])
        else:
            args.extend(["--dynamic-insertion-policy", "fixed_existing"])
        if profile in {"soft", "hybrid"} and self.rebuild_after_check.isChecked():
            args.extend([
                "--rebuild-rmf-workspace",
                os.path.expanduser(self.after_workspace_edit.text().strip()),
                "--base-ros-setup",
                os.path.expanduser(self.base_ros_setup_edit.text().strip()),
            ])
        if profile == "schedule_soft" and self.rebuild_schedule_soft_check.isChecked():
            args.extend([
                "--rebuild-rmf-workspace",
                os.path.expanduser(self.schedule_soft_workspace_edit.text().strip()),
                "--base-ros-setup",
                os.path.expanduser(self.base_ros_setup_edit.text().strip()),
            ])
        if profile == "hybrid_nego" and self.rebuild_after_nego_check.isChecked():
            args.extend([
                "--rebuild-rmf-workspace",
                os.path.expanduser(self.after_nego_workspace_edit.text().strip()),
                "--base-ros-setup",
                os.path.expanduser(self.base_ros_setup_edit.text().strip()),
            ])
        if not self.build_check.isChecked(): args.append("--skip-build")
        self.runtime_output_text = ""
        self.runtime_log.clear(); self.runtime_log_korean.clear()
        self.raw_log.clear(); self.jsonl_summary.clear()
        self.diagnosis.clear(); self.diagnosis_raw.clear()
        self.failure_summary.setPlainText("RMF 실행 중 · 실제 이벤트를 수집하는 중입니다.")
        self.failure_trace_table.setRowCount(0)
        self.schedule_explanation.setPlainText(
            "RMF 실행 중 · 왼쪽 Schedule DB 표의 행을 선택하면 값을 해석합니다.")
        self.schedule_model.setPlainText(
            "RMF 실행 중 · 실제 Database 객체 구조와 query 결과를 기다리는 중입니다.")
        self.astar_explanation.setPlainText(
            "RMF 실행 중 · 왼쪽 A* 표의 행을 선택하면 g/h/f 선택 근거를 해석합니다.")
        self.decision_explanation.setPlainText("RMF 실행 중 · 판단 이벤트를 기다리는 중입니다.")
        self.object_explanation.setPlainText(
            "RMF 실행 중 · Graph, Start/Goal, Validator, Proposal, 협상 원문 이벤트를 기다리는 중입니다.")
        self.object_guide.setPlainText(rmf_object_guide_text())
        for table in (
            self.schedule_state_table, self.schedule_operation_table,
            self.schedule_participant_table, self.schedule_route_table,
            self.schedule_point_table,
            self.corridor_definition_table, self.corridor_snapshot_table,
            self.corridor_interval_table, self.corridor_decision_table,
            self.corridor_runtime_table,
            self.route_validator_result_table,
            self.astar_table, self.policy_astar_table, self.decision_table,
            self.supergraph_table, self.graph_node_table, self.graph_lane_table,
            self.start_goal_table, self.validator_table, self.itinerary_table,
            self.route_object_table, self.trajectory_object_table,
            self.proposal_table, self.negotiation_timeline_table,
            self.negotiation_process_table,
            self.reject_forfeit_table,
        ):
            table.setRowCount(0)
        self.current_decision_label.setText("현재 판단: RMF가 경로와 협상안을 계산하고 있습니다.")
        self.scene.clear_robot_overlay()
        self.scene.set_runtime_penalties({}, {})
        self.animation_timer.stop()
        self.last_jsonl_content = ""
        self.result_path = ROOT / "results" / f"{result_name}.jsonl"
        try:
            self.result_path.unlink(missing_ok=True)
        except OSError as exc:
            QMessageBox.critical(self, "결과 파일 오류", str(exc)); return
        self.process = QProcess(self)
        self.process.setWorkingDirectory(str(ROOT))
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_process_output)
        self.process.finished.connect(self.process_finished)
        self.process.start(sys.executable, args)
        self.jsonl_timer.start()
        self.run_button.setText("RMF 코어 분석 진행 중…")
        self.run_button.setEnabled(False); self.stop_button.setEnabled(True)
        self.statusBar().showMessage(f"{core_label}: 실제 C++ RMF Traffic 실행 중…")

    def read_process_output(self) -> None:
        if self.process:
            text = bytes(self.process.readAllStandardOutput()).decode(errors="replace")
            self.runtime_output_text += text
            self.runtime_log.moveCursor(QTextCursor.MoveOperation.End)
            self.runtime_log.insertPlainText(text)
            self.runtime_log_korean.setPlainText(
                explain_runtime_output(self.runtime_output_text))
            self.runtime_log_korean.moveCursor(QTextCursor.MoveOperation.End)

    def stop_process(self) -> None:
        if self.process: self.process.terminate()

    def refresh_live_jsonl(self) -> None:
        if not self.result_path.is_file():
            return
        try:
            content = self.result_path.read_text(encoding="utf-8")
        except OSError:
            return
        if content == self.last_jsonl_content:
            return
        self.last_jsonl_content = content
        self.raw_log.setPlainText(content.rstrip("\n"))
        self.raw_log.moveCursor(QTextCursor.MoveOperation.End)
        self.events = []
        for line in content.splitlines():
            try:
                self.events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        self.update_event_explanations()
        self.fill_event_tables()

    def process_finished(self, exit_code: int, _status) -> None:
        self.read_process_output(); self.jsonl_timer.stop(); self.refresh_live_jsonl()
        self.run_button.setEnabled(True); self.stop_button.setEnabled(False)
        self.update_run_button_text()
        path = self.result_path
        if path.is_file():
            self.load_events(path)
            self.update_comparison()
            self.statusBar().showMessage(f"RMF 종료 코드 {exit_code} · JSONL {path}")
        else:
            self.statusBar().showMessage(f"실행 실패 (종료 코드 {exit_code})")

    def load_events(self, path: Path) -> None:
        lines = path.read_text(encoding="utf-8").splitlines()
        self.raw_log.setPlainText("\n".join(lines))
        self.events = []
        for line in lines:
            try: self.events.append(json.loads(line))
            except json.JSONDecodeError: continue
        self.update_event_explanations()
        self.fill_event_tables(); self.prepare_animation()

    def update_event_explanations(self) -> None:
        profiles = [
            event for event in self.events
            if event.get("event") == "runner_core_profile"
        ]
        profile = profiles[-1] if profiles else {}

        def numeric_lane_map(field: str) -> dict[int, float]:
            output: dict[int, float] = {}
            raw = profile.get(field, {})
            if isinstance(raw, dict):
                for lane, value in raw.items():
                    try:
                        output[int(lane)] = float(value)
                    except (TypeError, ValueError):
                        continue
            return output

        self.scene.set_runtime_penalties(
            numeric_lane_map("directed_lane_penalties"),
            numeric_lane_map("directed_lane_occupancy"),
        )
        diagnoses = [event for event in self.events if event.get("event") == "solution_diagnosis"]
        self.diagnosis_raw.setPlainText("\n\n".join(
            json.dumps(event, ensure_ascii=False, indent=2) for event in diagnoses
        ) or "solution_diagnosis 이벤트가 없습니다.")
        self.diagnosis.setPlainText(
            "\n\n".join(diagnosis_text(event) for event in diagnoses)
            or "RMF 실행 중 · 아직 최종 진단이 없습니다.\n\n"
               "단일 경로 결과 또는 협상 결과가 기록되면 원인·증거·해결 방법을 한글로 표시합니다."
        )
        self.jsonl_summary.setPlainText(summarize_jsonl(self.events))
        self.failure_summary.setPlainText(failure_summary_text(self.events))
        guide = schedule_guide_text(self.events)
        self.schedule_guide.setPlainText(guide)
        self.schedule_model.setPlainText(schedule_model_text(self.events))
        self.object_guide.setPlainText(rmf_object_guide_text(self.events))

    @staticmethod
    def _details(event: dict, excluded: set[str]) -> str:
        return json.dumps({k: v for k, v in event.items() if k not in excluded}, ensure_ascii=False)

    @staticmethod
    def set_table_rows(table: QTableWidget, rows: list[list[object]]) -> None:
        table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            for column, value in enumerate(row):
                table.setItem(row_index, column, QTableWidgetItem(str(value if value is not None else "")))

    def fill_event_tables(self) -> None:
        failure_rows = failure_trace_records(self.events)
        self.set_table_rows(self.failure_trace_table, [[
            row.get("seq", ""), row.get("stage", ""), row.get("actor", ""),
            row.get("event", ""), row.get("status", ""), row.get("location", ""),
            row.get("time", ""), row.get("detail", ""), row.get("source", ""),
        ] for row in failure_rows])
        states = [event for event in self.events if event.get("event") == "schedule_database_state"]
        self.set_table_rows(self.schedule_state_table, [[
            event.get("seq"), event.get("phase"), event.get("latest_version"),
            event.get("participant_count"), event.get("stored_route_count", 0),
            event.get("database_class", "rmf_traffic::schedule::Database"),
            event.get("view_class", "rmf_traffic::schedule::Viewer::View"),
            event.get("read_api", ""),
        ] for event in states])

        operations = [event for event in self.events if event.get("event") == "schedule_database_operation"]
        self.set_table_rows(self.schedule_operation_table, [[
            event.get("seq"), event.get("action"), event.get("api"),
            event.get("version_before"), event.get("version_after"),
            event.get("participant_id", ""), event.get("result", ""),
        ] for event in operations])

        participants = [event for event in self.events if event.get("event") == "schedule_participant"]
        cumulative_delay: dict[str, float] = {}
        reached_checkpoint: dict[str, object] = {}
        for runtime_event in self.events:
            if runtime_event.get("event") != "runtime_traffic_event":
                continue
            robot = str(runtime_event.get("robot", ""))
            if runtime_event.get("type") == "DELAY" and runtime_event.get("schedule_changed"):
                cumulative_delay[robot] = cumulative_delay.get(robot, 0.0) + float(
                    runtime_event.get("value_s", 0.0) or 0.0)
            if runtime_event.get("type") == "CHECKPOINT_RELEASE":
                reached_checkpoint[robot] = runtime_event.get("detail", "confirmed exit")
        self.set_table_rows(self.schedule_participant_table, [[
            event.get("phase"), event.get("participant_id"), event.get("name"),
            event.get("owner"), event.get("responsive"),
            f"{event.get('profile_footprint', 'Circle')} r={event.get('profile_radius_m', 0.3)}m",
            event.get("itinerary_version"),
            event.get("progress_version"), event.get("current_plan_id"),
            event.get("route_count"), event.get("trajectory_point_count"),
            event.get("cumulative_delay_s",
                      cumulative_delay.get(str(event.get("name", "")), 0.0)),
            reached_checkpoint.get(str(event.get("name", "")), ""),
            (
                f"SCHEDULE + {event.get('cumulative_delay_source')}"
                if event.get("cumulative_delay_s") else
                "SIMULATION_EVENT" if str(event.get("name", "")) in cumulative_delay
                else f"SCHEDULE / {event.get('reached_checkpoint_source', '')}"
            ),
            event.get("itinerary_read_api", event.get("read_from", "")),
        ] for event in participants])

        routes = [
            event for event in self.events
            if event.get("event") == "schedule_database_route"
        ]
        if not routes:  # Legacy JSONL fallback
            routes = [
                event for event in self.events
                if event.get("event") == "schedule_itinerary_route"
            ]
        corridor_route_context: dict[tuple[object, object, object], dict] = {}
        for interval in self.events:
            if interval.get("event") != "corridor_schedule_interval":
                continue
            key = (
                interval.get("participant_id"), interval.get("plan_id"),
                interval.get("route_id"))
            corridor_route_context[key] = {
                "corridor": interval.get("corridor_id"),
                "direction": interval.get("direction"),
                "enter": interval.get("corridor_enter_s"),
                "exit": interval.get("corridor_exit_s"),
                "source": (
                    "SCHEDULE + POLICY_DERIVED"
                    if interval.get("source") == "SCHEDULE"
                    else "POLICY_DERIVED"
                ),
            }
        for decision in self.events:
            if decision.get("event") != "corridor_policy_expansion":
                continue
            for overlap in decision.get("overlaps", []):
                key = (
                    overlap.get("participant_id"), overlap.get("plan_id"),
                    overlap.get("route_id"))
                corridor_route_context.setdefault(key, {
                    "corridor": decision.get("corridor_id"),
                    "direction": overlap.get("direction"),
                    "enter": overlap.get("occupancy_enter"),
                    "exit": overlap.get("occupancy_exit"),
                    "source": "POLICY_DERIVED",
                })
        self.set_table_rows(self.schedule_route_table, [[
            event.get("phase"), event.get("participant_id"), event.get("name"),
            event.get("plan_id"),
            event.get("route_id", event.get("route_index")), event.get("map"),
            event.get("start_time_s"), event.get("finish_time_s"),
            event.get("duration_s"), event.get("trajectory_point_count"),
            corridor_route_context.get((
                event.get("participant_id"), event.get("plan_id"),
                event.get("route_id", event.get("route_index"))), {}).get("corridor", ""),
            corridor_route_context.get((
                event.get("participant_id"), event.get("plan_id"),
                event.get("route_id", event.get("route_index"))), {}).get("direction", ""),
            corridor_route_context.get((
                event.get("participant_id"), event.get("plan_id"),
                event.get("route_id", event.get("route_index"))), {}).get("enter", ""),
            corridor_route_context.get((
                event.get("participant_id"), event.get("plan_id"),
                event.get("route_id", event.get("route_index"))), {}).get("exit", ""),
            corridor_route_context.get((
                event.get("participant_id"), event.get("plan_id"),
                event.get("route_id", event.get("route_index"))), {}).get("source", "SCHEDULE"),
            event.get("object_path", event.get("read_from", "")),
        ] for event in routes])

        points = [
            event for event in self.events
            if event.get("event") == "schedule_database_trajectory_point"
        ]
        if not points:  # Legacy JSONL fallback
            points = [
                event for event in self.events
                if event.get("event") == "schedule_trajectory_point"
            ]
        self.set_table_rows(self.schedule_point_table, [[
            event.get("phase"), event.get("participant_id"), event.get("name"),
            event.get("plan_id"),
            event.get("route_id", event.get("route_index")), event.get("sequence"),
            event.get("time_s"), event.get("x"), event.get("y"),
            event.get("yaw_rad"), event.get("vx"), event.get("vy"), event.get("vyaw"),
            event.get("object_path", event.get("read_from", "")),
        ] for event in points])

        self.schedule_event_rows = {
            self.schedule_state_table: states,
            self.schedule_operation_table: operations,
            self.schedule_participant_table: participants,
            self.schedule_route_table: routes,
            self.schedule_point_table: points,
        }

        corridor_definitions = [
            event for event in self.events
            if event.get("event") == "corridor_definition"]
        corridor_snapshots = [
            event for event in self.events
            if event.get("event") == "corridor_policy_snapshot"]
        corridor_decisions = [
            event for event in self.events
            if event.get("event") == "corridor_policy_expansion"]
        self.corridor_decision_events = corridor_decisions
        corridor_intervals = [
            event for event in self.events
            if event.get("event") == "corridor_schedule_interval"]
        corridor_runtime = [
            event for event in self.events
            if event.get("event") in {
                "runtime_event_definition", "runtime_traffic_event",
                "replan_trigger", "corridor_runtime_state",
                "corridor_state_transition"}]
        route_validator_results = [
            event for event in self.events
            if event.get("event") == "route_validator_result"]
        negotiating_validator_configs = [
            event for event in self.events
            if event.get("event") == "validator_configuration"
            and event.get("validator") == "NegotiatingRouteValidator"]
        self.set_table_rows(self.corridor_definition_table, [[
            event.get("seq"), event.get("corridor_id"), event.get("lanes_forward"),
            event.get("lanes_reverse"), event.get("capacity"),
            event.get("passing_allowed"), event.get("hard_opposite_direction_block"),
            event.get("holding_entry_a"), event.get("holding_entry_b"),
            event.get("base_penalty"), event.get("source"),
        ] for event in corridor_definitions])
        self.set_table_rows(self.corridor_snapshot_table, [[
            event.get("seq"), event.get("mode"), event.get("schedule_version"),
            event.get("participant_id"), event.get("planning_time_s"),
            event.get("corridor_count"), event.get("interval_count"),
            event.get("invocation_reason"), event.get("query_api"),
            f"{event.get('source')} + {event.get('derived_source')}",
        ] for event in corridor_snapshots])
        self.set_table_rows(self.corridor_interval_table, [[
            event.get("seq"), event.get("snapshot_generation"),
            event.get("schedule_version"), event.get("corridor_id"),
            event.get("participant_id"), event.get("plan_id"), event.get("route_id"),
            event.get("direction"), event.get("corridor_enter_s"),
            event.get("corridor_exit_s"), event.get("state"), event.get("owner"),
            event.get("responsive"), event.get("itinerary_version"),
            event.get("trajectory_source"), event.get("state_source"),
        ] for event in corridor_intervals])
        self.set_table_rows(self.corridor_decision_table, [[
            event.get("seq"), event.get("participant_id"), event.get("corridor_id"),
            event.get("direction"), event.get("predicted_enter_time"),
            event.get("predicted_exit_time"), event.get("is_entry"),
            event.get("decision"), event.get("static_penalty"),
            event.get("same_direction_penalty"),
            event.get("opposite_direction_penalty"),
            event.get("corridor_occupancy_penalty"), event.get("no_escape_penalty"),
            event.get("total_policy_penalty"),
            [overlap.get("participant_id") for overlap in event.get("overlaps", [])],
            f"{event.get('source')}/{event.get('analysis_source')}",
        ] for event in corridor_decisions])
        self.set_table_rows(self.policy_astar_table, [[
            event.get("seq"), event.get("candidate_id"), event.get("parent_id"),
            event.get("participant_id"), event.get("current_waypoint"),
            event.get("target_waypoint"), event.get("lane_ids"),
            event.get("corridor_id"), event.get("direction"),
            event.get("predicted_enter_time"), event.get("predicted_exit_time"),
            event.get("parent_g"), event.get("approach_cost"),
            event.get("rmf_core_alt_cost"),
            event.get("base_move_cost"), event.get("rotation_cost"),
            event.get("event_cost"), event.get("wait_cost"),
            event.get("static_penalty"),
            float(event.get("same_direction_penalty", 0.0) or 0.0)
            + float(event.get("opposite_direction_penalty", 0.0) or 0.0)
            + float(event.get("corridor_occupancy_penalty", 0.0) or 0.0),
            event.get("same_direction_penalty"),
            event.get("opposite_direction_penalty"),
            event.get("corridor_occupancy_penalty"), event.get("no_escape_penalty"),
            event.get("total_policy_penalty"), event.get("final_g"),
            event.get("h"), event.get("f"), event.get("decision"),
            json.dumps(event.get("overlaps", []), ensure_ascii=False),
            f"{event.get('source')}/{event.get('schedule_source')}/{event.get('analysis_source')}",
        ] for event in corridor_decisions])
        self.set_table_rows(self.corridor_runtime_table, [[
            event.get("seq"), event.get("event"), event.get("corridor_id", ""),
            event.get("state", event.get("to_state", event.get("type", ""))),
            event.get("direction", ""), event.get("owner", ""),
            event.get("occupants", ""), event.get("reserved_participants", ""),
            event.get("waiting_same_direction", ""),
            event.get("waiting_opposite_direction", ""),
            (
                f"{event.get('reserved_enter_s')}~{event.get('reserved_exit_s')}"
                if event.get("reserved_enter_s") is not None else ""
            ),
            event.get("last_update_s", event.get("at_s", "")),
            event.get("release_condition",
                      event.get("replan_trigger", event.get("reason", event.get("detail", "")))),
            event.get("passing_allowed", ""), event.get("capacity", ""),
            event.get("robot", ""), event.get("value_s", ""),
            (
                f"{event.get('schedule_changed', '')} / "
                f"{event.get('schedule_version_before', '')}→{event.get('schedule_version_after', '')}"
            ),
            f"{event.get('source')}/{event.get('schedule_source', '')}",
        ] for event in corridor_runtime])
        latest_corridor_states: dict[str, str] = {}
        for event in corridor_runtime:
            if event.get("event") == "corridor_runtime_state":
                latest_corridor_states[str(event.get("corridor_id", ""))] = str(
                    event.get("state", "FREE"))
        self.scene.set_corridor_overlay(
            self.corridor_overlay_check.isChecked(), latest_corridor_states)
        validator_rows = [[
            event.get("seq"), event.get("phase"), event.get("validator"),
            event.get("participant_id"), event.get("candidate_route_id"),
            event.get("decision"), event.get("reason_code"),
            event.get("blocker_participant", ""), event.get("blocker_plan_id", ""),
            event.get("blocker_route_id", ""), event.get("conflict_time_s", ""),
            f"{event.get('source')}/{event.get('schedule_source')}",
        ] for event in route_validator_results]
        validator_rows.extend([[
            event.get("seq"), event.get("phase"), event.get("validator"), "", "",
            "ACTIVE_INTERNAL_NOT_OBSERVABLE",
            "SimpleNegotiator 내부 실제 validator; public Result에는 호출별 conflict가 없음",
            "", "", "", "", f"{event.get('source')}/{event.get('schedule_source')}",
        ] for event in negotiating_validator_configs])
        self.set_table_rows(self.route_validator_result_table, validator_rows)

        graph_contexts = [
            event for event in self.events
            if event.get("event") == "planner_graph_context"
        ]
        graph_nodes = [
            event for event in self.events if event.get("event") == "graph_node"
        ]
        graph_lanes = [
            event for event in self.events if event.get("event") == "graph_lane"
        ]
        start_goals = [
            event for event in self.events
            if event.get("event") == "planning_request"
        ]
        validators = [
            event for event in self.events
            if event.get("event") == "validator_configuration"
        ]
        itineraries = [
            event for event in self.events
            if event.get("event") == "itinerary_summary"
        ]
        route_objects = [
            event for event in self.events
            if event.get("event") == "route_summary"
        ]
        trajectory_objects = [
            event for event in self.events
            if event.get("event") == "trajectory_point"
        ]
        proposal_events = [
            event for event in self.events
            if event.get("event") in {
                "proposal_summary", "proposal_plan", "proposal_outcome"}
        ]
        negotiation_logs = [
            event for event in self.events
            if event.get("event") == "negotiation_log"
        ]
        negotiation_timeline_events = [
            event for event in self.events
            if event.get("event") in {
                "negotiation_request", "dynamic_negotiation_request",
                "validator_configuration", "negotiation_log",
                "proposal_summary", "proposal_plan", "safety_verification",
                "safety_pair_check", "pairwise_conflict_check",
                "proposal_outcome", "schedule_database_operation",
                "schedule_commit", "negotiation_summary",
            }
            and (
                event.get("event") != "validator_configuration"
                or event.get("schedule_aware") is True
            )
        ]

        self.set_table_rows(self.supergraph_table, [[
            event.get("seq"), event.get("graph_object"),
            event.get("graph_read_api"), event.get("waypoint_count"),
            event.get("directed_lane_count"), event.get("supergraph_object"),
            event.get("supergraph_public_api_available"),
            event.get("supergraph_observation"),
        ] for event in graph_contexts])
        self.set_table_rows(self.graph_node_table, [[
            event.get("seq"), event.get("id"), event.get("name"), event.get("map"),
            event.get("x"), event.get("y"), event.get("holding"),
            event.get("parking"), event.get("passthrough"), event.get("mutex_group"),
            event.get("outgoing_lanes"), event.get("incoming_lanes"),
        ] for event in graph_nodes])
        self.set_table_rows(self.graph_lane_table, [[
            event.get("seq"), event.get("id"), event.get("entry"), event.get("exit"),
            event.get("length_m"), event.get("speed_limit_mps"),
            event.get("effective_speed_mps"), event.get("mutex_group"),
            event.get("closed"),
        ] for event in graph_lanes])
        self.set_table_rows(self.start_goal_table, [[
            event.get("seq"), event.get("robot"), event.get("mode"),
            event.get("start_object_type", "Plan::Start"), event.get("start"),
            event.get("effective_plan_time_s", event.get("start_time_s")),
            event.get("start_yaw_rad"), event.get("goal_object_type", "Plan::Goal"),
            event.get("goal"), event.get("goal_orientation_constraint"),
            event.get("goal_any_orientation", True), event.get("insertion_time_s"),
        ] for event in start_goals])
        self.set_table_rows(self.validator_table, [[
            event.get("seq"), event.get("phase"), event.get("stage"),
            event.get("planner_options_validator"),
            event.get("validator_object_publicly_exposed", ""),
            event.get("schedule_aware"), event.get("schedule_database_version"),
            event.get("post_proposal_validator"),
            event.get("important", event.get("purpose", "")),
        ] for event in validators])
        self.set_table_rows(self.itinerary_table, [[
            event.get("seq"), event.get("robot"), event.get("phase"),
            event.get("object_type"), event.get("route_count"),
            event.get("source_api"), event.get("schedule_committed"),
            event.get("meaning"),
        ] for event in itineraries])
        self.set_table_rows(self.route_object_table, [[
            event.get("seq"), event.get("robot"), event.get("phase"),
            event.get("route_index"), event.get("map"),
            event.get("trajectory_point_count"), event.get("start_time_s"),
            event.get("finish_time_s"), event.get("duration_s"),
            event.get("source_api"),
        ] for event in route_objects])
        self.set_table_rows(self.trajectory_object_table, [[
            event.get("seq"), event.get("robot"), event.get("phase"),
            event.get("route_index"), event.get("sequence"), event.get("time_s"),
            event.get("x"), event.get("y"), event.get("yaw_rad"),
            event.get("vx"), event.get("vy"), event.get("vyaw"),
            event.get("source_api", "Route::trajectory()"),
        ] for event in trajectory_objects])
        self.set_table_rows(self.proposal_table, [[
            event.get("seq"), event.get("event"), event.get("phase"),
            event.get("stage"), event.get("robot"), event.get("participant_id"),
            event.get("present"), event.get("participant_plan_count"),
            event.get("cost"), event.get("waypoint_count"),
            event.get("itinerary_route_count"), event.get("trajectory_point_count"),
            event.get("validated"), event.get("accepted"), event.get("committed"),
            event.get("action", event.get("reason", event.get("commit_state", ""))),
        ] for event in proposal_events])

        timeline_rows = []
        for event in negotiation_timeline_events:
            kind = str(event.get("event", ""))
            action = str(event.get("action", ""))
            detail = event.get("api", event.get("source_api", ""))
            result = ""
            if kind == "negotiation_log":
                action, _label, _description = classify_negotiation_message(
                    str(event.get("message", "")))
                detail = event.get("message", "")
            elif kind == "proposal_summary":
                action = "proposal_available" if event.get("present") else "no_proposal"
                result = f"Plan {event.get('participant_plan_count', 0)}개"
            elif kind == "proposal_plan":
                action = "proposal_plan"
                result = f"cost={event.get('cost')}, Route={event.get('itinerary_route_count')}"
            elif kind in {"safety_verification", "safety_pair_check", "pairwise_conflict_check"}:
                action = "detect_conflict"
                result = "통과" if event.get("passed") else "충돌"
                detail = event.get("method", event.get("reason", detail))
            elif kind == "proposal_outcome":
                result = (
                    f"accepted={event.get('accepted')}, "
                    f"committed={event.get('committed')}"
                )
                detail = event.get("reason", detail)
            elif kind == "schedule_database_operation":
                result = f"v{event.get('version_before')}→v{event.get('version_after')}"
                detail = f"{event.get('api', '')} · {event.get('result', '')}"
            elif kind == "negotiation_summary":
                action = "final_result"
                result = f"success={event.get('success')}"
                detail = event.get("interpretation", "")
            elif kind in {"negotiation_request", "dynamic_negotiation_request"}:
                action = "start_negotiation"
                detail = event.get("api", "CentralizedNegotiation::solve")
            elif kind == "validator_configuration":
                action = "configure_validator"
                detail = event.get("important", event.get("planner_options_validator", ""))
            elif kind == "schedule_commit":
                action = "commit"
                result = f"plan_id={event.get('plan_id')}"
            timeline_rows.append([
                event.get("seq"), kind, event.get("phase"), event.get("stage"),
                event.get("robot", event.get("participant_id", "")), action,
                result, detail,
            ])
        self.set_table_rows(self.negotiation_timeline_table, timeline_rows)

        negotiation_rows = []
        for event in negotiation_logs:
            action, label, _description = classify_negotiation_message(
                str(event.get("message", "")))
            negotiation_rows.append([
                event.get("seq"), event.get("stage"),
                event.get("action", action), label, event.get("message"),
                event.get("source_api", "CentralizedNegotiation::Result::log"),
            ])
        self.set_table_rows(self.negotiation_process_table, negotiation_rows)

        reject_forfeit_events = []
        reject_forfeit_rows = []
        for event in negotiation_logs:
            action, label, _description = classify_negotiation_message(
                str(event.get("message", "")))
            if action not in {"reject", "forfeit"}:
                continue
            reject_forfeit_events.append(event)
            reject_forfeit_rows.append([
                event.get("seq"), "CentralizedNegotiation raw log",
                event.get("stage"), action, label, "", "", event.get("message"),
            ])
        for event in proposal_events:
            if event.get("event") != "proposal_outcome":
                continue
            action = str(event.get("action", ""))
            label = {
                "reject_no_proposal": "Proposal 없음으로 거부",
                "reject_after_detect_conflict": "충돌검사 후 거부",
                "accept_and_commit": "검증 후 수락·저장",
            }.get(action, action)
            reject_forfeit_events.append(event)
            reject_forfeit_rows.append([
                event.get("seq"), "Proposal outcome", event.get("stage"),
                action, label, event.get("accepted"), event.get("committed"),
                event.get("reason"),
            ])
        ordered_reject_forfeit = sorted(
            zip(reject_forfeit_events, reject_forfeit_rows),
            key=lambda item: int(item[0].get("seq", 0) or 0),
        )
        reject_forfeit_events = [item[0] for item in ordered_reject_forfeit]
        reject_forfeit_rows = [item[1] for item in ordered_reject_forfeit]
        self.set_table_rows(self.reject_forfeit_table, reject_forfeit_rows)

        self.object_event_rows = {
            self.supergraph_table: graph_contexts,
            self.graph_node_table: graph_nodes,
            self.graph_lane_table: graph_lanes,
            self.start_goal_table: start_goals,
            self.validator_table: validators,
            self.itinerary_table: itineraries,
            self.route_object_table: route_objects,
            self.trajectory_object_table: trajectory_objects,
            self.proposal_table: proposal_events,
            self.negotiation_timeline_table: negotiation_timeline_events,
            self.negotiation_process_table: negotiation_logs,
            self.reject_forfeit_table: reject_forfeit_events,
        }

        astar_events = [event for event in self.events if event.get("event", "").startswith("astar_")]
        self.set_table_rows(self.astar_table, [[
            event.get("seq", ""), event.get("robot", ""), event.get("event", ""),
            event.get("step", ""),
            event.get("node_id", event.get("selected_node_id", event.get("expanded_node_id", ""))),
            event.get("parent_id", ""),
            event.get("waypoint", event.get("selected_waypoint", "")),
            event.get("g", event.get("selected_g", "")),
            event.get("delta_g_from_parent", ""),
            event.get("g_route_elapsed_s", ""),
            event.get("g_translation_time_s", ""),
            event.get("g_rotation_time_s", ""),
            event.get("g_wait_time_s", ""),
            event.get("g_translation_distance_m", ""),
            event.get("g_rotation_angle_rad", ""),
            event.get("g_unexposed_remainder", ""),
            event.get("h", event.get("selected_h", "")),
            event.get("h_graph_distance_m", ""),
            event.get("h_graph_cruise_time_s", ""),
            event.get("h_first_turn_time_s", ""),
            event.get("h_rmf_minus_graph_cruise_s", ""),
            event.get("f", event.get("selected_f", "")),
            event.get("queue_size", event.get("frontier_size_before", event.get("frontier_size_after", ""))),
            event.get("next_best_f", ""),
            event.get("selection_basis", event.get("ordering_rule", "")),
        ] for event in astar_events])
        self.astar_event_rows = astar_events

        self.decision_rows = decision_records(self.events)
        self.set_table_rows(self.decision_table, [[
            record["seq"], record["phase"], record["robot"], record["decision"],
            record["reason"], record["evidence"], record["result"],
        ] for record in self.decision_rows])
        self.decision_row_by_seq = {
            record["seq"]: row for row, record in enumerate(self.decision_rows)
        }

        if self.astar_table.currentRow() < 0:
            self.astar_explanation.setPlainText(
                "왼쪽 A* 행을 선택하면 실제 g/h/f 근거를 표시합니다. "
                "오른쪽 ‘g/h/f 가이드’ 탭에는 전체 용어 설명이 있습니다.")
        if self.decision_rows and self.decision_table.currentRow() < 0:
            self.decision_explanation.setPlainText(
                "판단 이벤트가 시간순으로 정리됐습니다. 행을 선택하거나 시뮬레이션을 재생하세요.\n\n"
                + self.decision_rows[0]["detail"]
            )

    @staticmethod
    def summarize_result(path: Path) -> dict[str, object] | None:
        if not path.is_file():
            return None
        events = []
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        if not events:
            return None

        def last(event_name: str) -> dict:
            matches = [event for event in events if event.get("event") == event_name]
            return matches[-1] if matches else {}

        successful_plans = [
            event for event in events
            if event.get("event") == "plan_summary" and event.get("success")
        ]
        requested_start_map: dict[str, float] = {}
        insertion_time_map: dict[str, float] = {}
        for event in events:
            if event.get("event") == "planning_request":
                requested_start_map[str(event.get("robot"))] = float(
                    event.get("start_time_s", 0) or 0)
                insertion_time_map[str(event.get("robot"))] = float(
                    event.get("insertion_time_s", 0) or 0)
        phases = {event.get("phase") for event in successful_plans}
        selected_phase = (
            "negotiated" if "negotiated" in phases else
            "free_flow" if "free_flow" in phases else ""
        )
        plans = [event for event in successful_plans if event.get("phase") == selected_phase]
        baselines = [
            event for event in successful_plans
            if event.get("phase") == "free_flow_baseline"
        ]
        route_map = {
            str(event.get("robot")): list(event.get("used_lanes", []))
            for event in plans
        }
        baseline_route_map = {
            str(event.get("robot")): list(event.get("used_lanes", []))
            for event in baselines
        }
        finish_map = {
            str(event.get("robot")): float(event.get("finish_time_s", 0) or 0)
            for event in plans
        }
        baseline_finish_map = {
            str(event.get("robot")): float(event.get("finish_time_s", 0) or 0)
            for event in baselines
        }

        def lane_map_text(mapping: dict[str, list]) -> str:
            if not mapping:
                return "경로 없음"
            return " | ".join(
                f"{robot}:{lanes}" for robot, lanes in sorted(mapping.items())
            )

        lane_lengths = {
            int(event.get("id")): float(event.get("length_m", 0) or 0)
            for event in events
            if event.get("event") == "graph_lane" and event.get("id") is not None
        }

        def route_distance(mapping: dict[str, list]) -> float:
            return sum(
                lane_lengths.get(int(lane), 0.0)
                for lanes in mapping.values()
                for lane in lanes
            )

        def explicit_wait_time(robot: str, phase: str) -> float:
            waypoints = sorted([
                event for event in events
                if event.get("event") == "plan_waypoint"
                and str(event.get("robot")) == robot
                and event.get("phase") == phase
            ], key=lambda event: int(event.get("sequence", 0)))
            wait = 0.0
            for left, right in zip(waypoints, waypoints[1:]):
                same_xy = math.hypot(
                    float(right.get("x", 0)) - float(left.get("x", 0)),
                    float(right.get("y", 0)) - float(left.get("y", 0)),
                ) < 1e-6
                yaw_delta = math.atan2(
                    math.sin(float(right.get("yaw_rad", 0)) - float(left.get("yaw_rad", 0))),
                    math.cos(float(right.get("yaw_rad", 0)) - float(left.get("yaw_rad", 0))),
                )
                if same_xy and abs(yaw_delta) < 1e-5:
                    wait += max(0.0, float(right.get("time_s", 0)) - float(left.get("time_s", 0)))
            return wait

        wait_map = {
            robot: explicit_wait_time(robot, selected_phase)
            for robot in route_map
        } if selected_phase else {}
        negotiation_rerouted = [
            robot for robot, lanes in route_map.items()
            if robot in baseline_route_map and lanes != baseline_route_map[robot]
        ]
        negotiation_rescheduled = [
            robot for robot, lanes in route_map.items()
            if robot in baseline_route_map
            and lanes == baseline_route_map[robot]
            and abs(finish_map.get(robot, 0) - baseline_finish_map.get(robot, 0)) > 1e-6
        ]
        robot_outcomes = []
        for robot, lanes in sorted(route_map.items()):
            baseline_lanes = baseline_route_map.get(robot)
            if baseline_lanes is None:
                robot_outcomes.append(f"{robot}: 최종 {lanes}")
            elif baseline_lanes != lanes:
                robot_outcomes.append(f"{robot}: 우회 {baseline_lanes}→{lanes}")
            else:
                delay = finish_map.get(robot, 0) - baseline_finish_map.get(robot, 0)
                robot_outcomes.append(
                    f"{robot}: Lane 동일·시간조정 {delay:+.3f}s" if abs(delay) > 1e-6
                    else f"{robot}: 기준 경로 유지")

        choice_events = [
            event for event in events if event.get("event") == "route_choice_explanation"
        ]
        candidate_evidence = " | ".join(
            f"{event.get('robot')}: 후보 {event.get('selected_rank', '?')}위, "
            f"비용 {event.get('selected_cost', '?')}, 차순위 차이 {event.get('cost_margin', '?')}"
            for event in choice_events
        ) or "후보 비교 없음"
        profile = last("runner_core_profile")
        diagnosis = last("solution_diagnosis")
        negotiation = last("negotiation_summary")
        schedule = last("schedule_database_state")
        astar = [event for event in events if event.get("event") == "astar_trace_summary"]
        final_db_phase = str(schedule.get("phase", ""))
        dynamic_stages = [
            event for event in events
            if event.get("event") == "dynamic_insertion_result"
        ]
        policy_expansions = [
            event for event in events
            if event.get("event") == "corridor_policy_expansion"]
        snapshot_events = [
            event for event in events
            if event.get("event") == "corridor_policy_snapshot"]
        return {
            "Core label": profile.get("label", ""),
            "RMF commit": profile.get("rmf_source_commit", ""),
            "RMF source dirty": profile.get("rmf_source_dirty", ""),
            "RMF diff SHA-256": profile.get("rmf_source_diff_sha256", ""),
            "RMF library": profile.get("resolved_rmf_library", ""),
            "Scenario SHA-256": profile.get("scenario_sha256", ""),
            "Lane penalty active": profile.get("lane_penalty_active", False),
            "Lane penalty mode": profile.get("lane_penalty_mode", ""),
            "Lane penalty value": profile.get("lane_penalty_value", ""),
            "Penalized directed lanes": profile.get("penalized_lane_count", 0),
            "Penalty lane values": profile.get("directed_lane_penalties", {}),
            "Predicted lane occupancy": profile.get("directed_lane_occupancy", {}),
            "Shared corridor robots": profile.get("shared_corridor_users", {}),
            "Traffic policy mode": profile.get("traffic_policy_mode", "baseline"),
            "Policy weights": profile.get("policy_weights", {}),
            "Policy expansion rows": len(policy_expansions),
            "Soft penalized children": sum(
                event.get("decision") == "SOFT_PENALIZED"
                for event in policy_expansions),
            "Hard admission blocks": sum(
                event.get("decision") == "HARD_CORRIDOR_BLOCK"
                for event in policy_expansions),
            "Total policy cost observed": sum(
                float(event.get("total_policy_penalty", 0.0) or 0.0)
                for event in policy_expansions),
            "Schedule snapshot count": len(snapshot_events),
            "Schedule snapshot version": (
                snapshot_events[-1].get("schedule_version", "") if snapshot_events else ""),
            "Schedule query count": sum(
                int(event.get("schedule_query_count", 0) or 0)
                for event in snapshot_events),
            "Queried participant count": sum(
                int(event.get("queried_participant_count", 0) or 0)
                for event in snapshot_events),
            "Queried route count": sum(
                int(event.get("queried_route_count", 0) or 0)
                for event in snapshot_events),
            "Self-filtered route count": sum(
                int(event.get("self_filtered_route_count", 0) or 0)
                for event in snapshot_events),
            "Overlap checks": sum(
                int(event.get("overlap_check_count", 0) or 0)
                for event in policy_expansions),
            "Solution": diagnosis.get("status", ""),
            "Diagnosis": diagnosis.get("category", ""),
            "Negotiation success": negotiation.get("success", ""),
            "Executable negotiation": negotiation.get("executable_plan", ""),
            "Safety verified": negotiation.get("safety_verified", ""),
            "Negotiated plan count": negotiation.get("proposal_plan_count", ""),
            "Negotiation ms": negotiation.get("elapsed_ms", ""),
            "Plan total cost": sum(float(event.get("cost", 0)) for event in plans),
            "Final completion s": max(finish_map.values(), default=0.0),
            "Explicit stationary wait s": sum(wait_map.values()),
            "Final robot lanes": lane_map_text(route_map),
            "Baseline robot lanes": lane_map_text(baseline_route_map),
            "Final route distance m": route_distance(route_map),
            "Baseline route distance m": route_distance(baseline_route_map),
            "Negotiation rerouted robots": len(negotiation_rerouted),
            "Negotiation rescheduled robots": len(negotiation_rescheduled),
            "Per-robot outcome": " | ".join(robot_outcomes) or "실행 경로 없음",
            "Candidate evidence": candidate_evidence,
            "Requested start times": " | ".join(
                f"{robot}:{start:g}s"
                for robot, start in sorted(requested_start_map.items())
            ) or "기록 없음",
            "Dynamic insertion times": " | ".join(
                f"{robot}:{insertion:g}s"
                for robot, insertion in sorted(insertion_time_map.items())
            ) or "기록 없음",
            "Dynamic stages": len(dynamic_stages),
            "Dynamic successful stages": sum(
                bool(event.get("success")) for event in dynamic_stages),
            "A* expansions (debug baselines)": sum(int(event.get("expansions", 0)) for event in astar),
            "Schedule DB version": schedule.get("latest_version", ""),
            "Schedule final phase": final_db_phase,
            "Stored routes": schedule.get("stored_route_count", ""),
            "Stored trajectory points": len([
                event for event in events
                if event.get("event") == "schedule_database_trajectory_point"
                and event.get("phase") == final_db_phase]),
            "_route_map": route_map,
            "_baseline_route_map": baseline_route_map,
            "_finish_map": finish_map,
            "_baseline_finish_map": baseline_finish_map,
            "_events": events,
        }

    def update_comparison(self) -> None:
        baseline = self.summarize_result(ROOT / "results" / "gui_baseline.jsonl")
        soft = self.summarize_result(ROOT / "results" / "gui_soft.jsonl")
        schedule_soft = self.summarize_result(ROOT / "results" / "gui_schedule_soft.jsonl")
        hybrid = self.summarize_result(ROOT / "results" / "gui_hybrid.jsonl")
        hybrid_nego = self.summarize_result(
            ROOT / "results" / "gui_hybrid_nego.jsonl")
        metrics = [
            ("코어 라벨", "Core label"),
            ("동일 시나리오 확인용 SHA-256", "Scenario SHA-256"),
            ("실제 로딩된 RMF 라이브러리", "RMF library"),
            ("RMF 소스 commit", "RMF commit"),
            ("소스 수정 여부", "RMF source dirty"),
            ("Lane penalty 실제 활성", "Lane penalty active"),
            ("Lane penalty 모드", "Lane penalty mode"),
            ("Lane당 penalty", "Lane penalty value"),
            ("Penalty 적용 directed Lane 수", "Penalized directed lanes"),
            ("Directed Lane별 penalty", "Penalty lane values"),
            ("Directed Lane별 예상 점유 대수", "Predicted lane occupancy"),
            ("공유 통로별 예상 로봇", "Shared corridor robots"),
            ("Traffic policy mode", "Traffic policy mode"),
            ("Policy weights", "Policy weights"),
            ("Policy expansion 행", "Policy expansion rows"),
            ("Soft penalty child 수", "Soft penalized children"),
            ("Hard admission block 수", "Hard admission blocks"),
            ("관찰된 policy cost 합", "Total policy cost observed"),
            ("Schedule snapshot 수", "Schedule snapshot count"),
            ("마지막 Schedule snapshot version", "Schedule snapshot version"),
            ("Schedule DB query 수", "Schedule query count"),
            ("조회 participant 누적", "Queried participant count"),
            ("조회 route 누적", "Queried route count"),
            ("Self route 제외 수", "Self-filtered route count"),
            ("Candidate overlap 검사 수", "Overlap checks"),
            ("로봇별 요청 출발 시각", "Requested start times"),
            ("로봇별 동적 투입 시각", "Dynamic insertion times"),
            ("동적 투입 stage 수", "Dynamic stages"),
            ("성공한 동적 stage 수", "Dynamic successful stages"),
            ("최종 해 상태", "Solution"),
            ("진단 분류", "Diagnosis"),
            ("협상 성공", "Negotiation success"),
            ("실행 가능한 협상안", "Executable negotiation"),
            ("연속시간 안전검사", "Safety verified"),
            ("협상 계획 수", "Negotiated plan count"),
            ("협상 계산시간(ms)", "Negotiation ms"),
            ("최종 로봇별 Lane", "Final robot lanes"),
            ("최종 경로 총거리(m)", "Final route distance m"),
            ("자유경로 기준 Lane", "Baseline robot lanes"),
            ("자유경로 기준 총거리(m)", "Baseline route distance m"),
            ("협상으로 우회한 로봇 수", "Negotiation rerouted robots"),
            ("협상으로 시간조정된 로봇 수", "Negotiation rescheduled robots"),
            ("로봇별 우회·시간조정 결과", "Per-robot outcome"),
            ("자유경로 후보 비용 근거", "Candidate evidence"),
            ("명시적 정지 대기시간 합(s)", "Explicit stationary wait s"),
            ("전체 완료시간(s)", "Final completion s"),
            ("최종 계획비용 합", "Plan total cost"),
            ("A* 확장 수", "A* expansions (debug baselines)"),
            ("Schedule DB 최종 단계", "Schedule final phase"),
            ("DB 저장 Route", "Stored routes"),
            ("DB 저장 Trajectory point", "Stored trajectory points"),
        ]
        rows: list[list[object]] = []
        changed_robots: list[str] = []
        comparison_target = hybrid_nego or hybrid or schedule_soft or soft
        if baseline and comparison_target:
            before_routes = baseline.get("_route_map", {})
            after_routes = comparison_target.get("_route_map", {})
            changed_robots = sorted(
                robot for robot in set(before_routes) | set(after_routes)
                if before_routes.get(robot) != after_routes.get(robot)
            )
        for label, key in metrics:
            left = "" if baseline is None else baseline.get(key, "")
            soft_value = "" if soft is None else soft.get(key, "")
            schedule_soft_value = "" if schedule_soft is None else schedule_soft.get(key, "")
            hybrid_value = "" if hybrid is None else hybrid.get(key, "")
            nego_value = "" if hybrid_nego is None else hybrid_nego.get(key, "")
            target = "" if comparison_target is None else comparison_target.get(key, "")
            delta: object = ""
            if isinstance(left, (int, float)) and isinstance(target, (int, float)):
                delta = round(float(target) - float(left), 6)
            elif key == "Final robot lanes" and baseline and comparison_target:
                delta = f"{len(changed_robots)}대 경로 변경" if changed_robots else "Lane 동일"
            elif key == "Scenario SHA-256" and baseline and comparison_target:
                delta = "동일 입력" if left == target and left else "입력 다름"
            rows.append([label, left, soft_value, schedule_soft_value, hybrid_value, nego_value, delta])
        self.set_table_rows(self.compare_table, rows)
        self.compare_explanation.setPlainText(
            self.build_comparison_explanation(
                baseline, soft, schedule_soft, hybrid, hybrid_nego, changed_robots))

    @staticmethod
    def build_comparison_explanation(
        before: dict[str, object] | None,
        soft: dict[str, object] | None,
        schedule_soft: dict[str, object] | None,
        hybrid: dict[str, object] | None,
        hybrid_nego: dict[str, object] | None,
        changed_robots: list[str],
    ) -> str:
        target = hybrid_nego or hybrid or schedule_soft or soft
        target_name = (
            "HYBRID+NEGO" if hybrid_nego is not None else
            "HYBRID" if hybrid is not None else
            "SCHEDULE_SOFT" if schedule_soft is not None else "OLD_SOFT")
        if before is None or target is None:
            return (
                "BASELINE/OLD_SOFT/SCHEDULE_SOFT/HYBRID/HYBRID+NEGO 결과 해석\n\n"
                "동일 시나리오를 각 모드로 실행하면 기존 Soft, 새 Schedule Snapshot Soft, hard admission, 신규 협상 효과를 분리해서 표시합니다.\n\n"
                "판정 순서\n"
                "1. Scenario SHA-256 동일 여부\n2. 실제 RMF 라이브러리 경로 차이\n"
                "3. 실행 가능한 해와 안전검사\n4. 로봇별 Lane 변경\n5. 대기·완료시간·비용 변화\n"
                "6. 검증된 Route의 Schedule DB commit 여부"
            )

        same_scenario = (
            bool(before.get("Scenario SHA-256"))
            and before.get("Scenario SHA-256") == target.get("Scenario SHA-256"))
        core_changed = any([
            before.get("RMF library") != target.get("RMF library"),
            before.get("RMF commit") != target.get("RMF commit"),
            before.get("RMF diff SHA-256") != target.get("RMF diff SHA-256"),
        ])
        before_solved = before.get("Solution") == "solved"
        after_solved = target.get("Solution") == "solved"
        before_routes = before.get("_route_map", {})
        after_routes = target.get("_route_map", {})

        lines = [
            f"Before/{target_name} 성과 판정",
            "",
            f"1. 입력 동일성: {'통과 · 같은 시나리오' if same_scenario else '확인 필요 · Scenario SHA가 다름'}",
            f"2. 코어 분리: {'변경 확인' if core_changed else '동일 코어일 가능성 · library/commit/diff를 확인'}",
        ]
        if not before_solved and after_solved:
            lines.append(f"3. 해 상태: 개선 · Before 해 없음 → {target_name} 실행 가능한 해 생성")
        elif before_solved and not after_solved:
            lines.append(f"3. 해 상태: 악화 · Before 성공 → {target_name} 해 없음")
        else:
            lines.append(
                f"3. 해 상태: Before={before.get('Solution')} / {target_name}={target.get('Solution')}")

        if changed_robots:
            lines.append("4. 경로 재생성: 확인됨")
            for robot in changed_robots:
                lines.append(
                    f"   • {robot}: {before_routes.get(robot, '없음')} → {after_routes.get(robot, '없음')}")
        else:
            lines.append("4. 경로 재생성: Lane 조합 변화 없음")

        rerouted = int(target.get("Negotiation rerouted robots", 0) or 0)
        rescheduled = int(target.get("Negotiation rescheduled robots", 0) or 0)
        before_distance = float(before.get("Final route distance m", 0) or 0)
        after_distance = float(target.get("Final route distance m", 0) or 0)
        longer_route = after_distance > before_distance + 1e-6
        lines.extend([
            f"5. {target_name} 협상 조정: 우회 {rerouted}대 · 동일 Lane 시간조정 {rescheduled}대",
            f"6. 안전·실행: negotiation={target.get('Executable negotiation')} · "
            f"DetectConflict={target.get('Safety verified')} · DB Route={target.get('Stored routes')}",
            f"7. 우회 거리: {before_distance:.3f}→{after_distance:.3f} m "
            f"({'더 긴 우회경로 확인' if longer_route else '거리 증가 없음'})",
            f"8. 성능 변화: 완료시간 {before.get('Final completion s')}→{target.get('Final completion s')} s · "
            f"비용 {before.get('Plan total cost')}→{target.get('Plan total cost')} · "
            f"A* 확장 {before.get('A* expansions (debug baselines)')}→{target.get('A* expansions (debug baselines)')}",
            "",
            "어떻게 해석하나",
            f"• ‘우회 성공’은 {target_name}의 최종 Lane이 자기 free-flow 기준 Lane과 달라졌고, 동시에 실행 가능·안전검사 통과·DB commit까지 된 경우입니다.",
            f"• ‘경로 재생성’은 동일 입력에서 Before와 {target_name}의 최종 Lane 조합이 달라진 경우입니다.",
            f"• ‘더 긴 우회경로’는 Lane 조합 변화와 함께 {target_name}의 기하학적 총거리가 늘어난 경우입니다. 수정 비용이 커지는 것은 의도된 penalty이므로 거리와 안전을 함께 봅니다.",
            "• Lane이 같아도 완료시간이나 정지시간이 달라지면 협상이 경로 대신 시간축을 조정한 것입니다.",
            "• 비용 감소만으로 개선이라 단정하지 않습니다. 충돌 없음과 모든 로봇 도착을 먼저 확인해야 합니다.",
            "• 명시적 정지시간은 같은 x·y·yaw에서 시간이 증가한 waypoint만 합산합니다. 회전시간은 제외됩니다.",
        ])
        return "\n".join(lines)

    def prepare_animation(self) -> None:
        self.scene.clear_robot_overlay()
        grouped: dict[tuple[str, str], list[dict]] = {}
        for event in self.events:
            if event.get("event") == "trajectory_point":
                grouped.setdefault((str(event.get("robot")), str(event.get("phase"))), []).append(event)
        phases = {phase for _, phase in grouped}
        preferred = "negotiated" if "negotiated" in phases else "free_flow"
        self.trajectories = {}
        for (robot, phase), points in grouped.items():
            if phase == preferred or (preferred == "free_flow" and len(grouped) == 1):
                self.trajectories[robot] = sorted(points, key=lambda point: float(point.get("time_s", 0)))
        for index, robot in enumerate(self.document.get("robots", [])):
            if str(robot.get("name", f"R{index}")) not in self.trajectories:
                node = self.document["nodes"][robot["start"]]
                appearance_time = max(
                    float(robot.get("start_time_s", 0) or 0),
                    float(robot.get("insertion_time_s", 0) or 0))
                self.trajectories[str(robot.get("name", f"R{index}"))] = [{
                    "time_s": appearance_time, "x": node["x"], "y": node["y"],
                    "yaw_rad": robot.get("yaw", 0.0), "vx": 0.0, "vy": 0.0,
                    "vyaw": 0.0, "fallback": True,
                }]
        for index, (_robot, points) in enumerate(sorted(self.trajectories.items())):
            if len(points) == 1 and points[0].get("fallback"):
                continue
            self.scene.set_robot_trajectory(
                points, ROBOT_COLORS[index % len(ROBOT_COLORS)])

        self.animation_decisions = sorted([
            event for event in self.events
            if event.get("event") == "plan_waypoint"
            and event.get("phase") == preferred
        ], key=lambda event: (float(event.get("time_s", 0)), int(event.get("seq", 0))))

        def latest(kind: str, robot: str | None = None, phase: str | None = None) -> dict:
            matches = [
                event for event in self.events
                if event.get("event") == kind
                and (robot is None or str(event.get("robot", event.get("name", ""))) == robot)
                and (phase is None or event.get("phase") == phase)
            ]
            return matches[-1] if matches else {}

        negotiation = latest("negotiation_summary")
        safety = latest("safety_verification")
        schedule_state = latest("schedule_database_state")
        negotiation_log_count = sum(
            event.get("event") == "negotiation_log" for event in self.events)
        self.live_robot_context = {}
        for robot in self.trajectories:
            final_plan = latest("plan_summary", robot, preferred)
            baseline = latest("plan_summary", robot, "free_flow_baseline")
            route_choice = latest("route_choice_explanation", robot)
            planning_request = latest("planning_request", robot)
            participants = [
                event for event in self.events
                if event.get("event") == "schedule_participant"
                and str(event.get("name", "")) == robot
            ]
            participant = participants[-1] if participants else {}
            final_lanes = list(final_plan.get("used_lanes", []))
            baseline_lanes = list(baseline.get("used_lanes", []))
            if baseline and final_lanes != baseline_lanes:
                adjustment = f"협상 우회 {baseline_lanes} → {final_lanes}"
            elif baseline:
                finish_delta = float(final_plan.get("finish_time_s", 0) or 0) - float(
                    baseline.get("finish_time_s", 0) or 0)
                adjustment = (
                    f"Lane 유지·시간축 조정 {finish_delta:+.3f}s"
                    if abs(finish_delta) > 1e-6 else "자유경로 기준 유지")
            else:
                adjustment = "단일 로봇 계획 · 협상 조정 없음"
            self.live_robot_context[robot] = {
                "phase": preferred,
                "final_plan": final_plan,
                "baseline": baseline,
                "route_choice": route_choice,
                "planning_request": planning_request,
                "participant": participant,
                "negotiation": negotiation,
                "safety": safety,
                "schedule_state": schedule_state,
                "negotiation_log_count": negotiation_log_count,
                "adjustment": adjustment,
            }
        self.animation_end = max((float(point.get("time_s", 0)) for points in self.trajectories.values() for point in points), default=0)
        self.animation_time = 0; self.time_slider.setValue(0)
        self.last_animation_decision_seq = None
        self.render_animation()

    @staticmethod
    def interpolate(points: list[dict], t: float) -> dict[str, float | str]:
        def pose(point: dict, state: str) -> dict[str, float | str]:
            if point.get("fallback"):
                state = "실행 경로 없음·정지"
            return {
                "x": float(point["x"]), "y": float(point["y"]),
                "yaw": float(point.get("yaw_rad", 0)),
                "vx": float(point.get("vx", 0)), "vy": float(point.get("vy", 0)),
                "vyaw": float(point.get("vyaw", 0)), "state": state,
            }

        if t <= float(points[0].get("time_s", 0)):
            return pose(points[0], "출발 대기")
        if t >= float(points[-1].get("time_s", 0)):
            return pose(points[-1], "도착")
        for left, right in zip(points, points[1:]):
            t0, t1 = float(left["time_s"]), float(right["time_s"])
            if t0 <= t <= t1:
                ratio = (t - t0) / max(t1 - t0, 1e-9)
                yaw0 = float(left.get("yaw_rad", 0))
                yaw1 = float(right.get("yaw_rad", yaw0))
                yaw_delta = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
                values = {
                    "x": float(left["x"]) + ratio * (float(right["x"]) - float(left["x"])),
                    "y": float(left["y"]) + ratio * (float(right["y"]) - float(left["y"])),
                    "yaw": yaw0 + ratio * yaw_delta,
                    "vx": float(left.get("vx", 0)) + ratio * (float(right.get("vx", 0)) - float(left.get("vx", 0))),
                    "vy": float(left.get("vy", 0)) + ratio * (float(right.get("vy", 0)) - float(left.get("vy", 0))),
                    "vyaw": float(left.get("vyaw", 0)) + ratio * (float(right.get("vyaw", 0)) - float(left.get("vyaw", 0))),
                }
                linear_speed = math.hypot(float(values["vx"]), float(values["vy"]))
                if linear_speed > 0.02:
                    values["state"] = "이동"
                elif abs(float(values["vyaw"])) > 0.02 or abs(yaw_delta) > 0.02:
                    values["state"] = "제자리 회전"
                else:
                    values["state"] = "대기"
                return values
        return pose(points[-1], "도착")

    def render_animation(self) -> None:
        status_lines = []
        for index, (robot, points) in enumerate(sorted(self.trajectories.items())):
            context = self.live_robot_context.get(robot, {})
            request = context.get("planning_request", {})
            insertion_time = float(request.get("insertion_time_s", 0) or 0)
            pose = self.interpolate(points, self.animation_time)
            self.scene.set_robot_pose(
                robot, float(pose["x"]), float(pose["y"]), float(pose["yaw"]),
                str(pose["state"]), ROBOT_COLORS[index % len(ROBOT_COLORS)])
            marker = self.scene.robot_items.get(robot)
            if marker is not None:
                marker.setVisible(
                    insertion_time <= 0 or self.animation_time + 1e-6 >= insertion_time)
            if insertion_time > self.animation_time + 1e-6:
                status_lines.append(
                    f"{robot}: 미투입 · t={insertion_time:g}s에 Schedule DB 등록")
                continue
            status_lines.append(
                f"{robot}: {pose['state']} · ({float(pose['x']):.2f}, {float(pose['y']):.2f}) · "
                f"yaw {math.degrees(float(pose['yaw'])):+.0f}°"
            )
        self.time_label.setText(
            f"{self.animation_time:.2f} / {self.animation_end:.2f} s · {self.playback_speed:g}x")
        self.sync_decision_to_animation(status_lines)

    def sync_decision_to_animation(self, status_lines: list[str]) -> None:
        active = [
            event for event in self.animation_decisions
            if float(event.get("time_s", 0)) <= self.animation_time + 1e-6
        ]
        upcoming = [
            event for event in self.animation_decisions
            if float(event.get("time_s", 0)) > self.animation_time + 1e-6
        ]
        current = upcoming[0] if upcoming else (active[-1] if active else None)
        shown_status = status_lines[:3]
        if len(status_lines) > 3:
            shown_status.append(f"외 {len(status_lines) - 3}대")
        position_text = " | ".join(shown_status) if shown_status else "실행 가능한 궤적 없음"
        detail_lines = [f"현재 t={self.animation_time:.2f}s · {position_text}"]
        if current:
            robot = str(current.get("robot", ""))
            context = self.live_robot_context.get(robot, {})
            movement = str(current.get("movement_type", ""))
            movement_label = {
                "start": "출발 자세 확정",
                "rotate_in_place": "후진 대신 제자리 회전",
                "wait": "충돌 회피를 위한 시간 대기",
                "forward_traverse": "정렬된 방향으로 전진",
            }.get(movement, "최종 궤적 이동")
            reason = {
                "start": "Planner 요청의 시작 위치·yaw를 사용합니다.",
                "rotate_in_place": "로봇이 후진 불가이므로 다음 Lane 방향과 맞춘 뒤 전진합니다.",
                "wait": "위치를 유지하고 시간만 진행해 이벤트 또는 협상 시공간 제약을 맞춥니다.",
                "forward_traverse": "최종 RMF plan에 채택된 접근 Lane을 전진으로 통과합니다.",
            }.get(movement, "실제 RMF 최종 plan의 waypoint와 trajectory를 재생합니다.")
            target_kind = "다음 목표" if upcoming else "도달한 목표"
            detail_lines.append(
                f"{target_kind} · {robot} waypoint #{current.get('sequence')} · "
                f"노드={current.get('graph_index')} · Lane={current.get('approach_lanes', [])} · "
                f"{movement_label} (Δt {float(current.get('delta_time_s', 0) or 0):.3f}s, "
                f"Δ거리 {float(current.get('delta_distance_m', 0) or 0):.3f}m, "
                f"Δyaw {math.degrees(float(current.get('delta_yaw_rad', 0) or 0)):+.1f}°)")
            detail_lines.append(f"이동 근거 · {reason}")

            plan = context.get("final_plan", {})
            choice = context.get("route_choice", {})
            planning_request = context.get("planning_request", {})
            detail_lines.append(
                f"경로 근거 · 요청 출발 t={planning_request.get('start_time_s', 0)}s, "
                f"최종 Lane={plan.get('used_lanes', [])}, 비용={plan.get('cost', '?')}, "
                f"자유경로 후보 {choice.get('selected_rank', '?')}/{choice.get('candidate_count', '?')}위, "
                f"차순위 비용차={choice.get('cost_margin', '?')} · {context.get('adjustment', '')}")

            negotiation = context.get("negotiation", {})
            safety = context.get("safety", {})
            participant = context.get("participant", {})
            schedule = context.get("schedule_state", {})
            if negotiation:
                negotiation_text = (
                    f"협상 · executable={negotiation.get('executable_plan')} · "
                    f"안전검사={safety.get('passed', negotiation.get('safety_verified'))} · "
                    f"내부 로그 {context.get('negotiation_log_count', 0)}건")
            else:
                negotiation_text = "협상 · 단일 로봇 계획이므로 CentralizedNegotiation 생략"
            detail_lines.append(
                f"{negotiation_text} · Schedule DB participant={participant.get('participant_id', '?')}, "
                f"plan={participant.get('current_plan_id', '?')}, itinerary v{participant.get('itinerary_version', '?')}, "
                f"DB v{schedule.get('latest_version', '?')} ({schedule.get('phase', '기록 전')})")
            seq = current.get("seq")
            if seq != self.last_animation_decision_seq:
                self.last_animation_decision_seq = seq
                row = self.decision_row_by_seq.get(seq)
                if row is not None:
                    self.decision_table.selectRow(row)
                    item = self.decision_table.item(row, 0)
                    if item:
                        self.decision_table.scrollToItem(item)
        else:
            diagnosis = next(
                (event for event in reversed(self.events)
                 if event.get("event") == "solution_diagnosis"), {})
            if diagnosis:
                detail_lines.append(
                    f"진단 · status={diagnosis.get('status')} · category={diagnosis.get('category')} · "
                    "실행 경로가 없으므로 로봇을 움직이지 않습니다.")
        self.current_decision_label.setText("\n".join(detail_lines))

    def toggle_animation(self) -> None:
        if self.animation_timer.isActive(): self.animation_timer.stop(); self.play_button.setText("▶ 재생")
        else:
            if self.animation_time >= self.animation_end: self.animation_time = 0
            self.animation_timer.start(); self.play_button.setText("⏸ 일시정지")

    def playback_speed_changed(self, _index: int) -> None:
        value = self.playback_speed_combo.currentData()
        try:
            self.playback_speed = float(value)
        except (TypeError, ValueError):
            self.playback_speed = 1.0
        self.render_animation()
        self.statusBar().showMessage(
            f"화면 재생 배속 {self.playback_speed:g}x · RMF 궤적 시각값은 변경하지 않습니다",
            2500,
        )

    def advance_animation(self) -> None:
        elapsed_per_tick = self.animation_timer.interval() / 1000.0
        self.animation_time = min(
            self.animation_end,
            self.animation_time + elapsed_per_tick * self.playback_speed,
        )
        self.time_slider.blockSignals(True)
        self.time_slider.setValue(int(1000 * self.animation_time / self.animation_end) if self.animation_end else 0)
        self.time_slider.blockSignals(False); self.render_animation()
        if self.animation_time >= self.animation_end: self.toggle_animation()

    def slider_changed(self, value: int) -> None:
        self.animation_time = self.animation_end * value / 1000.0
        self.render_animation()

    def closeEvent(self, event) -> None:
        self.jsonl_timer.stop()
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning: self.process.terminate()
        if (self.regression_process
                and self.regression_process.state() != QProcess.ProcessState.NotRunning):
            self.regression_process.terminate()
        self.settings.setValue("window/geometry", self.saveGeometry())
        self.settings.setValue("window/maximized", self.isMaximized())
        self.settings.setValue("splitter/map_editor_v2", self.main_splitter.saveState())
        self.settings.setValue("splitter/map_output_v2", self.vertical_splitter.saveState())
        self.settings.sync()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    font_loaded, font_family = configure_korean_font(app)
    app.setStyleSheet(APP_STYLE)
    window = MainWindow()
    if window.start_maximized:
        window.showMaximized()
    else:
        window.show()
    if font_loaded:
        window.statusBar().showMessage(
            f"한글 글꼴 적용: {font_family} · 시나리오를 편집한 뒤 변경사항 빌드 후 RMF 분석을 누르세요"
        )
    else:
        QMessageBox.warning(
            window,
            "Korean font missing",
            "Bundled Korean font could not be loaded.\n"
            "Run: sudo apt install -y fonts-noto-cjk",
        )
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
