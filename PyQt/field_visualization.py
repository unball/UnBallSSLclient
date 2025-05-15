from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QFrame, QGraphicsScene, QGraphicsView, QVBoxLayout
from PyQt5.QtGui import QPainter
from PyQt5.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsLineItem,
    QGraphicsTextItem,
    QGraphicsPathItem,
)
from PyQt5.QtCore import Qt, QRectF, QLineF
from PyQt5.QtGui import QPen, QBrush, QColor
import math
from typing import List, Tuple


class FieldVisualization(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)

        # Field margin for better appearance
        self.field_margin = 30  # pixels

        # Initialize el_bounds
        self.el_bounds = {
            "x_min": -3.0,  # Increased from -2.747 to allow more space
            "x_max": 3.0,  # Increased from 2.747
            "y_min": -2.5,  # Increased from -1.971
            "y_max": 2.5,  # Increased from 1.971
        }

        # Make view more adaptive
        self.view.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )

        # Better rendering settings
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setRenderHint(QPainter.SmoothPixmapTransform)
        self.view.setBackgroundBrush(
            QBrush(QColor(0, 80, 0))
        )  # Darker green for border

        # Remove scrollbars
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.view)

        # Updated robot tracking - now using dictionaries to store all graphics items for each robot
        self.blue_robot_items = (
            {}
        )  # robot_id -> {'shape': QGraphicsEllipseItem, 'line': QGraphicsLineItem, 'text': QGraphicsTextItem}
        self.yellow_robot_items = {}
        self.ball_item = None
        self.path_graphics_items = {}  # robot_id -> QGraphicsPathItem

        # Keep existing visibility settings
        self.blue_visible = True
        self.yellow_visible = True

        # Path visualization
        self._show_paths = False
        self.paths = {}  # robot_id -> path data

        # Add division-specific dimensions
        self.divisions = {
            "Division A": {
                "field_width": 12.0,  # Length (x-axis)
                "field_height": 9.0,  # Width (y-axis)
                "defense_width": 3.6,
                "defense_height": 1.8,
                "goal_width": 1.8,
                "goal_depth": 0.18,
                "max_robots": 11,
            },
            "Division B": {
                "field_width": 9.0,
                "field_height": 6.0,
                "defense_width": 2.0,
                "defense_height": 1.0,
                "goal_width": 1.0,
                "goal_depth": 0.18,
                "max_robots": 6,
            },
            "Entry Level": {
                "field_width": 4.5,
                "field_height": 3.0,
                "defense_width": 1.35,
                "defense_height": 0.5,
                "goal_width": 0.8,
                "goal_depth": 0.18,
                "max_robots": 3,
            },
        }

        self.current_division = "Division A"
        self.setup_field()

        # Force a resize to trigger proper rendering
        self.resizeEvent(None)

    def set_show_paths(self, show: bool):
        """Enable/disable path visualization"""
        try:
            if self._show_paths != show:  # Only update if value is changing
                # Clear existing path visualization
                self.clear_path_visualization()

                # Update flag
                self._show_paths = show
                print(f"A* visualization: {'enabled' if show else 'disabled'}")

                # If enabling paths, draw all paths
                if show:
                    self._redraw_all_paths()

        except Exception as e:
            print(f"Error setting show paths: {e}")

    def _redraw_all_paths(self):
        """Redraw all stored paths"""
        for robot_id, path_data in self.paths.items():
            self._update_or_create_path_item(robot_id, path_data)

    def clear_path_visualization(self, robot_id=None):
        """Clear path visualization items safely"""
        try:
            if robot_id is not None:
                # Clear for specific robot
                if robot_id in self.path_graphics_items:
                    if (
                        self.scene
                        and self.path_graphics_items[robot_id].scene() == self.scene
                    ):
                        self.scene.removeItem(self.path_graphics_items[robot_id])
                    del self.path_graphics_items[robot_id]
            else:
                # Clear for all robots
                for r_id in list(self.path_graphics_items.keys()):
                    if (
                        self.scene
                        and self.path_graphics_items[r_id].scene() == self.scene
                    ):
                        self.scene.removeItem(self.path_graphics_items[r_id])
                self.path_graphics_items.clear()
        except Exception as e:
            print(f"Error clearing path visualization: {e}")
            # Reset path items tracking to recover from error
            self.path_graphics_items = {}

    def update_path(self, robot_id: int, path: List[Tuple[float, float]]):
        """Update stored path and visualization for a robot."""
        try:
            if path is None or len(path) < 2:
                if robot_id in self.paths:
                    del self.paths[robot_id]
                self.clear_path_visualization(robot_id)  # Remove graphics item
                return

            self.paths[robot_id] = list(path)  # Store the path
            if self._show_paths:
                self._update_or_create_path_item(robot_id, path)
            else:  # If paths are not shown, still clear any old graphics item
                self.clear_path_visualization(robot_id)

        except Exception as e:
            print(f"Error updating path for robot {robot_id}: {e}")

    def _update_or_create_path_item(
        self, robot_id: int, path_data: List[Tuple[float, float]]
    ):
        """Creates or updates a QGraphicsPathItem for the robot's path."""
        try:
            if not path_data or len(path_data) < 2:
                self.clear_path_visualization(robot_id)
                return

            painter_path = QtGui.QPainterPath()
            start_x_viz, start_y_viz = self.scale_coordinates(
                path_data[0][0], path_data[0][1]
            )
            painter_path.moveTo(start_x_viz, start_y_viz)
            for i in range(1, len(path_data)):
                x_viz, y_viz = self.scale_coordinates(path_data[i][0], path_data[i][1])
                painter_path.lineTo(x_viz, y_viz)

            # Determine color based on robot team (check if robot exists in blue items)
            is_blue = robot_id in self.blue_robot_items
            path_color = (
                QColor(0, 100, 255, 180) if is_blue else QColor(255, 255, 0, 180)
            )  # Slightly more visible
            path_pen = QPen(path_color, 2, Qt.DashLine)

            if robot_id in self.path_graphics_items:
                self.path_graphics_items[robot_id].setPath(painter_path)
                self.path_graphics_items[robot_id].setPen(
                    path_pen
                )  # Update pen in case team color changed
            else:
                path_item = self.scene.addPath(painter_path, path_pen)
                self.path_graphics_items[robot_id] = path_item

            self.path_graphics_items[robot_id].setVisible(self._show_paths)

        except Exception as e:
            print(f"Error creating/updating path item for robot {robot_id}: {e}")

    def resizeEvent(self, event):
        """Handle resize events"""
        super().resizeEvent(event)
        self.setup_field()

    def setup_field(self):
        """Setup the field visualization"""
        print("Setting up field...")  # Debug print
        dims = self.divisions[self.current_division]

        try:
            # Clear existing path visualization first
            self.clear_path_visualization()

            # Clear scene before any setup
            self.scene.clear()

            # Clear path items tracking since scene.clear() removed them
            self.path_graphics_items = {}
            # Clear robot items tracking
            self.blue_robot_items = {}
            self.yellow_robot_items = {}
            self.ball_item = None

            # Calculate basic dimensions
            self.field_margin = 30
            field_aspect = dims["field_width"] / dims["field_height"]
            frame_width = self.width() - 2 * self.field_margin
            frame_height = self.height() - 2 * self.field_margin
            frame_aspect = frame_width / frame_height

            # Calculate visualization dimensions maintaining aspect ratio
            if frame_aspect > field_aspect:
                self.viz_height = frame_height
                self.viz_width = self.viz_height * field_aspect
            else:
                self.viz_width = frame_width
                self.viz_height = self.viz_width / field_aspect

            # Setup scene dimensions based on division
            if self.current_division == "Entry Level":
                # Extended margins for Entry Level
                goal_margin = 0
                side_margin = 0
                height_margin = 80

                # Set scene rect with extended margins
                total_width = self.viz_width + 2 * (side_margin + goal_margin)
                total_height = self.viz_height + 2 * height_margin

                self.scene.setSceneRect(
                    -(side_margin + goal_margin),
                    -height_margin,
                    total_width,
                    total_height,
                )
            else:
                # Standard margins for other divisions
                total_width = self.viz_width + 2 * self.field_margin
                total_height = self.viz_height + 2 * self.field_margin
                self.scene.setSceneRect(
                    -self.field_margin, -self.field_margin, total_width, total_height
                )

            # Field drawing colors
            field_color = QColor(0, 150, 0)  # Main field color
            line_color = Qt.white
            field_pen = QPen(line_color, 2)

            # Draw main field rectangle
            self.scene.addRect(
                0, 0, self.viz_width, self.viz_height, field_pen, QBrush(field_color)
            )

            # Draw center line
            self.scene.addLine(
                self.viz_width / 2, 0, self.viz_width / 2, self.viz_height, field_pen
            )

            # Draw center circle
            circle_radius = (0.5 * self.viz_width) / dims["field_width"]
            center_x = self.viz_width / 2
            center_y = self.viz_height / 2
            self.scene.addEllipse(
                center_x - circle_radius,
                center_y - circle_radius,
                circle_radius * 2,
                circle_radius * 2,
                field_pen,
                QBrush(Qt.transparent),
            )

            # Draw goals
            goal_width = (dims["goal_width"] * self.viz_width) / dims["field_width"]
            goal_depth = (dims["goal_depth"] * self.viz_width) / dims["field_width"]
            goal_margin_y = (self.viz_height - goal_width) / 2

            # Left goal
            self.scene.addRect(
                -goal_depth,
                goal_margin_y,
                goal_depth,
                goal_width,
                QPen(Qt.black),
                QBrush(Qt.transparent),
            )

            # Right goal
            self.scene.addRect(
                self.viz_width,
                goal_margin_y,
                goal_depth,
                goal_width,
                QPen(Qt.black),
                QBrush(Qt.transparent),
            )

            # Draw defense areas
            defense_width = (dims["defense_height"] * self.viz_width) / dims[
                "field_width"
            ]  # Using height for width
            defense_height = (dims["defense_width"] * self.viz_height) / dims[
                "field_height"
            ]  # Using width for height
            defense_margin_y = (self.viz_height - defense_height) / 2

            # Left defense area
            self.scene.addRect(
                0,
                defense_margin_y,
                defense_width,
                defense_height,
                field_pen,
                QBrush(Qt.transparent),
            )

            # Right defense area
            self.scene.addRect(
                self.viz_width - defense_width,
                defense_margin_y,
                defense_width,
                defense_height,
                field_pen,
                QBrush(Qt.transparent),
            )

            # Store scaling factors for coordinate conversion
            self.scale_factor_x = self.viz_width / dims["field_width"]
            self.scale_factor_y = self.viz_height / dims["field_height"]

            # If path visualization is enabled, redraw all paths
            if self._show_paths:
                self._redraw_all_paths()

            # Fit view to scene
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

        except Exception as e:
            print(f"Error setting up field: {e}")

    def scale_coordinates(self, x, y, clamp=True):
        """Convert SSL coordinates (meters) to visualization coordinates (pixels)"""
        dims = self.divisions[self.current_division]

        if clamp:
            # Field boundaries check (only clamp if requested)
            x = max(-dims["field_width"] / 2, min(dims["field_width"] / 2, x))
            y = max(-dims["field_height"] / 2, min(dims["field_height"] / 2, y))

        # Scale and translate coordinates
        scaled_x = (x + dims["field_width"] / 2) * (
            self.viz_width / dims["field_width"]
        )
        scaled_y = (-y + dims["field_height"] / 2) * (
            self.viz_height / dims["field_height"]
        )
        return scaled_x, scaled_y

    def update_robot(self, x, y, orientation, team_color_qt, robot_id):
        """Update or add robot position using improved graphics item management"""
        try:
            robot_size = min(self.viz_width, self.viz_height) * 0.05
            scaled_x, scaled_y = self.scale_coordinates(
                x, y, clamp=False
            )  # Don't clamp robots

            items_dict = (
                self.blue_robot_items
                if team_color_qt == Qt.blue
                else self.yellow_robot_items
            )

            # Check if robot is within field boundaries
            dims = self.divisions[self.current_division]
            is_inside_field = (
                -dims["field_width"] / 2 <= x <= dims["field_width"] / 2
            ) and (-dims["field_height"] / 2 <= y <= dims["field_height"] / 2)

            # For Entry Level, use el_bounds for overall visibility, otherwise assume always visible if on screen
            is_renderable = True
            if self.current_division == "Entry Level":
                is_renderable = (
                    self.el_bounds["x_min"] <= x <= self.el_bounds["x_max"]
                ) and (self.el_bounds["y_min"] <= y <= self.el_bounds["y_max"])

            if (
                not is_renderable
            ):  # If completely outside visualization bounds, remove if exists
                if robot_id in items_dict:
                    for item_key in ["shape", "line", "text"]:
                        item = items_dict[robot_id].get(item_key)
                        if item and item.scene() == self.scene:
                            self.scene.removeItem(item)
                    del items_dict[robot_id]
                return

            if robot_id not in items_dict:
                # Create new items
                shape = self.scene.addEllipse(
                    0, 0, robot_size, robot_size, QPen(Qt.black)
                )
                line = self.scene.addLine(0, 0, 0, 0, QPen(Qt.black, 1))
                text = self.scene.addText(str(robot_id))
                text.setDefaultTextColor(Qt.black)
                items_dict[robot_id] = {"shape": shape, "line": line, "text": text}

            # Update existing items
            robot_gfx = items_dict[robot_id]
            robot_shape_item = robot_gfx["shape"]
            direction_line_item = robot_gfx["line"]
            id_text_item = robot_gfx["text"]

            robot_shape_item.setRect(
                scaled_x - robot_size / 2,
                scaled_y - robot_size / 2,
                robot_size,
                robot_size,
            )

            current_brush_color = QColor(team_color_qt)
            if not is_inside_field:
                current_brush_color.setAlpha(
                    128
                )  # Semi-transparent if outside field but in viz bounds
            robot_shape_item.setBrush(QBrush(current_brush_color))

            line_len = robot_size * 0.5 # Make line a bit longer
            end_x = line_len * math.cos(orientation)
            end_y = line_len * math.sin(
                orientation
            )  # Screen Y is inverted for math.sin
            direction_line_item.setLine(
                scaled_x, scaled_y, scaled_x + end_x, scaled_y + end_y
            )

            # Center text on robot
            text_rect = id_text_item.boundingRect()
            id_text_item.setPos(
                scaled_x - text_rect.width() / 2, scaled_y - text_rect.height() / 2
            )

            # Set visibility based on team toggle
            team_is_visible = (
                self.blue_visible if team_color_qt == Qt.blue else self.yellow_visible
            )
            robot_shape_item.setVisible(team_is_visible)
            direction_line_item.setVisible(team_is_visible)
            id_text_item.setVisible(team_is_visible)

        except Exception as e:
            print(f"Error in update_robot (ID {robot_id}): {e}")

    def update_ball(self, x, y):
        """Update ball position with improved graphics management"""
        try:
            ball_size_factor = (
                0.035 if self.current_division == "Entry Level" else 0.025
            )
            ball_size = min(self.viz_width, self.viz_height) * ball_size_factor
            scaled_x, scaled_y = self.scale_coordinates(
                x, y, clamp=False
            )  # Don't clamp ball position

            dims = self.divisions[self.current_division]
            is_inside = (-dims["field_width"] / 2 <= x <= dims["field_width"] / 2) and (
                -dims["field_height"] / 2 <= y <= dims["field_height"] / 2
            )

            ball_color = (
                QColor(255, 165, 0) if is_inside else QColor(255, 0, 0, 150)
            )  # Orange/Red, slightly transparent if out

            if not self.ball_item:
                self.ball_item = self.scene.addEllipse(
                    0, 0, ball_size, ball_size, QPen(Qt.black), QBrush(ball_color)
                )
            else:
                self.ball_item.setBrush(
                    QBrush(ball_color)
                )  # Update color in case it went in/out

            self.ball_item.setRect(
                scaled_x - ball_size / 2, scaled_y - ball_size / 2, ball_size, ball_size
            )
        except Exception as e:
            print(f"Error updating ball: {e}")

    def set_team_visibility(self, blue_visible=True, yellow_visible=True):
        """Set visibility of teams"""
        self.blue_visible = blue_visible
        self.yellow_visible = yellow_visible
        self.update_robot_visibility()

    def update_robot_visibility(self):
        """Update visibility of all robots based on current settings"""
        # Update blue robots
        for robot_id in self.blue_robot_items:
            robot_gfx = self.blue_robot_items[robot_id]
            for item in robot_gfx.values():
                if item and item.scene() == self.scene:
                    item.setVisible(self.blue_visible)

        # Update yellow robots
        for robot_id in self.yellow_robot_items:
            robot_gfx = self.yellow_robot_items[robot_id]
            for item in robot_gfx.values():
                if item and item.scene() == self.scene:
                    item.setVisible(self.yellow_visible)

    def set_division(self, division):
        """Update field dimensions when division changes"""
        try:
            if division in self.divisions:
                # Clear any existing items safely
                self.clear_safely()

                # Reinitialize the current division
                self.current_division = division

                # Recreate the field setup
                self.setup_field()
        except Exception as e:
            print(f"Error setting division: {e}")

    def clear_safely(self):
        """Completely clear all robots and ball from visualization"""
        try:
            # Clear path visualizations first
            self.clear_path_visualization()

            # Clear blue robots using new dictionary structure
            for robot_id in list(self.blue_robot_items.keys()):
                robot_gfx = self.blue_robot_items[robot_id]
                if robot_gfx.get("shape") and robot_gfx["shape"].scene() == self.scene:
                    self.scene.removeItem(robot_gfx["shape"])
                if robot_gfx.get("line") and robot_gfx["line"].scene() == self.scene:
                    self.scene.removeItem(robot_gfx["line"])
                if robot_gfx.get("text") and robot_gfx["text"].scene() == self.scene:
                    self.scene.removeItem(robot_gfx["text"])
            self.blue_robot_items.clear()

            # Clear yellow robots using new dictionary structure
            for robot_id in list(self.yellow_robot_items.keys()):
                robot_gfx = self.yellow_robot_items[robot_id]
                if robot_gfx.get("shape") and robot_gfx["shape"].scene() == self.scene:
                    self.scene.removeItem(robot_gfx["shape"])
                if robot_gfx.get("line") and robot_gfx["line"].scene() == self.scene:
                    self.scene.removeItem(robot_gfx["line"])
                if robot_gfx.get("text") and robot_gfx["text"].scene() == self.scene:
                    self.scene.removeItem(robot_gfx["text"])
            self.yellow_robot_items.clear()

            # Remove ball
            if self.ball_item and self.ball_item.scene() == self.scene:
                self.scene.removeItem(self.ball_item)
            self.ball_item = None

            # Note: we don't clear self.paths here, just the visualization items

        except Exception as e:
            print(f"Error during clear_safely: {e}")

    def show_paths(self, robot_id=None, path=None):
        """
        Show path for a specific robot or all paths if no robot_id specified

        Args:
            robot_id: Optional robot ID to show path for
            path: Optional path data to update before showing
        """
        try:
            # If path data is provided, update it first
            if robot_id is not None and path is not None:
                self.update_path(robot_id, path)

            # Set show_paths flag to true
            self._show_paths = True

            # Redraw the field with paths enabled
            self.setup_field()

        except Exception as e:
            print(f"Error showing paths: {e}")
