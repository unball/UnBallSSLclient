from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QFrame, QGraphicsScene, QGraphicsView, QVBoxLayout
from PyQt5.QtGui import QPainter
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

        # Track objects
        self.blue_robots = (
            {}
        )  # Key: robot_id, Value: (robot_shape, direction_line, id_text)
        self.yellow_robots = {}
        self.ball = None
        self.blue_visible = True
        self.yellow_visible = True

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

        # PathPlanner
        self.show_paths = False
        self.paths = {}  # robot_id -> path

    def set_show_paths(self, show: bool):
        """Enable/disable path visualization"""
        self.show_paths = show
        if hasattr(self, 'scene'):
            self.scene.update()

    def resizeEvent(self, event):
        """Handle resize events"""
        super().resizeEvent(event)
        self.setup_field()

    def setup_field(self):
        """Setup the field visualization"""
        print("Setting up field...")  # Debug print
        dims = self.divisions[self.current_division]

        try:
            # Clear scene before any setup
            self.scene.clear()

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

            # Draw paths if enabled
            if self.show_paths:
                for robot_id, path in self.paths.items():
                    if not path:
                        continue
                    # Draw path with team color
                    is_blue = robot_id in self.blue_robots
                    path_color = (
                        QColor(0, 0, 255, 128) if is_blue else QColor(255, 255, 0, 128)
                    )
                    path_pen = QPen(path_color, 2, Qt.DashLine)

                    # Draw path segments
                    for i in range(len(path) - 1):
                        start_x, start_y = self.scale_coordinates(
                            path[i][0], path[i][1]
                        )
                        end_x, end_y = self.scale_coordinates(
                            path[i + 1][0], path[i + 1][1]
                        )
                        self.scene.addLine(start_x, start_y, end_x, end_y, path_pen)

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

    def update_path(self, robot_id: int, path: List[Tuple[float, float]]):
        """Update stored path for a robot"""
        self.paths[robot_id] = path
        # Force field redraw to show new path
        self.setup_field()

    def update_robot(self, x, y, orientation, team_color, robot_id):
        """Update or add robot position"""
        robot_size = min(self.viz_width, self.viz_height) * 0.05
        scaled_x, scaled_y = self.scale_coordinates(
            x, y, clamp=False
        )  # Don't clamp robots

        # Check if robot is within field boundaries
        dims = self.divisions[self.current_division]

        # Different bounds check based on division
        if self.current_division == "Entry Level":
            is_inside = (-dims["field_width"] / 2 <= x <= dims["field_width"] / 2) and (
                -dims["field_height"] / 2 <= y <= dims["field_height"] / 2
            )
            # Check if robot is within our extended visualization bounds
            is_visible = (self.el_bounds["x_min"] <= x <= self.el_bounds["x_max"]) and (
                self.el_bounds["y_min"] <= y <= self.el_bounds["y_max"]
            )
        else:
            is_inside = (-dims["field_width"] / 2 <= x <= dims["field_width"] / 2) and (
                -dims["field_height"] / 2 <= y <= dims["field_height"] / 2
            )
            is_visible = True

        # Remove old robot graphics if they exist
        robot_items = self.blue_robots if team_color == Qt.blue else self.yellow_robots
        if robot_id in robot_items:
            old_robot, old_line, old_text = robot_items[robot_id]
            self.scene.removeItem(old_robot)
            self.scene.removeItem(old_line)
            self.scene.removeItem(old_text)

        # Only create robot if it's within visible bounds
        if is_visible:
            # Create robot with opacity based on position
            robot_rect = QRectF(
                scaled_x - robot_size / 2,
                scaled_y - robot_size / 2,
                robot_size,
                robot_size,
            )

            # Create robot with different appearance when outside field
            robot = self.scene.addEllipse(robot_rect, QPen(Qt.black))
            if is_inside:
                robot.setBrush(QBrush(team_color))
            else:
                # Create a semi-transparent version of the team color for out-of-field robots
                out_color = QColor(team_color)
                out_color.setAlpha(128)  # 50% transparency
                robot.setBrush(QBrush(out_color))

            # Add direction indicator
            line_length = robot_size / 2
            end_x = scaled_x + line_length * math.cos(orientation)
            end_y = scaled_y + line_length * math.sin(orientation)
            line = self.scene.addLine(
                QLineF(scaled_x, scaled_y, end_x, end_y), QPen(Qt.black, 2)
            )

            # Add ID text
            text = self.scene.addText(str(robot_id))
            text.setDefaultTextColor(Qt.black)
            text.setPos(
                scaled_x - text.boundingRect().width() / 2,
                scaled_y - text.boundingRect().height() / 2,
            )

            # Store new robot items
            robot_items[robot_id] = (robot, line, text)

            # Apply current visibility settings
            visible = (
                self.blue_visible if team_color == Qt.blue else self.yellow_visible
            )
            robot.setVisible(visible)
            line.setVisible(visible)
            text.setVisible(visible)

    def update_ball(self, x, y):
        """Update ball position with out-of-field visualization"""
        # Scale ball size based on field size, make it slightly larger for SSL EL
        if self.current_division == "Entry Level":
            ball_size = (
                min(self.viz_width, self.viz_height) * 0.035
            )  # Larger for SSL EL
        else:
            ball_size = min(self.viz_width, self.viz_height) * 0.025

        # Get scaled coordinates without clamping
        scaled_x, scaled_y = self.scale_coordinates(x, y, clamp=False)

        # Check if ball is within field boundaries
        dims = self.divisions[self.current_division]
        is_inside = (-dims["field_width"] / 2 <= x <= dims["field_width"] / 2) and (
            -dims["field_height"] / 2 <= y <= dims["field_height"] / 2
        )

        if self.ball:
            self.scene.removeItem(self.ball)

        # Determine color based on position
        ball_color = (
            QColor(255, 165, 0) if is_inside else QColor(255, 0, 0)
        )  # Orange/Red

        # Create ball visualization
        ball_rect = QRectF(
            scaled_x - ball_size / 2, scaled_y - ball_size / 2, ball_size, ball_size
        )
        self.ball = self.scene.addEllipse(ball_rect, QPen(Qt.black), QBrush(ball_color))

    def set_team_visibility(self, blue_visible=True, yellow_visible=True):
        """Set visibility of teams"""
        self.blue_visible = blue_visible
        self.yellow_visible = yellow_visible
        self.update_robot_visibility()

    def update_robot_visibility(self):
        """Update visibility of all robots based on current settings"""
        for items in self.blue_robots.values():
            for item in items:
                item.setVisible(self.blue_visible)

        for items in self.yellow_robots.values():
            for item in items:
                item.setVisible(self.yellow_visible)

    def set_division(self, division):
        """Update field dimensions when division changes"""
        if division in self.divisions:
            # Clear everything first
            self.scene.clear()  # Removes ALL items from the scene

            # Reset tracking dictionaries
            self.blue_robots.clear()
            self.yellow_robots.clear()
            self.ball = None

            # Reinitialize the current division
            self.current_division = division

            # Recreate the field setup
            self.setup_field()

    def clear_safely(self):
        """Completely clear all robots and ball from visualization"""
        try:
            # Remove all blue robots
            for robot_id in list(self.blue_robots.keys()):
                if robot_id in self.blue_robots:
                    robot, line, text = self.blue_robots[robot_id]
                    self.scene.removeItem(robot)
                    self.scene.removeItem(line)
                    self.scene.removeItem(text)

            # Remove all yellow robots
            for robot_id in list(self.yellow_robots.keys()):
                if robot_id in self.yellow_robots:
                    robot, line, text = self.yellow_robots[robot_id]
                    self.scene.removeItem(robot)
                    self.scene.removeItem(line)
                    self.scene.removeItem(text)

            # Remove ball
            if self.ball:
                self.scene.removeItem(self.ball)

            # Clear dictionaries
            self.blue_robots.clear()
            self.yellow_robots.clear()
            self.ball = None

        except Exception as e:
            print(f"Error during clear_safely: {e}")

    def clear(self):
        """Clear all robots and ball from visualization"""
        for items in self.blue_robots.values():
            for item in items:
                self.scene.removeItem(item)
        for items in self.yellow_robots.values():
            for item in items:
                self.scene.removeItem(item)
        if self.ball:
            self.scene.removeItem(self.ball)

        self.blue_robots.clear()
        self.yellow_robots.clear()
        self.ball = None
