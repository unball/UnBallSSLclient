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
        self.yellow_visible = True  # Default visibility settings

        # Path visualization
        self._show_paths = False
        self.paths = {}  # robot_id -> path data
        self.path_items = {}  # robot_id -> list of path graphics items

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
                    for robot_id in self.paths:
                        self._draw_path_for_robot(robot_id)

        except Exception as e:
            print(f"Error setting show paths: {e}")

    def clear_path_visualization(self, robot_id=None):
        """Clear path visualization items safely"""
        try:
            if robot_id is not None:
                # Clear for specific robot
                if robot_id in self.path_items:
                    for item in list(self.path_items[robot_id]):
                        if (
                            item.scene() == self.scene
                        ):  # Check if item belongs to our scene
                            self.scene.removeItem(item)
                    self.path_items[robot_id] = []
            else:
                # Clear for all robots
                for rid in list(self.path_items.keys()):
                    if (
                        rid in self.path_items
                    ):  # Check again in case of concurrent modification
                        for item in list(self.path_items[rid]):
                            if (
                                item.scene() == self.scene
                            ):  # Check if item belongs to our scene
                                self.scene.removeItem(item)
                        self.path_items[rid] = []
        except Exception as e:
            print(f"Error clearing path visualization: {e}")
            # Reset path items tracking to recover from error
            self.path_items = {}

    def _draw_path_for_robot(self, robot_id):
        """Draw path for a specific robot"""
        try:
            if robot_id not in self.paths or not self.paths[robot_id]:
                return

            path = self.paths[robot_id]
            if len(path) < 2:
                return

            # Initialize path items list for this robot if not already present
            if robot_id not in self.path_items:
                self.path_items[robot_id] = []

            # Determine path color based on robot team
            is_blue = robot_id in self.blue_robots
            path_color = QColor(0, 0, 255, 128) if is_blue else QColor(255, 255, 0, 128)
            path_pen = QPen(path_color, 2, Qt.DashLine)

            # Draw path segments
            for i in range(len(path) - 1):
                try:
                    start_x, start_y = self.scale_coordinates(path[i][0], path[i][1])
                    end_x, end_y = self.scale_coordinates(
                        path[i + 1][0], path[i + 1][1]
                    )

                    # Create line item and track it
                    line = self.scene.addLine(start_x, start_y, end_x, end_y, path_pen)
                    self.path_items[robot_id].append(line)
                except Exception as e:
                    print(f"Error drawing path segment {i}: {e}")
        except Exception as e:
            print(f"Error drawing path for robot {robot_id}: {e}")

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
            self.path_items = {}

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
                for robot_id in self.paths:
                    self._draw_path_for_robot(robot_id)

            # Fit view to scene
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

        except Exception as e:
            print(f"Error setting up field: {e}")

    def update_path(self, robot_id: int, path: List[Tuple[float, float]]):
        """Update stored path for a robot"""
        try:
            # First clear existing path visualization for this robot
            self.clear_path_visualization(robot_id)

            if path is None or len(path) < 2:
                # No valid path to display
                if robot_id in self.paths:
                    del self.paths[robot_id]
                return

            # Store the path (make a copy to avoid external modifications)
            self.paths[robot_id] = list(path)

            # If path visualization is enabled, draw the new path immediately
            if self._show_paths:
                self._draw_path_for_robot(robot_id)

        except Exception as e:
            print(f"Error updating path for robot {robot_id}: {e}")

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

    def update_robot(self, x, y, orientation, team_color, robot_id):
        """Update or add robot position"""
        try:
            robot_size = min(self.viz_width, self.viz_height) * 0.05
            scaled_x, scaled_y = self.scale_coordinates(
                x, y, clamp=False
            )  # Don't clamp robots

            # Check if robot is within field boundaries
            dims = self.divisions[self.current_division]

            # Different bounds check based on division
            if self.current_division == "Entry Level":
                is_inside = (
                    -dims["field_width"] / 2 <= x <= dims["field_width"] / 2
                ) and (-dims["field_height"] / 2 <= y <= dims["field_height"] / 2)
                # Check if robot is within our extended visualization bounds
                is_visible = (
                    self.el_bounds["x_min"] <= x <= self.el_bounds["x_max"]
                ) and (self.el_bounds["y_min"] <= y <= self.el_bounds["y_max"])
            else:
                is_inside = (
                    -dims["field_width"] / 2 <= x <= dims["field_width"] / 2
                ) and (-dims["field_height"] / 2 <= y <= dims["field_height"] / 2)
                is_visible = True

            # Remove old robot graphics if they exist
            robot_items = (
                self.blue_robots if team_color == Qt.blue else self.yellow_robots
            )
            if robot_id in robot_items:
                try:
                    old_robot, old_line, old_text = robot_items[robot_id]

                    # Check if items still exist and belong to this scene
                    if old_robot and old_robot.scene() == self.scene:
                        self.scene.removeItem(old_robot)
                    if old_line and old_line.scene() == self.scene:
                        self.scene.removeItem(old_line)
                    if old_text and old_text.scene() == self.scene:
                        self.scene.removeItem(old_text)

                    # Delete reference to old items
                    del robot_items[robot_id]
                except Exception as e:
                    print(f"Error removing old robot {robot_id}: {e}")
                    # Delete reference to potentially invalid items
                    if robot_id in robot_items:
                        del robot_items[robot_id]

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
        except Exception as e:
            print(f"Error in update_robot: {e}")

    def update_ball(self, x, y):
        """Update ball position with out-of-field visualization"""
        try:
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

            # Remove old ball if it exists
            if self.ball:
                if self.ball.scene() == self.scene:
                    self.scene.removeItem(self.ball)
                self.ball = None

            # Determine color based on position
            ball_color = (
                QColor(255, 165, 0) if is_inside else QColor(255, 0, 0)
            )  # Orange/Red

            # Create ball visualization
            ball_rect = QRectF(
                scaled_x - ball_size / 2, scaled_y - ball_size / 2, ball_size, ball_size
            )
            self.ball = self.scene.addEllipse(
                ball_rect, QPen(Qt.black), QBrush(ball_color)
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
        for items in self.blue_robots.values():
            for item in items:
                if item and item.scene() == self.scene:
                    item.setVisible(self.blue_visible)

        for items in self.yellow_robots.values():
            for item in items:
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

            # Remove all blue robots
            for robot_id in list(self.blue_robots.keys()):
                try:
                    if robot_id in self.blue_robots:
                        robot, line, text = self.blue_robots[robot_id]
                        if robot and robot.scene() == self.scene:
                            self.scene.removeItem(robot)
                        if line and line.scene() == self.scene:
                            self.scene.removeItem(line)
                        if text and text.scene() == self.scene:
                            self.scene.removeItem(text)
                except Exception as e:
                    print(f"Error removing blue robot {robot_id}: {e}")

            # Remove all yellow robots
            for robot_id in list(self.yellow_robots.keys()):
                try:
                    if robot_id in self.yellow_robots:
                        robot, line, text = self.yellow_robots[robot_id]
                        if robot and robot.scene() == self.scene:
                            self.scene.removeItem(robot)
                        if line and line.scene() == self.scene:
                            self.scene.removeItem(line)
                        if text and text.scene() == self.scene:
                            self.scene.removeItem(text)
                except Exception as e:
                    print(f"Error removing yellow robot {robot_id}: {e}")

            # Remove ball
            try:
                if self.ball and self.ball.scene() == self.scene:
                    self.scene.removeItem(self.ball)
            except Exception as e:
                print(f"Error removing ball: {e}")

            # Clear dictionaries
            self.blue_robots.clear()
            self.yellow_robots.clear()
            self.ball = None
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
