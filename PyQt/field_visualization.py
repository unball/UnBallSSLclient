from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import Qt, QRectF, QLineF
from PyQt5.QtGui import QPen, QBrush, QColor
import math


class FieldVisualization(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QtWidgets.QGraphicsScene()
        self.view = QtWidgets.QGraphicsView(self.scene)

        # Add visibility tracking
        self.blue_visible = True
        self.yellow_visible = True

        # SSL field dimensions in meters (standard field)
        self.field_width = 4.5  # Length (x-axis)
        self.field_height = 3.0  # Width (y-axis)

        # Visualization dimensions (pixels)
        self.viz_width = 700  # Base width in pixels
        self.viz_height = int(
            self.viz_width * (self.field_height / self.field_width)
        )  # Maintain aspect ratio

        # Configure view
        self.view.setFixedSize(self.viz_width, self.viz_height)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)

        # Track objects
        self.blue_robots = (
            {}
        )  # Key: robot_id, Value: (robot_shape, direction_line, id_text)
        self.yellow_robots = {}
        self.ball = None

        # Setup layout
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.view)

        self.setup_field()

    def scale_coordinates(self, x, y):
        """Convert SSL coordinates (in meters) to visualization coordinates"""
        scaled_x = (x + self.field_width / 2) * (self.viz_width / self.field_width)
        scaled_y = (-y + self.field_height / 2) * (self.viz_height / self.field_height)
        return scaled_x, scaled_y

    def setup_field(self):
        field_color = QColor(0, 150, 0)
        field_pen = QPen(Qt.white, 2)

        # Main field
        self.scene.addRect(
            QRectF(0, 0, self.viz_width, self.viz_height),
            field_pen,
            QBrush(field_color),
        )

        # Center line
        self.scene.addLine(
            QLineF(self.viz_width / 2, 0, self.viz_width / 2, self.viz_height),
            field_pen,
        )

        # Center circle
        center_x = self.viz_width / 2 - 50
        center_y = self.viz_height / 2 - 50
        self.scene.addEllipse(
            QRectF(center_x, center_y, 100, 100), field_pen, QBrush(Qt.transparent)
        )

        # Penalty areas
        penalty_width = self.viz_width * 0.15
        penalty_height = self.viz_height * 0.4
        margin_y = (self.viz_height - penalty_height) / 2

        # Left penalty area
        self.scene.addRect(
            QRectF(0, margin_y, penalty_width, penalty_height),
            field_pen,
            QBrush(Qt.transparent),
        )

        # Right penalty area
        self.scene.addRect(
            QRectF(
                self.viz_width - penalty_width, margin_y, penalty_width, penalty_height
            ),
            field_pen,
            QBrush(Qt.transparent),
        )

    def update_robot(self, x, y, orientation, team_color, robot_id):
        """Update or add robot position"""
        robot_size = 30
        scaled_x, scaled_y = self.scale_coordinates(x, y)

        # Determine team and visibility
        is_blue = team_color == Qt.blue
        visible = self.blue_visible if is_blue else self.yellow_visible
        robot_items = self.blue_robots if is_blue else self.yellow_robots

        # Remove old robot graphics if they exist
        if robot_id in robot_items:
            old_robot, old_line, old_text = robot_items[robot_id]
            self.scene.removeItem(old_robot)
            self.scene.removeItem(old_line)
            self.scene.removeItem(old_text)

        # Create new robot graphics
        robot_rect = QRectF(
            scaled_x - robot_size / 2, scaled_y - robot_size / 2, robot_size, robot_size
        )
        robot = self.scene.addEllipse(robot_rect, QPen(Qt.black), QBrush(team_color))

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

        # Store new items
        robot_items[robot_id] = (robot, line, text)

        # Set visibility based on team
        robot.setVisible(visible)
        line.setVisible(visible)
        text.setVisible(visible)

    def update_ball(self, x, y):
        """Update ball position"""
        ball_size = 15
        scaled_x, scaled_y = self.scale_coordinates(x, y)

        if self.ball:
            self.scene.removeItem(self.ball)

        ball_rect = QRectF(
            scaled_x - ball_size / 2, scaled_y - ball_size / 2, ball_size, ball_size
        )
        self.ball = self.scene.addEllipse(
            ball_rect, QPen(Qt.black), QBrush(QColor(255, 165, 0))
        )

    def set_team_visibility(self, blue_visible=True, yellow_visible=True):
        """Set visibility of teams"""
        self.blue_visible = blue_visible
        self.yellow_visible = yellow_visible

        print(
            f"Setting visibility - Blue: {blue_visible}, Yellow: {yellow_visible}"
        )  # Debug print

        # Update visibility of all blue robots
        for robot_id in self.blue_robots:
            robot, line, text = self.blue_robots[robot_id]
            robot.setVisible(blue_visible)
            line.setVisible(blue_visible)
            text.setVisible(blue_visible)

        # Update visibility of all yellow robots
        for robot_id in self.yellow_robots:
            robot, line, text = self.yellow_robots[robot_id]
            robot.setVisible(yellow_visible)
            line.setVisible(yellow_visible)
            text.setVisible(yellow_visible)

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
