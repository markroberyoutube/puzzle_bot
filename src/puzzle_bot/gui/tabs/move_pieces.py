import json
import logging
import math
import posixpath

import numpy as np
from PyQt5 import QtTest
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication

from puzzle_bot import solver_backend
from puzzle_bot.paths import latest_batch_number
from puzzle_bot.utils import anchor_order, parse_int

logger = logging.getLogger(__name__)


class MovePiecesTab:
    def setup_perform_moves_tab(self) -> None:
        # Load the measured table-height map
        table_height_map_path = self.settings.project_file("table_height_map")
        self.table_height_map: list[tuple[int, int, int]] = []
        for line in table_height_map_path.read_text().splitlines():
            x, y, encoder_value = line.strip(" ").split(",")
            self.table_height_map.append((int(x), int(y), int(encoder_value)))

        self.list_moves_button.clicked.connect(self.list_moves)
        self.perform_moves_button.clicked.connect(self.move_pieces_into_place)

    @pyqtSlot()
    def list_moves(self) -> None:
        self.list_moves_button.setEnabled(False)
        self.perform_moves_button.setEnabled(False)
        self.set_clearcore_availability(False)
        self.set_gripper_availability(False)

        # Scale factor used to shrink the images, to scale the solution back
        scale_bmp_to_width = solver_backend.solver_config_value("SCALE_BMP_TO_WIDTH")
        scale_factor = scale_bmp_to_width / self.robot.input_image_width if scale_bmp_to_width else 1.0

        # Find the most recent numbered batch directory
        solver_dir = self.solver_directory_textbox.text()
        solver_batch_working_dir = posixpath.join(solver_dir, str(latest_batch_number(solver_dir)))

        # Compile all solution data into one array of dicts
        solution = []
        for piece_info_file in solver_backend.solution_dir(solver_batch_working_dir).glob("*.json"):
            solution.append(json.loads(piece_info_file.read_text()))
            QApplication.processEvents()

        # A "move" is: pickup_x, pickup_y => drop_x, drop_y, angle
        # (all values in absolute motor counts)
        max_x = max((piece["solution_x"] for piece in solution), default=0)
        max_y = max((piece["solution_y"] for piece in solution), default=0)

        # Sort the pieces in assembly order
        all_pieces = np.ndindex((max_x + 1, max_y + 1))
        pieces_in_anchor_order = anchor_order(all_pieces)

        for x, y in pieces_in_anchor_order:
            piece = next(p for p in solution if p["solution_x"] == x and p["solution_y"] == y)

            # Pickup point relative to the photo origin, in pixels
            photo_space_incenter_x, photo_space_incenter_y = piece["photo_space_incenter"]

            # Convert pixel pickup point to motor counts
            motor_counts_incenter_x = photo_space_incenter_x * float(self.motor_counts_x_per_px_x_textbox.text())
            motor_counts_incenter_y = photo_space_incenter_y * float(self.motor_counts_y_per_px_y_textbox.text())

            # Absolute pickup point in robot coordinates: photo offset + gripper offset
            pickup_motor_counts_x = int(
                piece["robot_state"]["photo_at_motor_position"][0]
                + -motor_counts_incenter_x
                + float(self.camera_to_gripper_x_transform_textbox.text())
            )
            pickup_motor_counts_y = int(
                piece["robot_state"]["photo_at_motor_position"][1]
                + -motor_counts_incenter_y
                + float(self.camera_to_gripper_y_transform_textbox.text())
            )

            destination_angle_radians = piece["dest_rotation"]
            destination_angle_degrees = destination_angle_radians * (180 / math.pi)
            destination_angle_motor_counts = destination_angle_degrees * self.robot.rotation_motor_counts_per_degree

            # The motor only accepts positive counts; wrap negative angles
            # (possibly twice when the solver requests >360° rotations)
            while destination_angle_motor_counts < 0:
                destination_angle_motor_counts = self.robot.max_rotation_motor_counts + destination_angle_motor_counts

            # Destination point relative to the top-left origin, in pixels
            dest_photo_space_incenter_x, dest_photo_space_incenter_y = piece["dest_photo_space_incenter"]

            logger.debug("dest_photo_space_incenter_x: %s", dest_photo_space_incenter_x)
            logger.debug("dest_photo_space_incenter_y: %s", dest_photo_space_incenter_y)

            dest_photo_space_incenter_x = dest_photo_space_incenter_x / scale_factor
            dest_photo_space_incenter_y = dest_photo_space_incenter_y / scale_factor

            dest_motor_counts_x = dest_photo_space_incenter_x * float(self.motor_counts_x_per_px_x_textbox.text())
            dest_motor_counts_y = dest_photo_space_incenter_y * float(self.motor_counts_y_per_px_y_textbox.text())

            # Reverse orientation per the motor_counts_per_px sign convention
            dest_motor_counts_x = -dest_motor_counts_x
            dest_motor_counts_y = -dest_motor_counts_y

            logger.debug("dest_motor_counts_x: %s", dest_motor_counts_x)
            logger.debug("dest_motor_counts_y: %s", dest_motor_counts_y)

            dest_motor_counts_x += self.robot.solution_motor_origin[0]
            dest_motor_counts_y += self.robot.solution_motor_origin[1]

            destination_angle_motor_counts = round(destination_angle_motor_counts)
            dest_motor_counts_x = round(dest_motor_counts_x)
            dest_motor_counts_y = round(dest_motor_counts_y)

            logger.debug("pickup x: %s", pickup_motor_counts_x)
            logger.debug("pickup y: %s", pickup_motor_counts_y)
            logger.debug("dest x: %s", dest_motor_counts_x)
            logger.debug("dest y: %s", dest_motor_counts_y)
            logger.debug("angle: %s", destination_angle_motor_counts)

            move = f"({x},{y}): {pickup_motor_counts_x},{pickup_motor_counts_y} => {dest_motor_counts_x},{dest_motor_counts_y},{destination_angle_motor_counts}"
            self.list_moves_textarea.insertHtml(f"{move}<br />")
            QApplication.processEvents()

        self.list_moves_button.setEnabled(True)
        self.perform_moves_button.setEnabled(True)
        self.set_clearcore_availability(True)
        self.set_gripper_availability(True)

    def get_table_height(self, x: int, y: int, z: int) -> float:
        target = (int(x), int(y))
        # First minimal-distance entry wins, matching the original strict "<" scan
        _, _, nearest_encoder_value = min(self.table_height_map, key=lambda entry: math.dist(entry[:2], target))

        # Compensate for the Z difference between map measurements and this move
        return nearest_encoder_value + ((int(z) - self.robot.table_height_map_measurement_z) * self.robot.encoder_counts_per_z_motor_count)

    def get_fully_placed_encoder_threshold(self, x: int, y: int, z: int) -> float:
        table_height_encoder_value = self.get_table_height(x, y, z)
        final_value = table_height_encoder_value - self.robot.piece_set_threshold - self.robot.minimum_piece_thickness_encoder_counts
        return min(final_value, self.robot.encoder_fully_extended)

    def get_nearly_placed_encoder_threshold(self, x: int, y: int, z: int) -> float:
        table_height_encoder_value = self.get_table_height(x, y, z)
        final_value = table_height_encoder_value - self.robot.piece_nearly_set_threshold - self.robot.minimum_piece_thickness_encoder_counts
        return min(final_value, self.robot.encoder_fully_extended)

    @pyqtSlot()
    def move_pieces_into_place(self) -> None:
        robot = self.robot
        moves = self.perform_moves_textarea.toPlainText()
        for move in moves.splitlines():
            _, data = move.split(":")
            src, dest = data.strip(" ").split("=>")
            src_x, src_y = src.strip().split(",")
            dst_x, dst_y, dst_angle = dest.strip().split(",")

            src_x = parse_int(src_x)
            src_y = parse_int(src_y)
            dst_x = parse_int(dst_x)
            dst_y = parse_int(dst_y)
            dst_angle = parse_int(dst_angle)

            logger.debug("\n\nPICKING UP NEXT PIECE")
            logger.debug("TURNING OFF VACUUM")
            self.send_clearcore_command("0", blocking=True, update_position=True)
            QApplication.processEvents()

            x = parse_int(self.move_x_textbox.text())
            y = parse_int(self.move_y_textbox.text())

            logger.debug("MOVING TO SAFE TRAVEL Z")
            self.send_clearcore_command(f"m {x},{y},{robot.safe_travel_z}", blocking=True)
            QApplication.processEvents()

            eye_angle = robot.left_eye_angle if src_x < x else robot.right_eye_angle
            logger.debug("POINTING EYES AT %s", eye_angle)
            self.send_gripper_command(f"e {eye_angle}", blocking=True)

            # Rotate back to 0 twice: the second call is non-blocking so it can
            # rotate while we travel to the pickup position.
            logger.debug("ROTATING TO 0")
            self.send_gripper_command("r 0", blocking=True)
            QApplication.processEvents()
            self.send_gripper_command("r 0", blocking=False)
            QApplication.processEvents()

            logger.debug("MOVING TO PICKUP POSITION")
            self.send_clearcore_command(f"m {src_x},{src_y},{robot.safe_travel_z}", blocking=True)
            QApplication.processEvents()

            self.send_gripper_command(f"e {robot.neutral_eye_angle}", blocking=True)

            logger.debug("MOVING DOWN TO PICKUP_Z")
            self.send_clearcore_command(f"m {src_x},{src_y},{robot.pickup_z}", blocking=True)
            QApplication.processEvents()

            logger.debug("TURNING VACUUM ON")
            self.send_clearcore_command("1", blocking=True, update_position=False)
            QApplication.processEvents()

            QtTest.QTest.qWait(1000)

            logger.debug("MOVING UP TO TRAVEL_Z")
            self.send_clearcore_command(f"m {src_x},{src_y},{robot.safe_travel_z}", blocking=True)
            QApplication.processEvents()

            eye_angle = robot.left_eye_angle if dst_x < src_x else robot.right_eye_angle
            self.send_gripper_command(f"e {eye_angle}", blocking=True)

            # Rotate to the destination angle; again, the second call can
            # complete while we travel.
            logger.debug("ROTATING TO FINAL ANGLE")
            self.send_gripper_command(f"r {dst_angle}", blocking=True)
            QApplication.processEvents()
            self.send_gripper_command(f"r {dst_angle}", blocking=False)
            QApplication.processEvents()

            logger.debug("MOVING TO DESTINATION POSITION")
            self.send_clearcore_command(f"m {dst_x},{dst_y},{robot.safe_travel_z}", blocking=True)
            QApplication.processEvents()

            self.send_gripper_command(f"e {robot.neutral_eye_angle}", blocking=True)

            logger.debug("MOVING DOWN TO DROP OFF POSITION")
            self.send_clearcore_command(f"m {dst_x},{dst_y},{robot.dropoff_z}", blocking=True)
            QApplication.processEvents()

            wiggle_x = dst_x
            wiggle_y = dst_y

            # Grid search: try positions until the encoder reads "placed"
            if self.settings.feature_flag("grid"):
                best_x_position = wiggle_x
                best_y_position = wiggle_y

                self.linear_encoder_value = None
                while self.linear_encoder_value is None:
                    self.send_gripper_command("d", update_position=False, blocking=True)

                localized_nearly_placed_encoder_threshold = self.get_nearly_placed_encoder_threshold(dst_x, dst_y, robot.dropoff_z)
                if not self.linear_encoder_value < localized_nearly_placed_encoder_threshold:
                    logger.debug("PIECE NEARLY PLACED! (%s > %s)", self.linear_encoder_value, localized_nearly_placed_encoder_threshold)
                else:
                    logger.debug(
                        "PIECE NOT NEARLY PLACED (%s not > %s)", self.linear_encoder_value, localized_nearly_placed_encoder_threshold
                    )
                    logger.debug("TRYING 'PICK UP AND GRID WIGGLE' ROUTINE...")
                    piece_placed = False
                    delta_x = 66
                    delta_y = 66
                    steps_neg_x = 10  # 10 steps of 66 counts (10 thou) → -0.100
                    steps_pos_x = 10
                    steps_neg_y = 10
                    steps_pos_y = 10

                    best_encoder_value = self.linear_encoder_value

                    while not piece_placed:
                        delta_positions = []
                        for x in range(-delta_x * steps_neg_x, delta_x * steps_pos_x, delta_x):
                            for y in range(+delta_y * steps_neg_y, -delta_y * steps_pos_y, -delta_y):
                                delta_positions.append([x, y])
                        print(f"GENERATED {len(delta_positions)} grid positions to try")

                        for delta_pos in delta_positions:
                            if not piece_placed:
                                self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                                QApplication.processEvents()

                                wiggle_x = dst_x + delta_pos[0]
                                wiggle_y = dst_y + delta_pos[1]

                                logger.debug("PICK AND GRID WIGGLE (%s,%s)", delta_pos[0], delta_pos[1])
                                QApplication.processEvents()

                                self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                                QApplication.processEvents()

                                self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.dropoff_z}", blocking=True)
                                QApplication.processEvents()

                                self.linear_encoder_value = None
                                while self.linear_encoder_value is None:
                                    self.send_gripper_command("d", update_position=False, blocking=True)

                                if self.linear_encoder_value > best_encoder_value:
                                    best_x_position = wiggle_x
                                    best_y_position = wiggle_y
                                    best_encoder_value = self.linear_encoder_value

                                if self.linear_encoder_value < localized_nearly_placed_encoder_threshold:
                                    logger.debug(
                                        "PIECE NOT NEARLY PLACED (%s not > %s)",
                                        self.linear_encoder_value,
                                        localized_nearly_placed_encoder_threshold,
                                    )
                                else:
                                    piece_placed = True
                                    logger.debug("PICK UP AND GRID WIGGLE WORKED!")

                    if not piece_placed:
                        # Go to the best location found before scoot-and-wiggle
                        self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                        QApplication.processEvents()
                        wiggle_x = best_x_position
                        wiggle_y = best_y_position
                        self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                        QApplication.processEvents()

            # Spiral wiggle
            if self.settings.feature_flag("spiral"):
                best_x_position = wiggle_x
                best_y_position = wiggle_y
                best_angle = dst_angle

                self.linear_encoder_value = None
                while self.linear_encoder_value is None:
                    self.send_gripper_command("d", update_position=False, blocking=True)

                localized_nearly_placed_encoder_threshold = self.get_nearly_placed_encoder_threshold(dst_x, dst_y, robot.dropoff_z)
                if not self.linear_encoder_value < localized_nearly_placed_encoder_threshold:
                    logger.debug("PIECE NEARLY PLACED! (%s > %s)", self.linear_encoder_value, localized_nearly_placed_encoder_threshold)
                else:
                    logger.debug(
                        "PIECE NOT NEARLY PLACED (%s not > %s)", self.linear_encoder_value, localized_nearly_placed_encoder_threshold
                    )
                    logger.debug("TRYING 'PICK UP AND SPIRAL WIGGLE' ROUTINE...")

                    piece_placed = False

                    for delta_angle in [0]:
                        if not piece_placed:
                            self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                            QApplication.processEvents()

                            self.send_gripper_command(f"r {dst_angle + delta_angle}", blocking=True)
                            QApplication.processEvents()
                            self.send_gripper_command(f"r {dst_angle + delta_angle}", blocking=True)
                            QApplication.processEvents()

                            delta_x = 50
                            delta_y = 50
                            try_number = 0
                            maximum_tries = 15
                            best_encoder_value = self.linear_encoder_value

                            while not piece_placed and try_number < maximum_tries:
                                # 9-point search, widened after each round
                                delta_positions = [
                                    (0, -delta_y),
                                    (-delta_x, -delta_y),
                                    (-delta_x, 0),
                                    (-delta_x, +delta_y),
                                    (0, +delta_y),
                                    (+delta_x, +delta_y),
                                    (+delta_x, 0),
                                    (+delta_x, -delta_y),
                                ]
                                for delta_pos in delta_positions:
                                    if not piece_placed:
                                        self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                                        QApplication.processEvents()

                                        wiggle_x = dst_x + delta_pos[0]
                                        wiggle_y = dst_y + delta_pos[1]

                                        logger.debug("PICK AND SPIRAL WIGGLE (%s,%s)", delta_pos[0], delta_pos[1])
                                        QApplication.processEvents()

                                        self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                                        QApplication.processEvents()

                                        self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.dropoff_z}", blocking=True)
                                        QApplication.processEvents()

                                        self.linear_encoder_value = None
                                        while self.linear_encoder_value is None:
                                            self.send_gripper_command("d", update_position=False, blocking=True)

                                        if self.linear_encoder_value > best_encoder_value:
                                            best_x_position = wiggle_x
                                            best_y_position = wiggle_y
                                            best_angle = dst_angle + delta_angle
                                            best_encoder_value = self.linear_encoder_value

                                        if self.linear_encoder_value < localized_nearly_placed_encoder_threshold:
                                            logger.debug(
                                                "PIECE NOT NEARLY PLACED (%s not > %s)",
                                                self.linear_encoder_value,
                                                localized_nearly_placed_encoder_threshold,
                                            )
                                        else:
                                            piece_placed = True
                                            logger.debug("PICK UP AND SPIRAL WIGGLE WORKED!")

                                try_number += 1
                                delta_x += 25
                                delta_y += 25

                    if not piece_placed:
                        # Go to the best location/angle found
                        self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                        QApplication.processEvents()
                        self.send_gripper_command(f"r {best_angle}", blocking=True)
                        QApplication.processEvents()
                        self.send_gripper_command(f"r {best_angle}", blocking=True)
                        QApplication.processEvents()
                        wiggle_x = best_x_position
                        wiggle_y = best_y_position
                        self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.wiggle_z}", blocking=True)
                        QApplication.processEvents()

                # Scoot-and-wiggle: drag the piece around at scoot height until it pops in
                if self.settings.feature_flag("scoot"):
                    localized_fully_placed_encoder_threshold = self.get_fully_placed_encoder_threshold(dst_x, dst_y, robot.dropoff_z)
                    if self.linear_encoder_value < localized_fully_placed_encoder_threshold:
                        logger.debug(
                            "PIECE NOT FULLY PLACED (%s not > %s)", self.linear_encoder_value, localized_fully_placed_encoder_threshold
                        )
                        logger.debug("TRYING 'SCOOT AND WIGGLE' ROUTINE...")
                        piece_placed = False
                        self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.scoot_z}", blocking=True)
                        QApplication.processEvents()

                        delta_x = 50
                        delta_y = 50
                        try_number = 0
                        maximum_tries = 15

                        while not piece_placed and try_number < maximum_tries:
                            for delta_pos in [(-delta_x, 0), (+delta_x, 0), (0, 0), (0, -delta_y), (0, +delta_y), (0, 0)]:
                                if not piece_placed:
                                    wiggle_x = best_x_position + delta_pos[0]
                                    wiggle_y = best_y_position + delta_pos[1]
                                    logger.debug("WIGGLE (%s,%s)", delta_pos[0], delta_pos[1])
                                    QApplication.processEvents()

                                    self.send_clearcore_command(f"m {wiggle_x},{wiggle_y},{robot.scoot_z}", blocking=True)
                                    QApplication.processEvents()

                                    self.linear_encoder_value = None
                                    while self.linear_encoder_value is None:
                                        self.send_gripper_command("d", update_position=False, blocking=True)
                                        QApplication.processEvents()
                                    localized_fully_placed_encoder_threshold = self.get_fully_placed_encoder_threshold(
                                        dst_x, dst_y, robot.scoot_z
                                    )
                                    if self.linear_encoder_value < localized_fully_placed_encoder_threshold:
                                        logger.debug(
                                            "PIECE STILL NOT FULLY PLACED (%s not > %s)",
                                            self.linear_encoder_value,
                                            localized_fully_placed_encoder_threshold,
                                        )
                                    else:
                                        piece_placed = True
                                        logger.debug("SCOOT AND WIGGLE WORKED!")

                                delta_x += 25
                                delta_y += 25
                                try_number += 1

            logger.debug("TURNING OFF VACUUM")
            self.send_clearcore_command("0", blocking=True, update_position=False)
            QApplication.processEvents()

            logger.debug("MOVING TO A SAFE TRAVEL Z")
            self.send_clearcore_command(f"m {dst_x},{dst_y},{robot.safe_travel_z}", blocking=True)
            QApplication.processEvents()
