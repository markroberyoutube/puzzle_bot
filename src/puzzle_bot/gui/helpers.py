import logging
import posixpath

import exif
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5.QtWidgets import QFileDialog, QLabel, QLineEdit

logger = logging.getLogger(__name__)


class UiHelpers:
    def pixmap_from_image(self, image_path: str) -> QPixmap | None:
        logger.debug("[UiHelpers.pixmap_from_image] Opening image path: %s", image_path)
        try:
            pixmap = QPixmap(image_path)
        except Exception as e:
            logger.error("[UiHelpers.pixmap_from_image] Could not open photo at %s: %s", image_path, e)
            return None

        try:
            with open(image_path, "rb") as image_file:
                img = exif.Image(image_file)
                orientation = img.orientation
                if orientation == 1:
                    pass
                elif orientation == 3:
                    pixmap = pixmap.transformed(QTransform().rotate(180))
                elif orientation == 6:
                    pixmap = pixmap.transformed(QTransform().rotate(90))
                elif orientation == 8:
                    pixmap = pixmap.transformed(QTransform().rotate(270))
                else:
                    logger.error("[UiHelpers.pixmap_from_image] Unsupported jpg orientation: %s", orientation)
        except Exception:
            pass

        return pixmap

    def display_image(self, image_path: str, label_area: QLabel | None) -> QPixmap | None:
        logger.debug("[UiHelpers.display_image] displaying image at: %s", image_path)
        if label_area is None:
            return None
        pixmap = self.pixmap_from_image(image_path)
        if pixmap is None:
            return None
        pixmap = pixmap.scaledToHeight(label_area.height(), Qt.SmoothTransformation)
        label_area.setPixmap(pixmap)
        label_area.setAlignment(Qt.AlignCenter)
        return pixmap

    def browse_for_image(self, textbox: QLineEdit, label_area: QLabel | None = None) -> None:
        preselected_filepath = textbox.text()
        directory = posixpath.split(preselected_filepath)[0] if preselected_filepath else ""
        new_filepath, _ = QFileDialog.getOpenFileName(
            self,  # type: ignore[arg-type]
            "Open Image",
            directory,
            "Images (*.png *.jpg)",
        )
        if new_filepath:
            textbox.setText(new_filepath)
            self.display_image(new_filepath, label_area)

    def browse_for_directory(self, textbox: QLineEdit) -> None:
        directory = QFileDialog.getExistingDirectory(
            self,  # type: ignore[arg-type]
            "Open Directory",
            textbox.text(),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if directory:
            textbox.setText(directory)
