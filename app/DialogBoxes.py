from PySide6.QtWidgets import QMessageBox


# these functions create the dialog boxes (popups) for the user
class QuestionDialog(QMessageBox):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        self.setIcon(QMessageBox.Question)


class InformationDialog(QMessageBox):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setStandardButtons(QMessageBox.Ok)
        self.setIcon(QMessageBox.Information)


class WarningDialog(QMessageBox):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setStandardButtons(QMessageBox.Ok)
        self.setIcon(QMessageBox.Warning)


class CriticalDialog(QMessageBox):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setStandardButtons(QMessageBox.Ok)
        self.setIcon(QMessageBox.Critical)