from PySide6 import QtCore
from PySide6.QtCore import Qt


# this class uses a Pandas dataframe to create a tableview
class PandasModel(QtCore.QAbstractTableModel):
    def __init__(self, data):
        super(PandasModel, self).__init__()
        self._data = data

        self.colors = dict()

    def data(self, index, role):
        if role == Qt.DisplayRole:
            value = self._data.iloc[index.row(), index.column()]
            return str(value)
        if role == Qt.BackgroundRole:
                color = self.colors.get((index.row(), index.column()))
                if color is not None:
                    return color

    def rowCount(self, index):
        return self._data.shape[0]

    def columnCount(self, index):
        return self._data.shape[1]

    def headerData(self, section, orientation, role):
        # section is the index of the column/row.
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return str(self._data.columns[section])

            if orientation == Qt.Vertical:
                return str(self._data.index[section])
    
    # this function sets the conditions for editing data
    def setData(self, index, value, role):
        if not index.isValid():
            return False
        if role != QtCore.Qt.EditRole:
            return False
        row = index.row()
        if row < 0 or row >= len(self._data.values):
            return False
        column = index.column()
        if column < 0 or column >= self._data.columns.size:
            return False
        if column == 0 and abs(int(value)) > 1600: # prevent values greater than 1600 from being entered into the load column
            return False
        if column == 1 and int(value) < 3: # prevent values less than 3 from being entered into the time column
            return False
        self._data.values[row][column] = value
        self.dataChanged.emit(index, index)
        return True

    def flags(self, index):
        flags = super(self.__class__,self).flags(index)
        flags |= QtCore.Qt.ItemIsEditable
        flags |= QtCore.Qt.ItemIsSelectable
        flags |= QtCore.Qt.ItemIsEnabled
        flags |= QtCore.Qt.ItemIsDragEnabled
        flags |= QtCore.Qt.ItemIsDropEnabled
        return flags
    
    def getData(self):
        return self._data
    
    def changeColor(self, row, column, color):
        ix = self.index(row, column)
        self.colors[(row, column)] = color
        self.dataChanged.emit(ix, ix, (Qt.BackgroundRole,))