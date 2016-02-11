# Import Qt modules
from PyQt4 import QtCore,QtGui

# Import the compiled UI module
from mainwindowUi import Ui_MainWindow

import os,sys

import GdcCardStatus


class Main(QtGui.QMainWindow):
    def __init__(self, *args):
        QtGui.QMainWindow.__init__(self)

        # This is always the same
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)


        ##tablemodel = MyTableModel(my_array, self)
        tablemodel = GdcTableModel(self)
        self.ui.tableView.setModel(tablemodel)
        QtCore.QObject.connect(tablemodel, QtCore.SIGNAL("hideColumn"), self.ui.tableView.hideColumn)
        QtCore.QObject.connect(tablemodel, QtCore.SIGNAL("showColumn"), self.ui.tableView.showColumn)
        QtCore.QObject.connect(tablemodel, QtCore.SIGNAL("hideRow"), self.ui.tableView.hideRow)
        QtCore.QObject.connect(tablemodel, QtCore.SIGNAL("showRow"), self.ui.tableView.showRow)
        
        
        #display the list of DOFs - empty at the beginning
        dof_list_model = MyListModel([], self)
        self.ui.dofView.setModel(dof_list_model)
        dof_list_model.itemChanged.connect(tablemodel.changeRows)
        QtCore.QObject.connect(tablemodel, QtCore.SIGNAL("updateList"), dof_list_model.updateList)
                
        #display the list of properties
        property_list_model = MyListModel(GdcCardStatus.property_list, self)
        self.ui.propertyView.setModel(property_list_model)
        property_list_model.itemChanged.connect(tablemodel.changeColumn)


        #connect clear and select buttons to adequate slots
        self.ui.propClearButton.clicked.connect(property_list_model.unselectall)
        self.ui.dofClearButton.clicked.connect(dof_list_model.unselectall)
        self.ui.propSelectButton.clicked.connect(property_list_model.selectall)
        self.ui.dofSelectButton.clicked.connect(dof_list_model.selectall)

        #file load
        self.ui.loadButton.clicked.connect(tablemodel.loadFile)
        self.ui.saveButton.clicked.connect(tablemodel.saveFile)



##class to create the model view
class MyListModel(QtGui.QStandardItemModel):
    def __init__(self, datain=[], parent=None, *args):
        QtGui.QStandardItemModel.__init__(self, parent)
        self.list_params = datain
        for params in self.list_params:
            item = QtGui.QStandardItem(params)
            item.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
            item.setData(QtCore.QVariant(QtCore.Qt.Checked), QtCore.Qt.CheckStateRole)
            self.appendRow(item)
        
    def unselectall(self):
        for i in range(0,self.rowCount()):
            self.item(i).setData(QtCore.QVariant(QtCore.Qt.Unchecked), QtCore.Qt.CheckStateRole)
            self.emit(QtCore.SIGNAL("itemChanged"),self.item(i))
            
    def selectall(self):
        for i in range(0,self.rowCount()):
            self.item(i).setData(QtCore.QVariant(QtCore.Qt.Checked), QtCore.Qt.CheckStateRole)
            self.emit(QtCore.SIGNAL("itemChanged"),self.item(i))
    
    def updateList(self, list):
        self.beginResetModel()
        self.list_params = list
        self.clear()
        for params in self.list_params:
            item = QtGui.QStandardItem(params)
            item.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
            item.setData(QtCore.QVariant(QtCore.Qt.Checked), QtCore.Qt.CheckStateRole)
            self.appendRow(item)
        self.endResetModel()
        
        
##the model for the table         
class GdcTableModel(QtCore.QAbstractTableModel):
    def __init__(self, parent=None, *args):
        QtCore.QAbstractTableModel.__init__(self, parent, *args)
        self.gdc_status = GdcCardStatus.GdcCardStatus()
        
    #number of rows
    def rowCount(self, parent):
        return len(self.gdc_status.data)

    #number of columns
    def columnCount(self, parent):
        if(len(self.gdc_status.data) != 0):
            return len(self.gdc_status.data[0])
        else:
            return 0

    #return the data to diplay in the table
    def data(self, index, role):
        if not index.isValid():
            return QtCore.QVariant()
        elif role != QtCore.Qt.DisplayRole:
            return QtCore.QVariant()
        else:
            if (GdcCardStatus.property_list[index.column()] == 'status0' or
               GdcCardStatus.property_list[index.column()] == 'status1' or
               GdcCardStatus.property_list[index.column()] == 'status2' or
               GdcCardStatus.property_list[index.column()] == 'status3' or 
               GdcCardStatus.property_list[index.column()] == 'mode_byte' or
               GdcCardStatus.property_list[index.column()] == 'invert_byte'):
                ret_string = '0x%(stat)x' % {'stat': int(self.gdc_status.data[index.row()][index.column()])}
            else:
                ret_string = self.gdc_status.data[index.row()][index.column()]
                
            return QtCore.QVariant(ret_string)
    
    #sets the names of the column and rows
    def headerData(self, section, orientation, role):
        if orientation == QtCore.Qt.Horizontal and role == QtCore.Qt.DisplayRole:
            return QtCore.QVariant(GdcCardStatus.property_list[section])
        elif orientation == QtCore.Qt.Vertical and role == QtCore.Qt.DisplayRole:
            return QtCore.QVariant(self.gdc_status.names[section])
        return QtCore.QVariant()
    
    #show or hide a row depending on the item
    def changeRows(self, item):
        #find the row number
        for index, it in enumerate(self.gdc_status.names):
            if item.text() == it:
                if item.checkState() == QtCore.Qt.Checked:
                    self.emit(QtCore.SIGNAL("showRow"),index)
                    return
                else:
                    self.emit(QtCore.SIGNAL("hideRow"),index)
                    return
    
    #shows or hides a column depending on the selection       
    def changeColumn(self, item):
        #find the column number
        for index, it in enumerate(GdcCardStatus.property_list):
            if item.text() == it:
                if item.checkState() == QtCore.Qt.Checked:
                    self.emit(QtCore.SIGNAL("showColumn"),index)
                    return
                else:
                    self.emit(QtCore.SIGNAL("hideColumn"),index)
                    return

    ##overloading
    def setData(self, index, value, role):
        if not index.isValid():
            return False
        elif role != QtCore.Qt.EditRole:
            return False
        else:
            
            self.gdc_status.data[index.row()][index.column()] = value.toString()
            self.emit(QtCore.SIGNAL("dataChanged"))
            print self.gdc_status.names[index.row()] + ' ' + GdcCardStatus.property_list[index.column()] + ' changed to: ' + value.toString()
            return True
    
    ##need to implement the function to edit items
    def flags(self, index):
        if not index.isValid():
            return QtCore.Qt.NoItemFlags
        else:
            return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsEnabled
            
            
    ##load data from a file and update the table and the DOF list
    def loadFile(self):
        filename = QtGui.QFileDialog.getOpenFileName(None, 'select file to load', '.')
        if filename != QtCore.QString():
            print 'loading file: ' + filename
            self.beginResetModel()
            self.gdc_status.load(filename)
            self.endResetModel()
            self.emit(QtCore.SIGNAL("dataChanged"))
            self.emit(QtCore.SIGNAL("updateList"), self.gdc_status.names)
    
    
    ##save the current gdc status to a file
    def saveFile(self):
        filename = QtGui.QFileDialog.getSaveFileName(None, 'select file to save', '.')
        if filename != QtCore.QString():
            print 'saving in ' + filename
            self.gdc_status.save(filename)
        
def main():
    # Again, this is boilerplate, it's going to be the same on
    # almost every app you write
    app = QtGui.QApplication(sys.argv)
    window=Main()
    window.show()
    # It's exec_ because exec is a reserved word in Python
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
