# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created: Mon Sep 22 16:58:27 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1802, 1026)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.tableView = QtGui.QTableView(self.centralwidget)
        self.tableView.setGeometry(QtCore.QRect(370, 10, 1421, 921))
        self.tableView.setObjectName(_fromUtf8("tableView"))
        self.dofView = QtGui.QListView(self.centralwidget)
        self.dofView.setGeometry(QtCore.QRect(10, 10, 101, 671))
        self.dofView.setObjectName(_fromUtf8("dofView"))
        self.propertyView = QtGui.QListView(self.centralwidget)
        self.propertyView.setGeometry(QtCore.QRect(125, 11, 211, 921))
        self.propertyView.setObjectName(_fromUtf8("propertyView"))
        self.dofClearButton = QtGui.QPushButton(self.centralwidget)
        self.dofClearButton.setGeometry(QtCore.QRect(0, 690, 114, 32))
        self.dofClearButton.setObjectName(_fromUtf8("dofClearButton"))
        self.dofSelectButton = QtGui.QPushButton(self.centralwidget)
        self.dofSelectButton.setGeometry(QtCore.QRect(0, 720, 114, 32))
        self.dofSelectButton.setObjectName(_fromUtf8("dofSelectButton"))
        self.propClearButton = QtGui.QPushButton(self.centralwidget)
        self.propClearButton.setGeometry(QtCore.QRect(120, 930, 114, 32))
        self.propClearButton.setObjectName(_fromUtf8("propClearButton"))
        self.propSelectButton = QtGui.QPushButton(self.centralwidget)
        self.propSelectButton.setGeometry(QtCore.QRect(230, 930, 114, 32))
        self.propSelectButton.setObjectName(_fromUtf8("propSelectButton"))
        self.loadButton = QtGui.QPushButton(self.centralwidget)
        self.loadButton.setGeometry(QtCore.QRect(0, 780, 114, 32))
        self.loadButton.setObjectName(_fromUtf8("loadButton"))
        self.saveButton = QtGui.QPushButton(self.centralwidget)
        self.saveButton.setGeometry(QtCore.QRect(0, 810, 114, 32))
        self.saveButton.setObjectName(_fromUtf8("saveButton"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1802, 22))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "MainWindow", None, QtGui.QApplication.UnicodeUTF8))
        self.dofClearButton.setText(QtGui.QApplication.translate("MainWindow", "Clear all", None, QtGui.QApplication.UnicodeUTF8))
        self.dofSelectButton.setText(QtGui.QApplication.translate("MainWindow", "Select all", None, QtGui.QApplication.UnicodeUTF8))
        self.propClearButton.setText(QtGui.QApplication.translate("MainWindow", "Clear all", None, QtGui.QApplication.UnicodeUTF8))
        self.propSelectButton.setText(QtGui.QApplication.translate("MainWindow", "Select all", None, QtGui.QApplication.UnicodeUTF8))
        self.loadButton.setText(QtGui.QApplication.translate("MainWindow", "Load Params", None, QtGui.QApplication.UnicodeUTF8))
        self.saveButton.setText(QtGui.QApplication.translate("MainWindow", "Save as", None, QtGui.QApplication.UnicodeUTF8))

