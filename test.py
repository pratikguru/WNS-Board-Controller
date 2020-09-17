
"""
The purpose of this script is for controlling servo accutators with a GUI.
Forward kinematics of an arm can be simulated with this controller software.
The script uses the serial library for communicating over the serial monitor of the host
computer.
@author: Pratik Gurudatt
@date  : 19/04/2019
"""
import copy
import time
import sys
import serial
import json
import glob
import serial.tools.list_ports
try:
    from   PyQt5           import QtCore
    from   PyQt5.QtCore    import pyqtSlot
    from   PyQt5           import QtWidgets, QtGui
    from   PyQt5.QtWidgets import QApplication, QDialog, QMainWindow, QMessageBox
    from   PyQt5.QtWidgets import *
    from   PyQt5.uic       import loadUi
except Exception as e:
    print ("Requirement Error! The PyQt Libs are not installed.")
    import os
import math

class App(QMainWindow):
    def __init__(self):
        super(App, self).__init__()
        try:
            loadUi('main.ui', self)
        except Exception as e:
            print (e)
            print ("main.ui could not be found!")
        self.serialObj: serial      = serial.Serial()
        self.serialPortList: list = []

        self.currentState = False

        ports = (serial.tools.list_ports.comports())
        self.listWidget = QListWidget(self.listWidget) 

        for port, desc, hwid in sorted(ports):
          self.comboBox.addItem(port)
          self.serialPortList.append(port)

    def run(self):
        print ("Main loop running!")
        self.pushButton_3.clicked.connect(self.handleConnect)
        self.spinBox.valueChanged.connect(self.handleBaudRate)
        self.comboBox.currentIndexChanged.connect(self.handleComPortChanged)
        self.pushButton.clicked.connect(self.toggleDirection) 
        self.pushButton_2.clicked.connect(self.toggleInit)
        self.pushButton_4.clicked.connect(self.getStatus)
        self.pushButton_5.clicked.connect(self.openValve)
        self.pushButton_6.clicked.connect(self.closeValve)
        self.dial.valueChanged.connect(self.handleDialChange)
        self.radioButton.toggled.connect(self.handleRadioButtonChange)


    def handleRadioButtonChange(self):
      print (self.radioButton.isChecked())


    def handleDialChange(self, e):
      print(e)
      self.label_4.setText(str(e) + "uS")

    def openValve(self):
      print ("Opening valve")
      if self.serialObj.is_open:
        self.serialObj.write(('o').encode('ascii'))
        if self.radioButton.isChecked():
          incoming = self.serialObj.readline().decode('utf-8')
          listWidgetItem = QListWidgetItem(incoming) 
          self.listWidget.addItem(listWidgetItem)
          print(incoming)
    
    def closeValve(self):
      print ("Closing valve")
      if self.serialObj.is_open:
        self.serialObj.write(('c').encode('ascii'))
        if self.radioButton.isChecked():
          incoming = self.serialObj.readline().decode('utf-8')
          listWidgetItem = QListWidgetItem(incoming) 
          self.listWidget.addItem(listWidgetItem)
          print(incoming)
        
    def getStatus(self):
      print("Fetching status")
      if self.serialObj.is_open:
        self.serialObj.write(('g').encode('ascii'))
        if self.radioButton.isChecked():
          incoming = self.serialObj.readline().decode('utf-8')
          listWidgetItem = QListWidgetItem(incoming) 
          self.listWidget.addItem(listWidgetItem)
          print(incoming)

    def toggleInit(self):
      print("Toggle status")
      if self.serialObj.is_open:
        self.currentState = not self.currentState
        if self.currentState:
          self.serialObj.write(('s').encode('ascii'))
        else:
          self.serialObj.write(('x').encode('ascii'))
        if self.radioButton.isChecked():
          incoming = self.serialObj.readline().decode('utf-8')
          listWidgetItem = QListWidgetItem(incoming) 
          self.listWidget.addItem(listWidgetItem)
          print(incoming)

    def toggleDirection(self):
      print("toggling direction")
      if self.serialObj.is_open:
        self.serialObj.write(('d').encode('ascii'))
        if self.radioButton.isChecked():
          incoming = self.serialObj.readline().decode("utf-8")
          listWidgetItem = QListWidgetItem(incoming) 
          self.listWidget.addItem(listWidgetItem)
          print(incoming)

    def handleBaudRate(self, e):
      self.baudRate = e 
      self.serialObj.baudrate = self.baudRate

    def handleComPortChanged(self, e):
      print(e)
      self.serialObj.port = self.serialPortList[e]
      
    def handleConnect(self):
      print ("Connection clicked.")
      self.serialObj.timeout = 2
      if self.serialObj.is_open:
        print("Port Already Open")
        return
      else:
        self.serialObj.open()





def main():
    app = QApplication(sys.argv)
    widget = App()
    widget.show()
    widget.run()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()