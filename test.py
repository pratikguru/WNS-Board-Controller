
"""
The purpose of this script is for controlling servo accutators with a GUI.
The script uses the serial library for communicating over the serial monitor of the host
computer.
@author: Pratik Gurudatt
@date  : 17/09/2020
"""
import copy
import time
import sys
import serial
import json
import glob
import time
import serial.tools.list_ports
from Models.Routing import (HardwareSetup) 
from Models.Sensing import (Actuator, Sensor)


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
        self.serialObj.timeout = 4
        self.serialPortList: list = []
        
        self.currentState = False
        
        self.relayID = [0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A]
        self.sensorID = [0x47, 0x48, 0x48, 0x4A, 0x4B, 0x4C, 0x4D]
        self.ultrasoundSensorID = [0x21, 0x22, 0x23, 0x24]
        self.flowSensorsID = [ 0x3D, 0x3E, 0x3F, 0x40, 0x41]

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

        self.dial.valueChanged.connect(self.handleDialChange)
        self.radioButton.toggled.connect(self.handleRadioButtonChange)


        #Fetching sensor readings.
        #Relay On controls: 1 - 4
        self.pushButton_7.clicked.connect(lambda: self.toggleRelayOn("1"))
        self.pushButton_8.clicked.connect(lambda: self.toggleRelayOn("2"))
        self.pushButton_10.clicked.connect(lambda: self.toggleRelayOn("3"))
        self.pushButton_11.clicked.connect(lambda: self.toggleRelayOn("4"))
        #Relay On controls : 5 - 8
        self.pushButton_9.clicked.connect(lambda:  self.toggleRelayOn("5"))
        self.pushButton_18.clicked.connect(lambda: self.toggleRelayOn("6"))
        self.pushButton_17.clicked.connect(lambda: self.toggleRelayOn("7"))
        self.pushButton_20.clicked.connect(lambda: self.toggleRelayOn("8"))

        #Relay Off controls: 1 - 4
        self.pushButton_12.clicked.connect(lambda: self.toggleRelayOff("1"))
        self.pushButton_14.clicked.connect(lambda: self.toggleRelayOff("2"))
        self.pushButton_13.clicked.connect(lambda: self.toggleRelayOff("3"))
        self.pushButton_15.clicked.connect(lambda: self.toggleRelayOff("4"))
        #Relay Off controls: 5 - 8
        self.pushButton_19.clicked.connect(lambda: self.toggleRelayOff("5"))
        self.pushButton_22.clicked.connect(lambda: self.toggleRelayOff("6"))
        self.pushButton_21.clicked.connect(lambda: self.toggleRelayOff("7"))
        self.pushButton_16.clicked.connect(lambda: self.toggleRelayOff("8"))


        #Opening and closing valves
        self.pushButton_39.clicked.connect(lambda: self.toggleValveOn(0x51))
        self.pushButton_40.clicked.connect(lambda: self.toggleValveOff(0x51)) 

        #Fetching sensor values
        #Magnetic sensors
        self.pushButton_5.clicked.connect(lambda : self.fetchSensorReading(id = self.sensorID[0], name = "TANK_BALL"))
        self.pushButton_6.clicked.connect(lambda: self.fetchSensorReading(id = self.sensorID[1], name = "TANK_BALL"))
        self.pushButton_23.clicked.connect(lambda: self.fetchSensorReading(id = self.sensorID[2], name="TANK_BALL"))
        self.pushButton_24.clicked.connect(lambda: self.fetchSensorReading(id = self.sensorID[3], name="TANK_BALL"))
        self.pushButton_25.clicked.connect(lambda: self.fetchSensorReading(id = self.sensorID[4], name="TANK_BALL"))
        self.pushButton_26.clicked.connect(lambda: self.fetchSensorReading(id = self.sensorID[5], name="TANK_BALL"))
        self.pushButton_27.clicked.connect(lambda: self.fetchSensorReading(id = self.sensorID[6], name="TANK_BALL"))
        self.pushButton_28.clicked.connect(lambda: self.fetchSensorReading(id = self.sensorID[7], name="TANK_BALL"))


        #Ultrasound sensors
        self.pushButton_29.clicked.connect(lambda : self.fetchSensorReading(id=self.ultrasoundSensorID[0], name="DEPTH"))
        self.pushButton_30.clicked.connect(lambda: self.fetchSensorReading(id=self.ultrasoundSensorID[1], name="DEPTH"))
        self.pushButton_31.clicked.connect(lambda: self.fetchSensorReading(id=self.ultrasoundSensorID[2], name="DEPTH"))
        self.pushButton_32.clicked.connect(lambda: self.fetchSensorReading(id=self.ultrasoundSensorID[3], name="DEPTH"))

        #Flow sensors
        self.pushButton_33.clicked.connect(lambda: self.fetchSensorReading(id=self.flowSensorsID[0], name="FLOW"))
        self.pushButton_36.clicked.connect(lambda: self.fetchSensorReading(id=self.flowSensorsID[1], name="FLOW"))
        self.pushButton_35.clicked.connect(lambda: self.fetchSensorReading(id=self.flowSensorsID[2], name="FLOW"))
        self.pushButton_34.clicked.connect(lambda: self.fetchSensorReading(id=self.flowSensorsID[3], name="FLOW"))
        self.pushButton_37.clicked.connect(lambda: self.fetchSensorReading(id=self.flowSensorsID[4], name="FLOW"))
        #self.pushButton_38.clicked.connect(lambda: self.fetchSensorReading(id=self.flowSensorsID[5], name="FLOW"))

    def fetchSensorReading(self, id, name):
      sensor = Sensor(sensorName=name, sensorID=id, sensorType=name)
      print(sensor.returnSensorReadingFrame(id=id))
      incoming = self.serialObj.write(sensor.returnSensorReadingFrame(id=id))
      if self.radioButton.isChecked():
          incoming = self.serialObj.readline()
          for x in incoming:
            listWidgetItem = QListWidgetItem(str(x)) 
            self.listWidget.addItem(listWidgetItem)
            print(str(x))
      
    def toggleValveOn(self, id):
      valve = Actuator(actuatorName="Valve", actuatorID=id, actuatorType="VALVE")
      print(valve.returnActuatorWriteFrameValve(valveState=1))
      incoming = self.serialObj.write( valve.returnActuatorWriteFrameValve(valveState=0x01) )
      
      if self.radioButton.isChecked():
          incoming = self.serialObj.readline()
          for x in incoming:
            listWidgetItem = QListWidgetItem(str(x)) 
            self.listWidget.addItem(listWidgetItem)
            print(str(x))

    def toggleValveOff(self, id):
      valve = Actuator(actuatorName="Valve", actuatorID=id, actuatorType="VALVE")
      print(valve.returnActuatorWriteFrameValve(valveState=0))
      incoming = self.serialObj.write( valve.returnActuatorWriteFrameValve(valveState=0x02) )
      
      if self.radioButton.isChecked():
          incoming = self.serialObj.readline()
          for x in incoming:
            listWidgetItem = QListWidgetItem(str(x)) 
            self.listWidget.addItem(listWidgetItem)
            print(str(x))


    def toggleRelayOff(self, counter):
      
      relay = Actuator(actuatorName="Relay", actuatorID=self.relayID[int(counter)-1], actuatorType="RELAY" )
      print(relay.returnActuatorWriteFrameRelay(0x00))
      
      incoming = self.serialObj.write( relay.returnActuatorWriteFrameRelay(0x00) )
      
      if self.radioButton.isChecked():
          incoming = self.serialObj.readline()
          for x in incoming:
            listWidgetItem = QListWidgetItem(str(x)) 
            self.listWidget.addItem(listWidgetItem)
            print(str(x))
      

    def toggleRelayOn(self, counter):
      
      relay = Actuator(actuatorName="Relay", actuatorID=self.relayID[int(counter)-1], actuatorType="RELAY" )
      print(relay.returnActuatorWriteFrameRelay(0x01))
      
      incoming = self.serialObj.write( relay.returnActuatorWriteFrameRelay(0x01) )
      if self.radioButton.isChecked():
          incoming = self.serialObj.readline()
          for x in incoming:
            listWidgetItem = QListWidgetItem(str(x)) 
            self.listWidget.addItem(listWidgetItem)
            print(str(x))
      
      

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
          incoming = self.serialObj.readline()
          listWidgetItem = QListWidgetItem(incoming) 
          self.listWidget.addItem(listWidgetItem)
          print(incoming)
    
    def closeValve(self):
      print ("Closing valve")
      if self.serialObj.is_open:
        self.serialObj.write(('c').encode('ascii'))
        if self.radioButton.isChecked():
          incoming = self.serialObj.readline()
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