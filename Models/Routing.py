from .Sensing import Sensor, Actuator
import serial

class HardwareSetup:
    def __init__(self):
      self.actuation_id = []
      self.sensor_id = []
      self.actuation_1 = 0xAC
      self.actuation_2 = 0xAE
      self.sense_1 = 0xEA
      self.sense_2 = 0xAD

    def getNewSensor(self) -> Sensor:
      sensor = Sensor()
      return sensor
      
    def getNewActuator(self) -> Actuator:
      actuator = Actuator()
      return actuator 

    def appendSensorModel( sensor: Sensor ) -> int:
      self.sensor_id.append( sensor )
      return len(self.sensor_id)
    
    def appendActuationModel( actuator: Actuator ) -> int:
      self.actuation_id.append(actuator)
      return len(self.actuation_id)

    

if __name__ == "__main__":
  HWS = HardwareSetup()
  
  tank_ball_1 = HWS.getNewSensor()




  obj          = serial.Serial()
  obj.baudrate = 9600
  obj.port     = "/dev/cu.usbmodem14101"
  obj.open()
  obj.timeout = 3

  obj.readline()
  # Fetch sensor reading.
  
  incoming = obj.write(tank_ball_1.returnSensorReadingFrame(0x47, "TANK_BALL"))
  incoming = obj.readline()
  print(tank_ball_1.getSensorReading(incoming))