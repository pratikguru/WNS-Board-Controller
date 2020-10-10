class Sensor:
    def __init__(self, sensorName : str = "input", sensorID :  int = 0, sensorType: str = ""):
        self.sensor_id : int = 0
        self.sensor_type : str = sensorType
        self.sensor_name : str = 0
        self.sensor_value  : int = 0
        self.Type = {
          "TANK_BALL" : 0x97,
          "DEPTH" : 0x98, 
          "FLOW"  : 0x99
        }
        

    def setSensorValue(self, sensor_value : int) -> None:
      self.sensor_value = sensor_value

    def getSensorReading(self, incoming) -> int:
      result = []
      for x in incoming:
        result.append(x)
      return result

    def returnSensorReadingFrame(self, id:int, type : str) -> bytearray:
      return bytearray([0xEA, 0xAD, id, self.Type[type], 0x0A])




class Actuator:
  def __init__(self, actuatorName : str = "motor", actuatorID : int = 0, actuatorType : str = ""):
    self.actuatorName : str = actuatorName
    self.actuatorID : int = actuatorID
    self.actuatorType : str = actuatorType
    self.actuatorStatus : int = 0 
    self.actuatorMode: int = 0
    self.actuatorDirection : int = -1
    self.actuatorPulse : int =  0 
    self.actuatorDegree : int = 0 
    self.Type = {
      "RELAY" : 0xA1,
      "VALVE" : 0xA2
    }


