import numpy as np

# class DataBaseClass():
#     def __init__(self):

# class TimeSeriesDataBaseClass():
#     def __init__(self):
#         self.DataBase = DataBaseClass()

# class InertialAndMagneticDataBaseClass():
#     def __init__(self, data):
#         self.TimeSeriesDataBase = TimeSeriesDataBaseClass()
#         set_dataformat(data)

#     def set_dataformat(self, data):
#         self.gyroscope = data[:,1:4]
#         self.accelerometer = data[:,4:7]
#         self.magnetometer = data[:,7:10]

class CalInertialAndMagneticDataClass():
    def __init__(self, filename, sr):
        self.FileNameAppendage = '_CalInertialAndMag.csv'
        self.SampleRate = sr
        self.GyroscopeUnits = 'degrees/s';
        self.AccelerometerUnits = 'g';
        self.MagnetometerUnits = 'G';
        data = np.loadtxt(filename+self.FileNameAppendage,
                            delimiter=',',skiprows=1)
        # self.InertialAndMagneticDataBase = InertialAndMagneticDataBaseClass(data)
        self.set_dataformat(data)
        self.packetNum = data.shape[0]
        self.Time = np.arange(0,self.packetNum)*(1/self.SampleRate)

    def set_dataformat(self, data):
        self.gyroscope = data[:,1:4]
        self.accelerometer = data[:,4:7]
        self.magnetometer = data[:,7:10]



def get_obj(filename, sr):
    obj = CalInertialAndMagneticDataClass(filename, sr)
    return obj