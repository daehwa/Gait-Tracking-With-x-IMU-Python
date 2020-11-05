import os
import sys
this_dir = os.path.dirname(__file__)
sys.path.append(this_dir)
import CalInertialAndMagneticDataClass as calIM

class xIMUdataClass():
    def __init__(self, filename='', sr_type='InertialMagneticSampleRate', sr = 20000/3):
        self.FileNamePrefix = filename
        self.ErrorData = []
        self.CommandData = []
        self.RegisterData = []
        self.DateTimeData = []
        self.RawBatteryAndThermometerData = []
        self.CalBatteryAndThermometerData = []
        self.RawInertialAndMagneticData = []
        self.CalInertialAndMagneticData = []
        self.QuaternionData = []
        self.RotationMatrixData = []
        self.EulerAnglesData = []
        self.DigitalIOdata = []
        self.RawAnalogueInputData = []
        self.CalAnalogueInputData = []
        self.PWMoutputData = []
        self.RawADXL345busData = []
        self.CalADXL345busData = []
        self.sr_type = sr_type
        self.sr = sr
        self.load_data()
        self.apply_samplerate()
        # self.plot()

    def load_data(self):
        dataImported = False
        try:
            self.CalInertialAndMagneticData = calIM.get_obj(self.FileNamePrefix, self.sr)
            dataImported = True
        except:
            print('')
        if not dataImported:
            print('No data was imported.')
            exit()

    def apply_samplerate(self):
        if self.sr_type == 'InertialMagneticSampleRate':
            try:
                h = self.CalInertialAndMagneticData
                h.SampleRate = self.sr
            except:
                print('')
        else:
            print('Invalid argument.')
            exit()

    # def plot(self):
    #     try:
    #         self.CalInertialAndMagneticData.plot() #########################
    #     except:
    #         continue