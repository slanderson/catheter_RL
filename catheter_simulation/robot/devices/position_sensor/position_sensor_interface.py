"""TrakSTARInterface

communicates with the cytpes wrapping of the trakstar dlls (based on pytrak's code)
initializes the communication and performs the sensor reading
"""
import threading
# from pytrak.trakstar 
from robot.devices.position_sensor.trakstar_lib import atc3dg_functions_3 as api
import os
import ctypes
from time import time
import numpy as np
from robot.devices.position_sensor.trakstar_lib.udp_connection import UDPConnection

class PoseSensor(object):
    """ The trakSTAR interface"""
    def __init__(self):
        self._record = api.DOUBLE_POSITION_ANGLES_TIME_Q_RECORD_AllSensors_Four()
        self._precord = ctypes.pointer(self._record)
        self._file = None
        self._write_quality = None
        self._write_angles = None
        self._write_udp = None

        self.filename = None
        self.directory = None
        self.system_configuration = None
        self.attached_sensors = None
        self.init_time = None
        self._is_init = False

        self.measurement_rate = 100 #Hz
        self.udp = UDPConnection()
        print('UDP: ', self.udp)

    def __del__(self):
        self.close(ignore_error=True)

    def close(self, ignore_error=True):
        if not self.is_init:
            return
        print("* closing trakstar")
        self.attached_sensors = None
        self.system_configuration = None
        error_code = api.CloseBIRDSystem()
        if error_code != 0 and not ignore_error:
            self.error_handler(error_code)
        self._is_init = False


    def initialize(self):
        if self.is_init:
            return
        print( "* Initializing trakstar ...")
        error_code = api.InitializeBIRDSystem()
        if error_code != 0:
            self._error_handler(error_code)

        transmitter_id = ctypes.c_ushort(0)
        api.SetSystemParameter(api.SystemParameterType.SELECT_TRANSMITTER,
                               ctypes.pointer(transmitter_id), 2)
        # read in System configuration
        self.read_configurations(print_configuration=False)
        for x in range(self.system_configuration.numberSensors):
            api.SetSensorParameter(ctypes.c_ushort(x),
                                   api.SensorParameterType.DATA_FORMAT,
                ctypes.pointer(api.DataFormatType.DOUBLE_POSITION_ANGLES_TIME_Q),
                                   4)
        self.reset_timer()
        self._is_init = True

        #set System configuration
        self.set_system_configuration(measurement_rate=self.measurement_rate,
                                            max_range=36, 
                                            metric=True, 
                                            power_line=60, 
                                            report_rate=1, 
                                            print_configuration=True)

        

    def reset_timer(self):
        self.init_time = time()

    @property
    def is_init(self):
        """Returns True if trakstar is initialized"""
        return self._is_init

    def _error_handler(self, error_code):
        print( "** Error: ", error_code)
        txt = " " * 500
        pbuffer = ctypes.c_char_p(txt)
        api.GetErrorText(error_code, pbuffer, ctypes.c_int(500),
                         api.MessageType.VERBOSE_MESSAGE)
        print( pbuffer.value)
        self.close(ignore_error=True)
        raise RuntimeError("** trakSTAR Error")

    def readPoseSensor(self, write_data_file = False, secondSensor = False, justOneSensor = True):
        """gets most recent update of data"""
        error_code = api.GetAsynchronousRecord(api.ALL_SENSORS,
                                              self._precord, 4 * 1 * 64)
        if error_code != 0:
            self._error_handler(error_code)
        udp_data = self.udp.poll()
        sensor_read = np.zeros(14)
        if 1 in self.attached_sensors:
            sensor_read[:6] = np.array([self._record.x0, self._record.y0, self._record.z0,
                             self._record.a0, self._record.e0, self._record.r0])
            sensor_read[6]  = self._record.quality0 / 256
        if 2 in self.attached_sensors:
            sensor_read[7:13] = np.array([self._record.x1, self._record.y1, self._record.z1,
                             self._record.a1, self._record.e1, self._record.r0])
            sensor_read[13] = self._record.quality1 / 256
        return sensor_read

    def read_configurations(self, print_configuration=True):
        """read system and sensor configuration from device"""

        #system configuration
        sysconf = api.SYSTEM_CONFIGURATION()
        psys_conf = ctypes.pointer(sysconf)

        # report SystemConfiguration
        error_code = api.GetBIRDSystemConfiguration(psys_conf)
        if error_code != 0:
            self._error_handler(error_code)
        self.system_configuration = sysconf

        report_rate = 0

        # read attached sensors config
        sensor_conf = api.SENSOR_CONFIGURATION()
        psensor_conf = ctypes.pointer(sensor_conf)
        attached_sensors = []
        for cnt in range(self.system_configuration.numberSensors):
            error_code = api.GetSensorConfiguration(ctypes.c_ushort(cnt),
                                                    psensor_conf)
            if error_code != 0:
                self._error_handler(error_code)
            elif sensor_conf.attached:
                attached_sensors.append(cnt + 1)
        self.attached_sensors = attached_sensors

        if print_configuration:
            print( PoseSensor.configuration_text(self.attached_sensors,
                                 sysconf.measurementRate, sysconf.maximumRange,
                                 bool(sysconf.metric), sysconf.powerLineFrequency,
                                 report_rate))

    def set_system_configuration(self, measurement_rate=80, max_range=36,
                                 metric=True, power_line=60, report_rate=1,
                                 print_configuration=True):
        """
        measurement_rate in Hz: 20.0 < rate < 255.0
        max_range: valid values (in inches): 36.0, 72.0, 144.0
        metric: True (data in mm) or False (data in inches)
        power_line in Hz: 50.0 or 60.0 (frequency of the AC power source)
        report_rate: (int), between 1 and 127 --> report every 2nd, 3rd, etc. value
        """

        print( "* setting system configuration")

        mR = ctypes.c_double(measurement_rate)
        max_range = ctypes.c_double(max_range)
        metric = ctypes.c_int(int(metric))
        power_line = ctypes.c_double(power_line)
        report_rate = ctypes.c_ushort(report_rate)

        error_code = api.SetSystemParameter(
            api.SystemParameterType.MEASUREMENT_RATE,
            ctypes.pointer(mR), 8)
        if error_code != 0:
            self._error_handler(error_code)
        error_code = api.SetSystemParameter(
            api.SystemParameterType.MAXIMUM_RANGE,
            ctypes.pointer(max_range), 8)
        if error_code != 0:
            self._error_handler(error_code)
        error_code = api.SetSystemParameter(api.SystemParameterType.METRIC,
                                            ctypes.pointer(metric), 4)
        if error_code != 0:
            self._error_handler(error_code)
        error_code = api.SetSystemParameter(
            api.SystemParameterType.POWER_LINE_FREQUENCY,
            ctypes.pointer(power_line), 8)
        if error_code != 0:
            self._error_handler(error_code)
        error_code = api.SetSystemParameter(api.SystemParameterType.REPORT_RATE,
                                            ctypes.pointer(report_rate), 2)
        if error_code != 0:
            self._error_handler(error_code)

        self.read_configurations(print_configuration = print_configuration)

    @staticmethod
    def configuration_text(attached_sensors, measurementRate, maximumRange,
                           metric, powerLineFrequency, reportRate):
        """Creates text from configuration"""
        txt = ""
        txt = txt + "Measurement Rate: " + str(measurementRate)
        txt = txt + "\nattached sensors: " + repr(attached_sensors)
        txt = txt + "\nmaximum range: " + str(maximumRange) + " inches"
        txt = txt + "\nmetric data reporting: " + str(bool(metric))
        txt = txt + "\npower line frequency: " + str(powerLineFrequency) + " Hz"
        txt = txt + "\nreport rate: " +str(reportRate)
        return txt

