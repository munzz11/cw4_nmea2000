#!/usr/bin/env python3

import rospy
import pynmea2
import datetime
from nmea_msgs.msg import Sentence
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

def diag_log(Array):

    #TODO: Calibrate OK/Warning/Error levels for Fuel, Env temp and Exhaust temp 
    ######Warning Ranges########

    #Fuel Level Lower Bounds
    fuel_safe = 0.015
    fuel_warn = 0.01

    #Env Temp Upper Bounds
    envt_safe = 35
    envt_warn = 50

    #Exhaust Temp Upper Bounds
    exaust_safe = 500
    exhaust_warn = 700

    diagnostic_pub = rospy.Publisher('/diagnostics',DiagnosticArray,queue_size=10)
    ds = DiagnosticStatus()
    ds.hardware_id = 'ben'
    diag_array = DiagnosticArray()
    #diag_array.header = 'test'
    diag_array.header.stamp = rospy.Time.now()

    if Array[0] == '$IIXDR':
        if Array[1] == 'V':
            ds.name = 'Fluid Level'
            ds.values.append(KeyValue('Fuel Level(L)', Array[2]))
            if float(Array[2]) > fuel_safe:
                ds.level = DiagnosticStatus.OK
            elif float(Array[2]) < fuel_safe and float(Array[2]) > fuel_warn :
                ds.level = DiagnosticStatus.WARN
            elif float(Array[2]) < fuel_warn:
                ds.level = DiagnosticStatus.ERROR
            diag_array.status.append(ds)
            diagnostic_pub.publish(diag_array)
        elif Array[1] == 'C':
            if Array[4].startswith('ENV'):
                ds.name = 'Environmental Parameters'
                ds.values.append(KeyValue('Enviroment Temp(C)', Array[2])) #TODO: Determine where enviroment temp is being read from
                if float(Array[2]) < envt_safe:
                    ds.level = DiagnosticStatus.OK
                elif float(Array[2]) > envt_safe and float(Array[2]) < envt_warn:
                    ds.level = DiagnosticStatus.WARN
                elif float(Array[2]) > envt_warn:
                    ds.level = DiagnosticStatus.ERROR
            elif Array[4].startswith('EXHAUST'):
                ds.name = 'Environmental Parameters'
                ds.values.append(KeyValue('Exhaust Temp (C)', Array[2])) 
                if float(Array[2]) < exaust_safe:
                    ds.level = DiagnosticStatus.OK
                elif float(Array[2]) > exaust_safe and float(Array[2]) < exhaust_warn:
                    ds.level = DiagnosticStatus.WARN
                elif float(Array[2]) > exhaust_warn:
                    ds.level = DiagnosticStatus.ERROR
            diag_array.status.append(ds)
            diagnostic_pub.publish(diag_array)

    elif Array[0] == '$IIHDG':
        ds.name = 'Vessel Heading'
        ds.values.append(KeyValue('Magetic Sensor Heading', Array[1]))
        ds.values.append(KeyValue('Magetic Deviation', Array[2]))
        ds.values.append(KeyValue('Deviation E/W', Array[3]))
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$IIROT':
        ds.name = 'Rate of Turn'
        ds.values.append(KeyValue('Rate of Turn', Array[1]))
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$IIVHW':
        ds.name = 'Speed'
        ds.values.append(KeyValue('Heading Degrees True',Array[1]))
        ds.values.append(KeyValue('Heading Degrees Mag.',Array[3]))
        ds.values.append(KeyValue('Water Speed (Knots)',Array[5]))
        ds.values.append(KeyValue('Water Speed (Kph)',Array[7]))
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$PMAREPD':
        ds.name = 'Engine Parameters'
        ds.values.append(KeyValue('Engine Number',Array[1]))
        ds.values.append(KeyValue('Engine Oil Press (mBar)',Array[2]))
        ds.values.append(KeyValue('Engine Oil Temp (C)',Array[3]))
        ds.values.append(KeyValue('Engine Temp (C)',Array[4]))
        ds.values.append(KeyValue('Alternator Potential (V) ',Array[5]))
        ds.values.append(KeyValue('Fuel Rate (lph)',Array[6]))
        ds.values.append(KeyValue('Total Engine Hours (hrs)',Array[7]))
        ds.values.append(KeyValue('Coolant Pressure (mbar)',Array[8]))
        ds.values.append(KeyValue('Fuel Pressure (mbar) ',Array[9]))
        ds.values.append(KeyValue('Engine Discrete Status 1 ',Array[10]))
        ds.values.append(KeyValue('Engine Discrete Status 2 ',Array[11]))
        ds.values.append(KeyValue('Percent Engine Load ',Array[12]))
        ds.values.append(KeyValue('Percent Eng. Torque',Array[13]))
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$PMAROUT':
        ds.name = 'Attitude'
        ds.values.append(KeyValue('Roll',Array[2]))
        ds.values.append(KeyValue('Pitch',Array[3]))
        ds.values.append(KeyValue('Yaw',Array[4]))
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    else:
        ds.values.append(KeyValue('unknown acronym' , Array[0]))
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)


def callback(data):
    response = data.sentence.split(',')
    diag_log(response)

def listener():
    rospy.init_node('nmea_conv')
    rospy.Subscriber("/ben/sensors/nmea_sentence", Sentence, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
