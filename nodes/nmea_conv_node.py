#!/usr/bin/env python3

import rospy
import pynmea2
import datetime
from nmea_msgs.msg import Sentence
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

def diag_log(Array):

    #TODO: Move all config values to config server
    
    diagnostic_pub = rospy.Publisher('/diagnostics',DiagnosticArray,queue_size=10)
    ds = DiagnosticStatus()
    ds.hardware_id = 'ben'
    diag_array = DiagnosticArray()
    diag_array.header.stamp = rospy.Time.now()

    if Array[0] == '$IIXDR':
        try:
            if Array[1] == 'V':
                ds.name = 'Fluid Level'
                ds.values.append(KeyValue('Fuel Level(L)', Array[2]))
                diag_array.status.append(ds)
                diagnostic_pub.publish(diag_array)
            elif Array[1] == 'C':
                if Array[4].startswith('ENV'):
                    ds.name = 'Environmental Parameters'
                    ds.values.append(KeyValue('Enviroment Temp(C)', Array[2])) #TODO: Determine where enviroment temp is being read from
                elif Array[4].startswith('EXHAUST'):
                    ds.name = 'Environmental Parameters'
                    ds.values.append(KeyValue('Exhaust Temp (C)', Array[2])) 
        except:
            ds.name = 'NMEA Format Error'
            ds.message = 'Error within the format of the current IIXDR translation'
            ds.values.append(KeyValue('Error non-matching format', ','.join(Array)))
            ds.level = DiagnosticStatus.ERROR
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$IIHDG':
        try:
            ds.name = 'Vessel Heading'
            ds.values.append(KeyValue('Magetic Sensor Heading', Array[1]))
            ds.values.append(KeyValue('Magetic Deviation', Array[2]))
            ds.values.append(KeyValue('Deviation E/W', Array[3]))
        except:
            ds.name = 'NMEA Format Error'
            ds.message = 'Error within the format of the current IIHDG translation'
            ds.values.append(KeyValue('Error non-matching format', ','.join(Array)))
            ds.level = DiagnosticStatus.ERROR
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$IIROT':
        try:
            ds.name = 'Rate of Turn'
            ds.values.append(KeyValue('Rate of Turn', Array[1]))
        except:
            ds.name = 'NMEA Format Error'
            ds.message = 'Error within the format of the current IIROT translation'
            ds.values.append(KeyValue('Error non-matching format', ','.join(Array)))
            ds.level = DiagnosticStatus.ERROR
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$IIVHW':
        try:
            ds.name = 'Speed'
            ds.values.append(KeyValue('Heading Degrees True',Array[1]))
            ds.values.append(KeyValue('Heading Degrees Mag.',Array[3]))
            ds.values.append(KeyValue('Water Speed (Knots)',Array[5]))
            ds.values.append(KeyValue('Water Speed (Kph)',Array[7]))
        except:
            ds.name = 'NMEA Format Error'
            ds.message = 'Error within the format of the current IIVHW translation'
            ds.values.append(KeyValue('Error non-matching format', ','.join(Array)))
            ds.level = DiagnosticStatus.ERROR
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$PMAREPD':
        try:
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
        except:
            ds.name = 'NMEA Format Error'
            ds.message = 'Error within the format of the current PMAREPD translation'
            ds.values.append(KeyValue('Error non-matching format', ','.join(Array)))
            ds.level = DiagnosticStatus.ERROR
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)

    elif Array[0] == '$PMAROUT':
        try:
            ds.name = 'Attitude'
            ds.values.append(KeyValue('Roll',Array[2]))
            ds.values.append(KeyValue('Pitch',Array[30]))
            ds.values.append(KeyValue('Yaw',Array[4]))
        except:
            ds.name = 'NMEA Format Error'
            ds.message = 'Error within the format of the current PMAROUT translation'
            ds.values.append(KeyValue('Error non-matching format', ','.join(Array)))
            ds.level = DiagnosticStatus.ERROR
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
