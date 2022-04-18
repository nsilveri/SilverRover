#!/usr/bin/python
import paho.mqtt.client as mqtt
from paho.mqtt import client as mqtt_client
import time
import threading
import multiprocessing
import sys
from threading import Thread
import json
import collections
import serial
import os
import subprocess
import pprint
from typing import Any, Callable
from huawei_lte_api.Client import Client
from huawei_lte_api.Connection import Connection
from huaweisms.api import monitoring, sms, user
from huaweisms.api.common import ApiCtx
import git
from datetime import datetime

client_telemetry_radar_topic = "rover_telemetry_radar"
client_telemetry_energy_power_topic = "rover_telemetry_energy_power"
client_telemetry_wifi_status_topic = "rover_telemetry_wifi_status"
client_telemetry_motors_status_topic = "rover_telemetry_motors_status"
client_sys_message_topic = "rover_sys_message"

now = datetime.now()
#g = git.cmd.Git(git_dir)
#g.pull()

#radar_45_l = 5
#radar_front = 5
#radar_45_r = 5
#radar_left = 5
#radar_right = 5
#steer_value = 5
#battery_voltage = 5
#battery_percentage = 5
#solar_panel_voltage  = 5
#rover_voltage = 5
#solar_current = 5
#rover_current = 5
#run_value = 5
#panels_on_off = 0
#cam_sx_dx = 5
#cam_up_dn = 5
#rover_celsius = 5

###### 4G LTE CONNECTION ########################################
#################################################################
def dump(method: Callable[[], Any]) -> None:
    print("==== %s" % method.__qualname__)
    try:
        pprint.pprint(method())
    except Exception as e:
        print(str(e))
    print("")

def beautifyjson(jsonobj):
    text = json.dumps(jsonobj, sort_keys=True, indent=4)
    return text

isp = ""
cell_signal = ""

def cellular_connection():
    print("Cellular func")
    with Connection('http://admin:06051992@192.168.8.1/') as connection:
        #print("Cellular connected")
        client = Client(connection)
        carrier = beautifyjson(client.net.current_plmn())
        cellular_signal_level = beautifyjson(client.device.signal())
        cellular_signal_dict = json.loads(cellular_signal_level)
        carrier_dict = json.loads(carrier)
        isp = carrier_dict["FullName"]
        cell_signal = cellular_signal_dict["rsrp"]

###########################################################

###### ROVER_GETWAY <---------> RASPBERRY MIDDLEWARE ######
try:                                                      #
   # Change the baud rate here if diffrent than 19200     #
   rover = serial.Serial('/dev/ttyS0', 115200)            #
except IOError:                                           #
   print("Comm port not found")                           #
   sys.exit(0)                                            #
###########################################################
broker_address = "127.0.0.1"
port=1883
client_telemetry = mqtt.Client('rover_telemetry_client')
client_telemetry.connect(broker_address, port)
 #OUTPUT
###########################################################
client_command_topic = "rover_command"
###########################################################
client_id_command = 'rover_command_id'

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id_command)
    client.on_connect = on_connect
    client.connect(broker_address, port)
    return client

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        #print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        print(msg.payload.decode())
        data = json.loads(msg.payload.decode())
        #print(type(data))
        #if(type(data) == 'dict')) + '\n')
        if isinstance(data, dict):
            on_message_command(data)

    client.subscribe([("rover_command",0), ("ALERT",0)])
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()
#####################################################################################
def ustreamer_thread():
    os.system('sudo .//home/dietpi/ustreamer/ustreamer --host :: -m mjpeg  --device=/dev/video0 --buffers=1 --desired-fps=29 --quality 10  --tcp-nodelay  -r 1280x720 --rot 180  --port=8082')

#####################################################################################
rover_message_aux = ''
message_sent = False

def on_message_command(data):
    global rover_message_aux
    if data['action'] == "closed_connection":
        rover_command_string = str('KB ' + str(ord('C')) + '\n')
        print(rover_command_string)
        rover.write(rover_command_string.encode('ascii'))
    if data['action'] == "charge_battery_mode":
        #print("SOLAR")
        #rover.write(b'p')) + '\n')
        message_sent = False
        rover_command_string = str('KB ' + str(ord('r')) + '\n')
        print(rover_command_string)
        rover.write(rover_command_string.encode('ascii'))
    if data['action'] == "solar":
        #print("SOLAR")
        #rover.write(b'p')) + '\n')
        message_sent = False
        rover_command_string = str('KB ' + str(ord('p')) + '\n')
        print(rover_command_string)
        rover.write(rover_command_string.encode('ascii'))
    if data['action'] == "radar_onoff":
        #print("radar_onoff")
        #rover.write(b'5')) + '\n')
        message_sent = False
        rover_command_string = str('KB ' + str(ord('5')) + '\n')
        #rover_command_string = str('JS 330 330 250 150\n')
        rover.write(bytes(rover_command_string.encode('ascii')))
    if data['action'] == "fws":
        #print("4 wheel steering")
        #rover.write(b'4')) + '\n')
        message_sent = False
        rover_command_string = str('KB ' + str(ord('4')) + '\n')
        rover.write(bytes(rover_command_string.encode('ascii')))
    if data['action'] == "cam_steer":
        #print("4 wheel steering") reset_telemetry
        #rover.write(b'1')) + '\n')      
        message_sent = False
        rover_command_string = str('KB ' + str(ord('1')) + '\n')
        rover.write(bytes(rover_command_string.encode('ascii')))
    if data['action'] == "restart_telemetry":
        message_sent = False
        os.system('sudo systemctl restart mqtt_service')
        #reset_telemetry()
    if data['action'] == "reboot_system":
        message_sent = False
        os.system('sudo shutdown -r now')
        #reset_telemetry()
    if data['action'] == "stop":
        message_sent = False
        print("STOP")
    if data['action'] == "JS":
        R_stick_x = data['R_stick_x']
        R_stick_y = data['R_stick_y']
        L_stick_x = data['L_stick_x']
        L_stick_y = data['L_stick_y']
        #if(R_stick_x < 0.00001 or R_stick_x > -0.00001):
        #    R_stick_x = 0.00000000000
        Acc_z = data['Acc_z']
        rover_command_string = str('JS ' + str(_map(L_stick_x, -1, 1, 220, 440)) + ' ' + str(_map(R_stick_x, 1, -1, 85, 520)) + ' ' + str( _map(R_stick_y, 1, -1, 235, 465)) + ' ' + str(_map(Acc_z, -1, 1, 0, 100)) + '\n')#  _map(Acc_z, -1, 1, 220, 440)
        rover.write(bytes(rover_command_string.encode('ascii')))
    if data['action'] == "move":
        message_sent = False
        if data['dir'] == 7:
            #print("LEFT")
            #rover.write(b'q')) + '\n')
            #rover.write(b'q')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('q')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 8:
            print("FORWARD")
            #rover.write(b'w')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('w')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 9:
            #print("RIGHT")
            #rover.write(b'e')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('e')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 4:
            #rover.write(b'a')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('a')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 6:
            #rover.write(b'd')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('d')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 1:
            print("LEFT BACKWARD TURN")
        if data['dir'] == 2:
            #rover.write(b's')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('s')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 12:
            #rover.write(b'c')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('c')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 13:
            #rover.write(b'v')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('v')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 14:
            #rover.write(b'y')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('y')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 15:
            #rover.write(b'g')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('g')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 16:
            #rover.write(b'h')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('h')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 17:
            #rover.write(b'z')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('z')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 18:
            #rover.write(b'x')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('x')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 19:
            #rover.write(b'x')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('0')) + '\n')
            #rover.write(bytes(rover_command_string.encode('ascii')))
#####################################
    if data["action"] == "cam":
        if data['dir'] == 5:
            message_sent = False
            rover_command_string = str('KB ' + str(ord('2')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 7:
            #rover.write(b'j')) + '\n')
            #rover.write(b'k')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('j')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
            rover_command_string = str('KB ' + str(ord('k')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 8:
            #rover.write(b'i')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('i')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 9:
            #rover.write(b'l')) + '\n')
            #rover.write(b'k')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('l')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
            rover_command_string = str('KB ' + str(ord('k')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 4:
            #rover.write(b'j')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('j')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 6:
            #rover.write(b'l')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('l')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 1:
            #rover.write(b'i')) + '\n')
            #rover.write(b'j')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('i')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
            rover_command_string = str('KB ' + str(ord('j')) + '\n')
            rover.write(rover_command_string.encode('utf-8'))
        if data['dir'] == 2:
            #rover.write(b'k')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('k')) + '\n')
            rover.write(rover_command_string.encode('utf-8'))
        if data['dir'] == 3:
            #rover.write(b'i')) + '\n')
            #rover.write(b'l')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('i')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
            rover_command_string = str('KB ' + str(ord('l')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 10:
            #rover.write(b'u')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('u')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
        if data['dir'] == 11:
            #rover.write(b'o')) + '\n')
            message_sent = False
            rover_command_string = str('KB ' + str(ord('o')) + '\n')
            rover.write(bytes(rover_command_string.encode('ascii')))
    #steer_x = 0
    #cam_x = 0
    #cam_y = 0
    #acc_z = 0
    #rover_message = ('' + str(steer_x) + 'A' + str(cam_x) + 'B' + str(cam_y) + 'C' + str(acc_z) + 'D' + str(rover_command_string) + 'E')) + '\n') # + '\n'
    print(rover_command_string)
    #if(message_sent == False): #rover_message != rover_message_aux and 
    #    rover_message_aux = rover_message
    #    rover.write(('' + str(steer_x) + 'A' + str(cam_x) + 'B' + str(cam_y) + 'C' + str(acc_z) + 'D' + str(rover_command_string) + 'E')) + '\n').encode('ascii'))
    #    message_sent = True
    #rover.write(b'' + str(steer_x) + 'A' + str(cam_x) + 'B' + str(cam_y) + 'C' + str(acc_z) + 'D' + str(rover_command_string) + '\n')) + '\n')
####################################################################
#class controller():
#broker = '127.0.0.1'
#port = 1883

####################################################################
####################################################################
def on_message_telemetry(client, userdata, message):
    print("message received telemetry" ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)
####################################################################

############## MAP FUNCTION ########################################
def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
####################################################################
def getSSID():
    try:
        return str(subprocess.check_output(["/sbin/iwgetid -r"], shell = True).rstrip())
    except subprocess.CalledProcessError:
        print('error getting SSID')

def getSignalStrength():
    # signal strength
    #while True:
        #try:
            cmd = subprocess.Popen('iwconfig wlan0', shell=True,
                                stdout=subprocess.PIPE)
            #print()
            strength = str(cmd.communicate()[0])
            #print(strength)
            strength = strength.split(" Signal level=")
            if len(strength) > 1: 
                strength = strength[1].split(" dBm")
                return strength[0]
            else:
                return False
        #except:
           # continue

def getLinkQual():
    # Link Qual
    #while True:
        #try:
            cmd = subprocess.Popen('iwconfig wlan0', shell=True,
                                stdout=subprocess.PIPE)

            strength = str(cmd.communicate()[0])
            strength = strength.split(" Link Quality=")
            if len(strength) > 1:
                strength = strength[1].split(" Signal level=")
                return strength[0]
            else:
                return False
        #except:
            #continue

def telemetry_radar(string_message_radar):
    client_telemetry_radar = mqtt.Client('rover_telemetry_radar')
    client_telemetry_radar.connect(broker_address, port)

    while True:
        try:
            string_message_radar =       str('' + str(radar_front) + ',' +     str(radar_45_l)          + ',' + str(radar_45_r)    + ',' + str(radar_left)    + ',' + str(radar_right) + '')
            client_telemetry_radar.publish(client_telemetry_radar_topic, string_message_radar)
        except:
            continue

def telemetry_battery(string_message_battery):
    client_telemetry_battery = mqtt.Client('rover_telemetry_battery')
    client_telemetry_battery.connect(broker_address, port)

    while True:
        try:
            string_message_battery =     str('' + str(battery_voltage) + ',' + str(solar_panel_voltage) + ',' + str(solar_current) + ',' + str(rover_voltage) + ',' + str(rover_current) + ',' + str(battery_percentage) + '')
            client_telemetry_battery.publish(client_telemetry_energy_power_topic, string_message_battery)
        except:
            continue

def telemetry_motors(string_message_motors):
    client_telemetry_motors = mqtt.Client('rover_telemetry_motors')
    client_telemetry_motors.connect(broker_address, port)

    while True:
        try:
            string_message_motors =      str('' + str(panels_on_off)   + ',' + str(cam_sx_dx)           + ',' + str(cam_up_dn)     + ',' + str(run_value) + '')
            client_telemetry_motors.publish(client_telemetry_motors_status_topic, string_message_motors)
        except:
            continue

def telemetry_wifi(string_message_wifi_status):
    client_telemetry_wifi = mqtt.Client('rover_telemetry_wifi')
    client_telemetry_wifi.connect(broker_address, port)
    wifi_signal_level = getSignalStrength()
    wifi_signal_level = str(wifi_signal_level)
    wifi_signal_level = wifi_signal_level.split()
    wifi_signal_level = wifi_signal_level[0]
    wifi_signal_level = wifi_signal_level#[2:5]
    #wifi_signal_level = wifi_info[0].split()
    wifi_quality = getLinkQual()
    while True:
        try:
            string_message_wifi_status = str('' + str(wifi_quality)    + ',' + str(wifi_signal_level) + '')
            sent = client_telemetry_wifi.publish(client_telemetry_wifi_status_topic, string_message_wifi_status)
        except:
            continue

def get_session():
    return user.quick_login('admin', '06051992')

def isfloat(num):
    try:
        float(num)
        return True
    except ValueError:
        return False

def string_validation(string_message):
    valid = 0
    for i in range(1, 6, 1):
        if(string_message[i].isnumeric()):
            #print(string_message[i])
            valid = 1
        else:
            #print(string_message[i])
            #print('Valid: ', valid)
            return False
    for i in range(6, 20, 1):
        if(isfloat(string_message[i])):
            #print(string_message[i])
            valid = 1
        else:
            #print(string_message[i])
            #print('Valid: ', valid)
            return False

   #print('Valid: ', valid)
    return True

def save_debug_string_error(string_error, start_mode):
    #print("String error: ", string_error)
    #print("String error aux", string_error_aux)        
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
    lines = [str(dt_string), str(string_error)]
    with open('/home/dietpi/debug_string_error.txt', 'a') as f:
        if(start_mode == 1):
            start_line = ['Start session: ', str(dt_string)]
            f.write('\n')
            f.writelines('====================================================================')
            f.write('\n')
            f.writelines(start_line)
            f.write('\n')
            f.writelines('====================================================================')
            f.write('\n')
        f.write('ERROR: ')
        f.write('\t'.join(lines))
        f.write('\n')
        

### ROVER_GETWAY <---------> RASPBERRY MIDDLEWARE SERIAL CONVERSION ######
def get_values_from_serial():
    client_telemetry_topic = "rover_telemetry"
    client_telemetry.subscribe(client_telemetry_topic)
    global radar_45_l
    global radar_front
    global radar_45_r
    global radar_left
    global radar_right
    global steer_value
    global battery_voltage
    global battery_percentage
    global battery_remaining_time
    global solar_panel_voltage
    global rover_voltage
    global solar_current
    global rover_current
    global run_value
    global panels_on_off
    global cam_sx_dx
    global cam_up_dn
    global rover_celsius
    global sys_msg_timer
    string_message_wifi_status = ""
    read_string_aux = ""
    sys_msg_timer = 0
    while True:
     try:
         read_serial_string = rover.readline()
         #print(read_serial_string)
         read_serial_string = read_serial_string.decode('utf-8').rstrip()
         read_string = read_serial_string.split(",")
         sys_msg_timer = sys_msg_timer + 1
         if(sys_msg_timer == 15):
             sys_msg_string = ''
             client_telemetry.publish(client_sys_message_topic, sys_msg_string)
             sys_msg_timer = 0
         #read_string = read_serial.split(",")
         #print(read_string[0])
         #print(read_string[20])
         if(len(read_string) == 3 and read_string[0] == 'SYS_MSG' and read_string[2] == 'ES'):
            sys_msg_string = read_string[1]
            client_telemetry.publish(client_sys_message_topic, sys_msg_string)
            sys_msg_timer = 0

         elif(len(read_string) == 21 and read_string[0] == 'telemetry' and read_string[20] == 'ET' and string_validation(read_string)):#read_string[1].isnumeric()):
            radar_front = round(float(read_string[1])/10)
            radar_45_l = round(float(read_string[2])/10)
            radar_45_r = round(float(read_string[3])/10)
            radar_left = round(float(read_string[4])/10)
            radar_right = round(float(read_string[5])/10)
            battery_voltage = float(read_string[6])
            battery_current = float(read_string[9])
            #battery_current = round(battery_current_raw / 185)
            battery_percentage = round(_map(battery_voltage, 3.15, 4.15, 0, 100))
            if(battery_current < 0):
                battery_remaining_time = (10000 - (battery_percentage * 100)) / battery_current
                #print(battery_remaining_time)
            elif(battery_current > 0):
                battery_remaining_time = (battery_percentage * 100) / battery_current * 0.7
            else:
                battery_remaining_time = 0
            #print(battery_remaining_time)
            battery_remaining_time_HH = int(battery_remaining_time)
            battery_remaining_time_MM = (battery_remaining_time*60) % 60

            solar_panel_voltage = float(read_string[7])
            solar_current = float(read_string[10])
            solar_watt = solar_panel_voltage * solar_current
            rover_voltage = float(read_string[8])
            rover_current = float(read_string[11])
            rover_watt = rover_voltage * rover_current
            battery_watt = 0#battery_voltage * battery_current #solar_watt - rover_watt
            steer_value = float(read_string[12])
            run_value = float(read_string[13])
            panels_on_off = round(float(read_string[14]))
            cam_sx_dx = round(float(read_string[15]))
            cam_up_dn = round(float(read_string[16]))
            rover_celsius = float(read_string[17])
            motor_speed = read_string[19]
            if(getSignalStrength()):
                wifi_signal_level = getSignalStrength()
                wifi_signal_level = str(wifi_signal_level)
                wifi_signal_level = wifi_signal_level.split()
                wifi_signal_level = wifi_signal_level[0]
                wifi_signal_level = wifi_signal_level#[2:5]
            else:
                wifi_signal_level = -1

            if(getLinkQual()):
                wifi_quality = getLinkQual()
            else:
                wifi_quality = -1
            
            wifi_SSID = getSSID()
            #print(wifi_SSID)
            #wifi_BitRate = 
            #if(wifi_SSID == "b'HUAWEI_WINGLE_LTE'"):
            #    cellular_connection()
            #    string_message_wifi_status = str('' + str(wifi_quality)    + ',' + str(cell_signal)   + ',' + str(isp))


            #			                    0                           1                                2                           3                          4                           5
            string_message_wifi_status_aux = ''
            string_message_radar_aux = ''
            string_message_battery_aux = ''
            string_message_motors_aux = ''
            string_message_radar =       str('' + str(radar_front)     + ',' + str(radar_45_l)          + ',' + str(radar_45_r)    + ',' + str(radar_left)    + ',' + str(radar_right)   + ' ')
            #string_message_battery =     str('' + str(round(battery_voltage, 2)) + ',' + str(round(solar_panel_voltage, 2)) + ',' + str(solar_current) + ',' + str(round(rover_voltage, 2)) + ',' + str(rover_current) + ',' + str(battery_percentage) + ',' + str(round(solar_watt/1000, 2)) + ',' + str(round(rover_watt/1000, 2)) + ',' + str(round(battery_watt/1000, 2)) + ',' + str(round(battery_voltage, 2)) + ',' + str(round((battery_current / 185)*1000)) + '')
            string_message_battery =     str('' + str(round(battery_voltage, 2)) + ',' + str(round(solar_panel_voltage, 2)) + ',' + str(solar_current) + ',' + str(round(rover_voltage, 2)) + ',' + str(rover_current) + ',' + str(battery_percentage) + ',' + str(round(solar_watt/1000, 2)) + ',' + str(round(rover_watt/1000, 2)) + ',' + str(round(battery_watt)) + ',' + str(round(battery_voltage, 2)) + ',' + str(battery_current) + ',' + str(battery_remaining_time) + ',' + str(battery_remaining_time_HH) + ',' + str(battery_remaining_time_MM) + '')
            string_message_motors =      str('' + str(panels_on_off)   + ',' + str(cam_sx_dx)           + ',' + str(cam_up_dn)     + ',' + str(rover_celsius) + ',' + str(steer_value)   + ',' + str(motor_speed)        + '')
            if(wifi_SSID == "b'HUAWEI_WINGLE_LTE'"):
                #cellular_connection()
                with Connection('http://admin:06051992@192.168.8.1/') as connection:
                    #ctx = get_session()
                    #print("Cellular connected")
                    client = Client(connection)
                    carrier = beautifyjson(client.net.current_plmn())
                    cellular_signal_level = beautifyjson(client.device.signal())
                    #print(client.net.network())
                    #print(cellular_signal_level)
                    #print(carrier)
                    cellular_signal_dict = json.loads(cellular_signal_level)
                    carrier_dict = json.loads(carrier)
                    isp = carrier_dict["FullName"]
                    cell_signal = cellular_signal_dict["rsrp"]
                    cell_speed = cellular_signal_dict["earfcn"]
                    cell_id = cellular_signal_dict["cell_id"]
                    #print(isp)
                    #print(cell_signal)
                    #print(cell_speed)
                    #print(cell_id)
                    if(cell_signal == None):
                        #print(cell_signal)
                        cell_signal = '-1dBm'
                    if(isp == None):
                        isp = 'No SIM Card'
                    #print(cell_signal)
                    #print(isp)
                    string_message_wifi_status = str('' + str(cell_speed)    + ',' + str(_map(int(cell_signal.replace("dBm", "")), -120, -80, 0, 100))   + ',' + str(isp)  + ',' + '1'+ ',' + str(cell_signal)   + ',')
                    if(string_message_wifi_status != string_message_wifi_status_aux):
                        client_telemetry.publish(client_telemetry_wifi_status_topic, string_message_wifi_status)
                        string_message_wifi_status_aux = string_message_wifi_status
            else:
                string_message_wifi_status = str('' + str(wifi_quality)    + ',' + str(wifi_signal_level)   + ',' + str(wifi_SSID.replace("'", ""))  + ',' + '0' + '')
                if(string_message_wifi_status != string_message_wifi_status_aux):
                        client_telemetry.publish(client_telemetry_wifi_status_topic, string_message_wifi_status)
                        string_message_wifi_status_aux = string_message_wifi_status
            #print(string_message_battery)
            #print(string_message_wifi_status)
            if(string_message_radar != string_message_radar_aux):
                client_telemetry.publish(client_telemetry_radar_topic, string_message_radar)
                string_message_radar_aux = string_message_radar
            if(string_message_battery != string_message_battery_aux):
                client_telemetry.publish(client_telemetry_energy_power_topic, string_message_battery)
                string_message_battery_aux = string_message_battery
            if(string_message_motors_aux != string_message_motors):
                client_telemetry.publish(client_telemetry_motors_status_topic, string_message_motors)
                string_message_motors_aux = string_message_motors
            #client_telemetry.publish(client_telemetry_wifi_status_topic, string_message_wifi_status)
         else:
           #print('Invalid telemetry string')
           #print(read_string)
           #print(read_string_aux)
            if(read_string != read_string_aux and read_string_aux != ''):
                read_string_aux = read_string
                save_debug_string_error(read_string, 0)
            elif(read_string != read_string_aux and read_string_aux == ''):
                read_string_aux = read_string
                save_debug_string_error(read_string, 1)
     except ValueError:
         print("Could not convert data to an integer.")
         pass
     except json.decoder.JSONDecodeError:
         #print("ERROR JSON")
         pass
     except serial.serialutil.SerialException:
          #print("ERROR SERIAL")
         pass

if __name__ == '__main__':
    client_telemetry_process = multiprocessing.Process(name='Telemetry', target=get_values_from_serial)
    client_telemetry_process.daemon = False

    client_command_process = multiprocessing.Process(name='Commands', target=run)
    client_telemetry_process.daemon = False

    client_telemetry_process.start()

    client_command_process.start()
