#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import proto
from proto import SerialStream,Messenger,Message
from rospy import Publisher,Service
from time import sleep,time
from gs_interfaces.srv import Live
from gs_interfaces.srv import Log,LogResponse
from gs_interfaces.srv import Led,LedResponse
from gs_interfaces.srv import Event,EventResponse
from gs_interfaces.srv import Time,TimeResponse
from gs_interfaces.srv import Info,InfoResponse
from gs_interfaces.srv import NavigationSystem,NavigationSystemResponse
from gs_interfaces.srv import Position,PositionResponse
from gs_interfaces.srv import PositionGPS,PositionGPSResponse
from gs_interfaces.srv import Yaw, YawResponse
from gs_interfaces.srv import ParametersList, ParametersListResponse
from gs_interfaces.msg import SimpleBatteryState,PointGPS,OptVelocity,Orientation,SatellitesGPS, Parameter
from std_msgs.msg import String,Float32,ColorRGBA,Int32,Int8
from geometry_msgs.msg import Point

rospy.init_node("ros_plaz_node")
stream = SerialStream(rospy.get_param(rospy.search_param("port")),57600)

messenger = Messenger(stream)

log = []
live = False
event_messages = (10, 12, 23, 2)
callback_event_messages = (255, 26, 31, 32, 42, 43, 51, 56, 65)
state_event = -1
state_callback_event = 0
state_position = [0., 0., 0., 0.]
state_gps_position = [0., 0., 0.]
state_board_led=[]
state_module_led=[]
navSystem = 0
navSystemName = {0:"GPS", 1:"LPS", 2:"OPT"}
global_point_seq = 0
autopilot_params = []

def handle_event(req):
    global messenger
    global state_event
    global event_messages
    try:
        if state_event != req.event:
            send_log("send: Event - {}".format(req.event))
            messenger.hub['UavMonitor']['mode'].write(event_messages[req.event])
            state_event = req.event
    except:
        return EventResponse(0)
    return EventResponse(1)

def handle_local_pos(req):
    global state_position
    global messenger
    request_position = [req.position.x,req.position.y,req.position.z]
    try:
        if request_position != state_position:
            send_log("send: go to local point - x: {}, y: {}, z: {}, time: {}".format(request_position[0],request_position[1],request_position[2],req.time))
            fields = {}
            fields['id'] = Message.GOTO_LOCAL_POINT
            fields['x'] = int(request_position[0] * 1e3 )
            fields['y'] = int(request_position[1] * 1e3 )
            fields['z'] = int(request_position[2] * 1e3 )
            fields['time'] = int(req.time)
            messenger.invokeAsync(packet=fields)
            state_position = request_position
    except:
        return PositionResponse(False)
    return PositionResponse(True)

def handle_gps_pos(req):
    global state_gps_position
    global global_point_seq
    global messenger
    request_position = [req.position.latitude,req.position.longitude,req.position.altitude]
    try:
        if request_position != state_gps_position:
            callback = lambda packet: proto.listen(Message.GO_TO_POINT_RESPONSE, packet)
            send_log("send: go to global position - latitude: {} , longitude: {} altitude: {}".format(request_position[0],request_position[1],request_position[2]))
            fields = {}
            fields['id'] = Message.GO_TO_POINT_V2
            fields['sequence'] = global_point_seq
            fields['latitude'] = int(request_position[0] * 1e7)
            fields['longitude'] = int(request_position[1] * 1e7)
            fields['altitude'] = int(request_position[2] * 1e2)
            fields['yaw'] = 0
            fields['flags'] = 0
            fields['type'] = 3
            fields['duration'] = 0
            fields['radius'] = 0
            fields['onStartedCommand'] = 0xFFFF
            fields['onReachedCommand'] = 0xFFFF

            global_point_seq += 1
            if global_point_seq >= 256:
                global_point_seq = 0
            response = messenger.invoke(packet = fields, callback = callback)
            if response is not None:
                state_gps_position = request_position
            else:
                return PositionGPSResponse(False)
    except:
        return PositionGPSResponse(False)
    return PositionGPSResponse(True)

def handle_yaw(req):
    global messenger
    try:
        send_log("send: update yaw - angle:{}".format(req.angle))
        messenger.hub['ManualControl']['yawManual'].write(int(req.angle * 100.0))
    except:
        return YawResponse(False)
    return YawResponse(True)

def handle_board_led(req): # Module_lua = 0
    global messenger
    global state_board_led
    try:
        if req.leds != state_board_led:
            for i in range(0, len(req.leds)):
                if req.leds[i] != state_board_led[i]:
                    send_log("send: change board leds color - n:{}, r: {}, g: {}, b: {}".format(i, req.leds[i].r, req.leds[i].g, req.leds[i].b))
                    messenger.hub['LedBar']['color'].write( int(req.leds[i].r) | (int(req.leds[i].g) << 8) | (int(req.leds[i].b) << 16) | (i << 24) )
                    state_board_led[i] = req.leds[i]
    except:
        return LedResponse(False)
    return LedResponse(True)

def handle_module_led(req):
    global messenger
    global state_module_led
    try:
        if req.leds != state_module_led:
            for i in range(4, len(req.leds)+4):
                if req.leds[i] != state_module_led[i]:
                    send_log("send: change module leds color - n:{}, r: {}, g: {}, b: {}".format(i, req.leds[i].r, req.leds[i].g, req.leds[i].b))
                    messenger.hub['LedBar']['color'].write( int(req.leds[i].r) | (int(req.leds[i].g) << 8) | (int(req.leds[i].b) << 16) | (i << 24) )
                    state_module_led[i] = req.leds[i]
    except:
        return LedResponse(False)
    return LedResponse(True)

def handle_live(req):
    global live
    return live

def send_log(msg):
    global log
    global logger_publisher
    msg = "[{}] {}".format(time(),msg)
    log.append(msg)
    logger_publisher.publish(msg)

def handle_log(req):
    global log
    return LogResponse(log)

def handle_time(req):
    global messenger
    try:
        send_log("send: time request")
        board_time = float(messenger.hub['UavMonitor']['time'].read()[0]) / 1000000.0 + 315964800.0
        send_log("response: time - {}".format(board_time))
        return TimeResponse(board_time)
    except:
        pass
    return TimeResponse(0.)

def handle_uptime(req):
    global messenger
    try:
        send_log("send: uptime request")
        uptime = messenger.hub['UavMonitor']['uptime'].read()[0]
        send_log("response: uptime - {}".format(uptime))
        return TimeResponse(uptime)
    except:
        pass
    return TimeResponse(0.)

def handle_flight_time(req):
    global messenger
    try:
        send_log("send: flight time request")
        flight_time = messenger.hub['UavMonitor']['missionTime'].read()[0]
        send_log("response: flight time - {}".format(flight_time))
        return TimeResponse(flight_time)
    except:
        pass
    return TimeResponse(0.)      

def handle_info(req):
    global messenger
    try:
        send_log("send: board number request")
        num = str(messenger.hub['UavMonitor']['number'].read()[0])
        send_log("response: board number - {}".format(num))
        return InfoResponse(num)
    except:
        pass
    return InfoResponse("error")

def handle_navSys(req):
    global navSystem
    global navSystemName
    return NavigationSystemResponse(navSystemName[navSystem])

def handle_autopilot_params(req):
    global autopilot_params
    return ParametersListResponse(autopilot_params)

def on_fields_changed(device, fields):
    global messenger
    global state_callback_event
    global callback_event_messages
    global callback_event_publisher
    if messenger.hub[device].name == 'FlightManager':
        event = messenger.hub['FlightManager']['event'].value
        messenger.hub['FlightManager']['event'].write(value=event,callback=None, blocking=False)
        if event != state_callback_event:
            callback_event_publisher.publish(callback_event_messages.index(event))
            state_callback_event = event
    elif messenger.hub[device].name == 'UavMonitor':
        if messenger.hub['UavMonitor']['mode'].value == 2:
            messenger.hub['FlightManager']['event'].write(value=255,callback=None, blocking=False)

logger = Service("geoscan/get_log",Log,handle_log)
alive = Service("geoscan/alive",Live,handle_live)

info_service = Service("geoscan/board/get_info",Info,handle_info)
time_service = Service("geoscan/board/get_time",Time,handle_time)
uptime_service = Service("geoscan/board/get_uptime",Time,handle_uptime)
flight_time_service = Service("geoscan/board/get_flight_time",Time,handle_flight_time)
autopilot_params_service = Service("geoscan/board/get_parameters",ParametersList,handle_autopilot_params)

navigation_service = Service("geoscan/navigation/get_system",NavigationSystem,handle_navSys)

local_position_service = Service("geoscan/flight/set_local_position",Position,handle_local_pos)
global_position_service = Service("geoscan/flight/set_global_position",PositionGPS,handle_gps_pos)
yaw_service = Service("geoscan/flight/set_yaw",Yaw,handle_yaw)
event_service = Service("geoscan/flight/set_event",Event,handle_event)
callback_event_publisher = Publisher("geoscan/flight/callback_event",Int32,queue_size=10)

board_led_service = Service("geoscan/led/board/set",Led,handle_board_led)
module_led_service = Service("geoscan/led/module/set",Led,handle_board_led)

logger_publisher = Publisher("geoscan/log",String,queue_size=10)

battery_publisher = Publisher("geoscan/battery_state",SimpleBatteryState,queue_size=10)

local_position_publisher = Publisher("geoscan/navigation/local/position",Point,queue_size=10)
local_yaw_publisher = Publisher("geoscan/navigation/local/yaw",Float32,queue_size=10)
local_velocity_publisher = Publisher("geoscan/navigation/local/velocity",Point,queue_size=10)

global_position_publisher = Publisher("geoscan/navigation/global/position",PointGPS,queue_size=10)
global_status_publisher = Publisher("geoscan/navigation/global/status",Int8,queue_size=10)
satellites_publisher = Publisher("geoscan/navigation/satellites",SatellitesGPS,queue_size=10)

opt_velocity_publisher = Publisher("geoscan/navigation/opt/velocity",OptVelocity,queue_size=10)

gyro_publisher = Publisher("geoscan/sensors/gyro",Point,queue_size=10)
accel_publisher = Publisher("geoscan/sensors/accel",Point,queue_size=10)
orientation_publisher = Publisher("geoscan/sensors/orientation",Orientation,queue_size=10)
altitude_publisher = Publisher("geoscan/sensors/altitude",Float32,queue_size=10)
mag_publisher = Publisher("geoscan/sensors/mag",Point,queue_size=10)

rospy.loginfo("Wait start connection ...")
send_log("wait start connect")
try:
    messenger.connect()
except:
    rospy.logerr("Geoscan Pioneer baseboard is not connected to this serial port")
    messenger.stop()
    stream.socket.close()
    exit()
if messenger.hub.model == 12:
    while not rospy.is_shutdown():
        try:
            for i in range(0,messenger.hub.getParamCount()):
                parameter = Parameter()
                string, parameter.value = messenger.hub.getParam(i)
                parameter.name = String(string)
                if parameter.name.data == 'Flight_com_navSystem':
                    navSystem = int(parameter.value)
                autopilot_params.append(parameter)
            break
        except:
            pass

    for _ in range(0,4):
            state_board_led.append(ColorRGBA())

    for _ in range(0,25):
            state_module_led.append(ColorRGBA())
    
    messenger.hub.onFieldsChanged = on_fields_changed
    messenger.hub['FlightManager']['event'].write(255)
    messenger.hub['LedBar']['color'].write(0 | (0 << 8) | (0 << 16) | (255 << 24) )
    
    rospy.loginfo("Board start connect - done")
    send_log("start connect - done")
    live = True
    while not rospy.is_shutdown():
        try:
            send_log("send: Battery state request")
            battery_state = SimpleBatteryState()
            battery_state.header.stamp = rospy.Time.now()
            battery_state.charge = messenger.hub['CBoard']['VoltBatt'].read()[0] / 1000.0
            send_log("response: Battery state - {}".format(battery_state.charge))
            battery_publisher.publish(battery_state)
        except:
            pass

        try:
            send_log("send: Gyro request")
            gyro = Point()
            gyro.x = messenger.hub['SensorMonitor']['gyroX'].read()[0] / 1e3
            gyro.y = messenger.hub['SensorMonitor']['gyroY'].read()[0] / 1e3
            gyro.z = messenger.hub['SensorMonitor']['gyroZ'].read()[0] / 1e3
            send_log("response: Gyro - [{}, {}, {}]".format(gyro.x,gyro.y,gyro.z))
            gyro_publisher.publish(gyro)
        except:
            pass

        try:
            send_log("send: Accel request")
            accel = Point()
            accel.x = messenger.hub['SensorMonitor']['accelX'].read()[0] / 1e3
            accel.y = messenger.hub['SensorMonitor']['accelY'].read()[0] / 1e3
            accel.z = messenger.hub['SensorMonitor']['accelZ'].read()[0] / 1e3
            send_log("response: Accel - [{}, {}, {}]".format(accel.x,accel.y,accel.z))
            accel_publisher.publish(accel)
        except:
            pass

        try:
            send_log("send: Orientation request")
            orientation = Orientation()
            orientation.roll = messenger.hub['UavMonitor']['roll'].read()[0] / 1e2
            orientation.pitch = messenger.hub['UavMonitor']['pitch'].read()[0] / 1e2
            orientation.azimuth = messenger.hub['UavMonitor']['yaw'].read()[0] / 1e2
            send_log("response: Orientation - [{}, {}, {}]".format(orientation.roll,orientation.pitch,orientation.azimuth))
            orientation_publisher.publish(orientation)
        except:
            pass

        try:
            send_log("send: Altitude request")
            altitude = messenger.hub['UavMonitor']['altitude'].read()[0] / 1e3
            send_log("response: Altitude - {}".format(altitude))
            altitude_publisher.publish(altitude)
        except:
            pass

        if navSystem == 0:
            try:
                send_log("send: Global position request")
                global_point = PointGPS()
                global_point.latitude = messenger.hub['Ublox']['latitude'].read()[0] / 1e7
                global_point.longitude = messenger.hub['Ublox']['longitude'].read()[0] / 1e7
                global_point.altitude = messenger.hub['Ublox']['altitude'].read()[0] / 1e3
                send_log("response: Global position - [{}, {}, {}]".format(global_point.latitude,global_point.longitude,global_point.altitude))
                global_position_publisher.publish(global_point)

                send_log("send: Mag request")
                mag = Point()
                mag.x = messenger.hub['SensorMonitor']['magX'].read()[0] / 1e3
                mag.y = messenger.hub['SensorMonitor']['magY'].read()[0] / 1e3
                mag.z = messenger.hub['SensorMonitor']['magZ'].read()[0] / 1e3
                send_log("response: Mag - [{}, {}, {}]".format(mag.x,mag.y,mag.z))
                mag_publisher.publish(mag)

                sat = SatellitesGPS()
                try:
                    sat.gps = messenger.hub['Ublox']['satGps'].read()[0]
                except:
                    sat.gps = 0

                try:
                    sat.glonass = messenger.hub['Ublox']['satGlonass'].read()[0]
                except:
                    sat.glonass = 0
                satellites_publisher.publish(sat)

                send_log("send: Global status")
                status = messenger.hub['Ublox']['status'].read()[0]
                send_log("response: Global status - {}".format(status))
                global_status_publisher.publish(status)

            except:
                rospy.logerr("Gnns Module not found")
                send_log("error: Gnns Module not found")
                break
        elif navSystem == 1:
            try:
                send_log("send: Local position request")
                local_point = Point()
                local_point.x = messenger.hub['USNav_module']['x'].read()[0] * 0.001
                local_point.y = messenger.hub['USNav_module']['y'].read()[0] * 0.001
                local_point.z = messenger.hub['USNav_module']['z'].read()[0] * 0.001
                send_log("response: Local position - [{}, {}, {}]".format(local_point.x,local_point.y,local_point.z))
                local_position_publisher.publish(local_point)

                send_log("send: LPS yaw request")
                yaw=messenger.hub['USNav_module']['yaw'].read()[0]
                send_log("response: LPS yaw - {}".format(yaw))
                local_yaw_publisher.publish(yaw)

                send_log("send: LPS velocity request")
                lps_vel = Point()
                lps_vel.x = messenger.hub['USNav_module']['velX'].read()[0]
                lps_vel.y = messenger.hub['USNav_module']['velY'].read()[0]
                lps_vel.z = messenger.hub['USNav_module']['velZ'].read()[0]
                send_log("response: LPS velocity - [{}, {}, {}]".format(lps_vel.x,lps_vel.y,lps_vel.z))
                local_velocity_publisher.publish(lps_vel)
            except:
                rospy.logerr("LPS not found")
                send_log("error: LPS not found")
                break
        elif navSystem == 2:
            try:
                send_log("send: OpticalFlow velocity request")
                velocity = OptVelocity()
                velocity.x = messenger.hub['SensorMonitor']['optFlowX'].read()[0]
                velocity.y = messenger.hub['SensorMonitor']['optFlowY'].read()[0]
                velocity.range = messenger.hub['SensorMonitor']['optFlowRange'].read()[0] / 1e3
                send_log("response: OpticalFlow velocity - [{}, {}, {}]".format(velocity.x,velocity.y,velocity.range))
                opt_velocity_publisher.publish(velocity)
            except:
                rospy.logerr("OpticalFlow Module not found")
                send_log("error: OpticalFlow Module not found")
                break
        sleep(0.01)
messenger.stop()
stream.socket.close()