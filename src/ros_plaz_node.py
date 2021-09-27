#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import proto
from proto import SerialStream, Messenger, Message
from rospy import Publisher, Service
from time import sleep, time
from gs_interfaces.srv import Live, LiveResponse
from gs_interfaces.srv import Log,LogResponse
from gs_interfaces.srv import Led,LedResponse
from gs_interfaces.srv import Event,EventResponse
from gs_interfaces.srv import Time,TimeResponse
from gs_interfaces.srv import Info,InfoResponse
from gs_interfaces.srv import NavigationSystem,NavigationSystemResponse
from gs_interfaces.srv import SetNavigationSystem,SetNavigationSystemResponse
from gs_interfaces.srv import Position,PositionResponse
from gs_interfaces.srv import PositionGPS,PositionGPSResponse
from gs_interfaces.srv import Yaw, YawResponse
from gs_interfaces.srv import ParametersList, ParametersListResponse
from gs_interfaces.srv import SetParametersList, SetParametersListResponse
from gs_interfaces.msg import SimpleBatteryState,PointGPS,OptVelocity,Orientation,SatellitesGPS, Parameter
from std_msgs.msg import String,Float32,ColorRGBA,Int32,Int8
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

TIME_FOR_RESTART = 5 # приблизительное время необходимое для перезапуска платы

class ROSPlazNode(): # класс ноды ros_plaz_node
    def __init__(self, uart="/dev/ttyS0", rate = None):
        self.navSystemParam = { # параметры необходимые для изменения систем позиционирования
            "GPS": [
                Parameter("BoardPioneer_modules_gnss", 1.0),
                Parameter("Imu_magEnabled", 1),
                Parameter("SensorMux_gnss", 255.0),
                Parameter("Copter_man_velScale", 5),
                Parameter("Copter_man_vzScale", 5),
                Parameter("Copter_pos_vDesc", 0.7),
                Parameter("Copter_pos_vDown", 1),
                Parameter("Copter_pos_vLanding", 0.5),
                Parameter("Copter_pos_vMax", 2),
                Parameter("Copter_pos_vTakeoff", 0.5),
                Parameter("Copter_pos_vUp", 1),
                Parameter("Flight_com_homeAlt", 1.5),
                Parameter("Flight_com_landingAlt", 1),
                Parameter("Flight_com_navSystem", 0),
                Parameter("State_startCount", 92)
            ],
            "LPS": [
                Parameter("BoardPioneer_modules_gnss", 0),
                Parameter("Imu_magEnabled", 0.0),
                Parameter("SensorMux_gnss", 0.0),
                Parameter("Copter_man_velScale", 0.5),
                Parameter("Copter_man_vzScale", 0.5),
                Parameter("Copter_pos_vDesc", 0.4),
                Parameter("Copter_pos_vDown", 0.7),
                Parameter("Copter_pos_vLanding", 0.3),
                Parameter("Copter_pos_vMax", 1),
                Parameter("Copter_pos_vTakeoff", 0.3),
                Parameter("Copter_pos_vUp", 0.5),
                Parameter("Flight_com_homeAlt", 0.5),
                Parameter("Flight_com_landingAlt", 0),
                Parameter("Flight_com_navSystem", 1),
                Parameter("State_startCount", 90)
            ],
            "OPT": [
                Parameter("BoardPioneer_modules_gnss", 0.0),
                Parameter("Imu_magEnabled", 0),
                Parameter("SensorMux_gnss", 0.0),
                Parameter("Copter_man_velScale", 0.5),
                Parameter("Copter_man_vzScale", 0.5),
                Parameter("Copter_pos_vDesc", 0.4),
                Parameter("Copter_pos_vDown", 0.5),
                Parameter("Copter_pos_vLanding", 0.3),
                Parameter("Copter_pos_vMax", 0.4),
                Parameter("Copter_pos_vTakeoff", 0.3),
                Parameter("Copter_pos_vUp", 0.5),
                Parameter("Flight_com_homeAlt", 0.5),
                Parameter("Flight_com_landingAlt", 0),
                Parameter("Flight_com_navSystem", 2),
                Parameter("State_startCount", 93)
            ]
        }

        self.uart = uart # название порта
        self.log = [] # массив, хранящий все сообщения лога
        self.live = False # состояние подключение к базовой платы АП
        self.restart = False # состояние перезапуска
        self.navSystem = 0 # текущая система позиционирования
        self.event_messages = (10, 12, 23, 2) # доступные события(команды) АП
        self.callback_event_messages = (255, 26, 31, 32, 42, 43, 51, 56, 65) # события, возвращаемые АП
        self.navSystemName = {0:"GPS", 1:"LPS", 2:"OPT"} # доступные системы позиционирования
        self.state_event = -1 # последнеее событие, отправленное в АП
        self.state_callback_event = 0 # полседнее событие пришедшее от АП
        self.state_position = [0., 0., 0., 0.] # последняя точка, на которую был отправлен коптер (в локальных координатах)
        self.state_gps_position = [0., 0., 0.] # последняя координата, нак оторую был отправлен коптер (в глобальных координатах)
        self.state_board_led=[] # текущее состояние светодиодов на базовой плате
        self.state_module_led=[] # текущее состояние светодиодов на Led-модуле  
        self.global_point_seq = 0 # номер точки в глобальной системе
        self.autopilot_params = [] # выгруженные параметры АП
        self.messenger = None # основной объект класса Messenger, отвечающий за коммуникацию между RPi и базовой платы
        self.rate = rate # таймер

        self.logger = Service("geoscan/get_log",Log, self.handle_log) # сервис логов
        self.alive = Service("geoscan/alive", Live, self.handle_live) # сервис, показывающий состояние подключения

        self.info_service = Service("geoscan/board/get_info", Info, self.handle_info) # сервис, возвращающий бортовую информация
        self.time_service = Service("geoscan/board/get_time", Time, self.handle_time) # сервис, возвращающий время с момента включения коптера
        self.uptime_service = Service("geoscan/board/get_uptime", Time, self.handle_uptime) # сервис, возвращающий время запуска для системы навигации
        self.flight_time_service = Service("geoscan/board/get_flight_time", Time, self.handle_flight_time)  # сервис, возвращающий время с начала полета
        self.get_autopilot_params_service = Service("geoscan/board/get_parameters", ParametersList, self.handle_get_autopilot_params) # сервис, возвращающий параметры АП
        self.set_autopilot_params_service = Service("geoscan/board/set_parameters", SetParametersList, self.handle_set_autopilot_params) # сервис, устанавливающий параметры АП
        self.restart_service = Service("geoscan/board/restart", Empty, self.handle_restart) # сервси перезапуска базововй платы
        
        self.get_navigation_service = Service("geoscan/navigation/get_system", NavigationSystem, self.handle_get_navigation_system) # сервис, возвращающий текущую систему позиционирования
        self.set_navigation_service = Service("geoscan/navigation/set_system", SetNavigationSystem, self.handle_set_navigation_system) # сервис, устанавливающий текущую систему позиционирования

        self.local_position_service = Service("geoscan/flight/set_local_position", Position, self.handle_local_pos) # сервис полета в локальную точку
        self.global_position_service = Service("geoscan/flight/set_global_position", PositionGPS, self.handle_gps_pos) # сервси полета в глобальную точку
        self.yaw_service = Service("geoscan/flight/set_yaw", Yaw, self.handle_yaw) # сервис управления рысканьем
        self.event_service = Service("geoscan/flight/set_event", Event, self.handle_event) # севрис управления событиями АП

        self.board_led_service = Service("geoscan/led/board/set", Led, self.handle_board_led) # сервис управления светодиодами на базовой плате
        self.module_led_service = Service("geoscan/led/module/set", Led, self.handle_board_led) # сервис управления светодиодами на LED-модуле

        self.logger_publisher = Publisher("geoscan/log", String, queue_size=10) # издатель темы логов

        self.battery_publisher = Publisher("geoscan/battery_state", SimpleBatteryState, queue_size=10) # издатель темы состояния АКБ

        self.local_position_publisher = Publisher("geoscan/navigation/local/position", Point, queue_size=10) # издатель темы позиции в LPS
        self.local_yaw_publisher = Publisher("geoscan/navigation/local/yaw", Float32, queue_size=10) # издаетель темы рысканья в LPS
        self.local_velocity_publisher = Publisher("geoscan/navigation/local/velocity", Point, queue_size=10) # издатель темы ускорения в LPS

        self.global_position_publisher = Publisher("geoscan/navigation/global/position", PointGPS, queue_size=10) # издатель темы позиции в GPS
        self.global_status_publisher = Publisher("geoscan/navigation/global/status", Int8, queue_size=10) # издатель темы статуса GPS модуля
        self.satellites_publisher = Publisher("geoscan/navigation/satellites", SatellitesGPS, queue_size=10) # издатель темы состояния спутников

        self.opt_velocity_publisher = Publisher("geoscan/navigation/opt/velocity", OptVelocity, queue_size=10) # издатель темы ускорения в OPT

        self.callback_event_publisher = Publisher("geoscan/flight/callback_event", Int32, queue_size=10) # издатель темы событий, возвращаемых АП

        self.gyro_publisher = Publisher("geoscan/sensors/gyro", Point, queue_size=10) # издатель темы данных гироскопа
        self.accel_publisher = Publisher("geoscan/sensors/accel", Point, queue_size=10) # издатель темы данных акселерометра
        self.orientation_publisher = Publisher("geoscan/sensors/orientation", Orientation, queue_size=10) # издатель темы данных о положении
        self.altitude_publisher = Publisher("geoscan/sensors/altitude", Float32, queue_size=10) # издатель темы данных о высоте по барометру
        self.mag_publisher = Publisher("geoscan/sensors/mag", Point, queue_size=10) # издатель темы данных магнитометра


    def __send_log(self, msg): # функция отправки сообщения в лог
        msg = f"[{time()}] {msg}"
        self.log.append(msg) # добавляем сообщение в масив логов
        self.logger_publisher.publish(msg) # публикуем сообщения лога в топик logger

    def disconnect(self): # функция разрыва коммуникации между RPi и базовой платой
        if self.messenger != None:
            self.messenger.stop() # останвливаем поток сообщений
            self.messenger.handler.stream.socket.close() # закрываем порт
            self.messenger = None # обнуляем messenger

    def __navSystem_except(self, name): # обработка исключения при ошибки связи с модулями навигации
        if not self.restart: # проверяем не идет ли перезагрузка платы
            self.live = False # устанавливаем состояние подключения
            rospy.logerr(f"{name} not found")
            self.__send_log(f"error: {name} not found")
            self.disconnect() # разрываем подключение, чтобы вызвать переподключение к плате

    def restart_board(self): # функция перезагрузки платы
        self.__send_log("restart board - start")
        self.restart = True # устанавливаем статус перезапуска
        self.live = False # устанавливаем состояние подключения
        rospy.loginfo("Restarting board ...")
        self.messenger.hub.sendCommand(18) # отправляем команду на перезапуск платы
        self.disconnect() # разрываем подключение
        sleep(TIME_FOR_RESTART) # ожидание перезагрузки
        self.__send_log("restart board - finish")
        rospy.loginfo("Restart board - done")
        self.restart = False # устанавливаем статус перезапуска

    def handle_restart(self, request): # функция обработки запроса на перезагрузку
        self.restart_board() # перезапускам плату
        return EmptyResponse() # возвращаем пустой ответ

    def handle_live(self, request): 
        return LiveResponse(self.live)

    def handle_event(self, request): # функция обработки запроса на отправление события в АП
        try:
            if self.state_event != request.event: # проверяем на совпадение предыдущее событие с событием из запроса
                self.__send_log(f"send: Event - {request.event}")
                self.messenger.hub['UavMonitor']['mode'].write(self.event_messages[request.event]) # записываем событие в АП
                self.state_event = request.event # присваиваем событие из запроса в предыдущее событие
        except:
            return EventResponse(0) # если произошла ошибка записи, то отправляем ответом код ошибки 0
        return EventResponse(1) # отправляем 1 - команда отправлена

    def handle_local_pos(self, request): # функция обработки запроса на полет в локальную точку
        request_position = [request.position.x, request.position.y, request.position.z] # запоминаем координаты точки из запроса
        try:
            if request_position != self.state_position: # сравниваем координаты точки с предыдущими координатами
                self.__send_log(f"send: go to local point - x: {request_position[0]}, y: {request_position[1]}, z: {request_position[2]}, time: {request.time}")
                fields = {} # объявляем тело запроса в АП
                fields['id'] = Message.GOTO_LOCAL_POINT # присваиваем тип сообщения
                fields['x'] = int(request_position[0] * 1e3 ) # присваиваем координату x с переводом из метров в милимметры
                fields['y'] = int(request_position[1] * 1e3 ) # присваиваем координату y с переводом из метров в милимметры
                fields['z'] = int(request_position[2] * 1e3 ) # присваиваем координату z с переводом из метров в милимметры
                fields['time'] = int(request.time) # присваиваем время перелета
                self.messenger.invokeAsync(packet=fields) # отправляем запрос в АП
                self.state_position = request_position # присваиваем предудщей координате запрошенную
        except:
            return PositionResponse(False) # если произошла ошибка отправки возвращаем код ошибки
        return PositionResponse(True) # возвращаем True - команда выполнена

    def handle_gps_pos(self, request): # функция обработки запроса на полет в точку в глобальных системах координат
        request_position = [request.position.latitude, request.position.longitude, request.position.altitude] # запоминаем координаты точки из запроса
        try:
            if request_position != self.state_gps_position: # сравниваем координаты точки с предыдущими координатами
                callback = lambda packet: proto.listen(Message.GO_TO_POINT_RESPONSE, packet) # объявляем функцию обработки ответа АП
                self.__send_log(f"send: go to global position - latitude: {request_position[0]} , longitude: {request_position[1]} altitude: {request_position[2]}")
                fields = {} # объявляем тело запроса в АП
                fields['id'] = Message.GO_TO_POINT_V2 # присваиваем тип сообщения
                fields['sequence'] = self.global_point_seq # устанвливаем номер точки
                fields['latitude'] = int(request_position[0] * 1e7) # присваиваем широту
                fields['longitude'] = int(request_position[1] * 1e7) # присваиваем долготу
                fields['altitude'] = int(request_position[2] * 1e2) # присваиваем высоту над уровнем моря
                fields['yaw'] = 0 # устанавливаем рысканье
                fields['flags'] = 0
                fields['type'] = 3
                fields['duration'] = 0
                fields['radius'] = 0
                fields['onStartedCommand'] = 0xFFFF
                fields['onReachedCommand'] = 0xFFFF

                self.global_point_seq += 1 # увеличивает номер точки на 1
                if self.global_point_seq >= 256: # чтобы не было переполнения памяти платы обнуляем номер точки, если он достиг 256
                    self.global_point_seq = 0
                response = self.messenger.invoke(packet = fields, callback = callback) # отправляем запрос в АП
                if response is not None: # если есть ответ отплаты, то присваиваем предыдущей координате запрошенную
                    self.state_gps_position = request_position
                else:
                    return PositionGPSResponse(False) # если не получен ответ возвращаем код ошибки
        except:
            return PositionGPSResponse(False) # если произошла ошибка отправки возвращаем код ошибки
        return PositionGPSResponse(True) # возвращаем True - команда выполнена

    def handle_yaw(self, request): # функция обработки запроса на изменение угла рысканья
        try:
            self.__send_log(f"send: update yaw - angle:{request.angle}")
            self.messenger.hub['ManualControl']['yawManual'].write(int(request.angle * 100.0)) # отправляем запрос на изменение угла рысканья
        except:
            return YawResponse(False)  # если произошла ошибка отправки возвращаем код ошибки
        return YawResponse(True) # возвращаем True - команда выполнена

    def handle_board_led(self, request): # функция обработки запроса на изменение цвета светодиодов на базовой плате
        try:
            if request.leds != self.state_board_led: # сравниваем запрос с текущим состоянием светодиодов
                for i in range(0, len(request.leds)): # делаем проход по всем светодиодам
                    if request.leds[i] != self.state_board_led[i]: # если цвет на текущем светодиоде не совпадает с запрошенным, тогда обновляем на нем цвет
                        self.__send_log(f"send: change board leds color - n:{i}, r: {request.leds[i].r}, g: {request.leds[i].g}, b: {request.leds[i].b}")
                        self.messenger.hub['LedBar']['color'].write( int(request.leds[i].r) | (int(request.leds[i].g) << 8) | (int(request.leds[i].b) << 16) | (i << 24) ) # обновление цвета светодиода
                        self.state_board_led[i] = request.leds[i] # запоминаем состояние светодиода
        except:
            return LedResponse(False)  # если произошла ошибка отправки возвращаем код ошибки
        return LedResponse(True) # возвращаем True - команда выполнена

    def handle_module_led(self, request): # функция обработки запроса на изменение цвета светодиодов на LED-модуле
        try:
            if request.leds != self.state_module_led: # сравниваем запрос с текущим состоянием светодиодов
                for i in range(4, len(request.leds) + 4): # делаем проход по всем светодиодам. Нумерация светодиодов LED-модуля начинается с 4 (поскольку всего 4 светодиода на базовой плате)
                    if request.leds[i] != self.state_module_led[i]: # если цвет на текущем светодиоде не совпадает с запрошенным, тогда обновляем на нем цвет
                        self.__send_log(f"send: change module leds color - n:{i}, r: {request.leds[i].r}, g: {request.leds[i].g}, b: {request.leds[i].b}")
                        self.messenger.hub['LedBar']['color'].write( int(request.leds[i].r) | (int(request.leds[i].g) << 8) | (int(request.leds[i].b) << 16) | (i << 24) ) # обновление цвета светодиода
                        self.state_module_led[i] = request.leds[i] # запоминаем состояние светодиода
        except:
            return LedResponse(False)  # если произошла ошибка отправки возвращаем код ошибки
        return LedResponse(True) # возвращаем True - команда выполнена

    def handle_log(self, request): # функция обработки запроса на получение логов
        return LogResponse(self.log) # возвращаем массив логов

    def handle_time(self, request): # функция обработки запроса на получение времени с момента включения коптера
        try:
            self.__send_log("send: time request")
            board_time = float(self.messenger.hub['UavMonitor']['time'].read()[0]) / 1000000.0 + 315964800.0 # получаем время и переводим его в секунды
            self.__send_log(f"response: time - {board_time}")
            return TimeResponse(board_time) # возвращаем время
        except:
            pass
        return TimeResponse(0.) # если не получен ответ возвращаем код ошибки

    def handle_uptime(self, request): # функция обработки запроса на получение времени запуска для системы навигации
        try:
            self.__send_log("send: uptime request")
            uptime = self.messenger.hub['UavMonitor']['uptime'].read()[0] # получаем время
            self.__send_log(f"response: uptime - {uptime}")
            return TimeResponse(uptime) # возвращаем время
        except:
            pass
        return TimeResponse(0.) # если не получен ответ возвращаем код ошибки

    def handle_flight_time(self, request): # функция обработки запроса на получение времени с начала полета
        try:
            self.__send_log("send: flight time request")
            flight_time = self.messenger.hub['UavMonitor']['missionTime'].read()[0] # получаем время
            self.__send_log(f"response: flight time - {flight_time}")
            return TimeResponse(flight_time) # возвращаем время
        except:
            pass
        return TimeResponse(0.) # если не получен ответ возвращаем код ошибки

    def handle_info(self, request): # функция обработки запроса на получение бортовой информации
        try:
            self.__send_log("send: board number request")
            num = str(self.messenger.hub['UavMonitor']['number'].read()[0]) # получаем информацию
            self.__send_log(f"response: board number - {num}")
            return InfoResponse(num) # возвращаем информацию
        except:
            pass
        return InfoResponse("error") # если не получен ответ возвращаем код ошибки

    def handle_get_navigation_system(self, request): # функция обработки запроса на получение текущей системы навигации
        return NavigationSystemResponse(self.navSystemName[self.navSystem]) # возвращаем имя системы позиционирования
    
    def handle_set_navigation_system(self, request):
        self.live = False
        for param in self.navSystemParam[request.navigation]:
            try:
                self.messenger.hub.setParam(name = param.name, value = param.value)
            except:
                pass
        rospy.loginfo("Changing the navigation system ...")
        self.disconnect()
        return SetNavigationSystemResponse(True) # возвращаем True - команда выполнена

    def handle_get_autopilot_params(self, request):
        return ParametersListResponse(self.autopilot_params)

    def handle_set_autopilot_params(self, request):
        try:
            for param in request.params:
                exist = False
                for ap_param in self.autopilot_params:
                    if ap_param.name == param.name:
                        if ap_param.value != param.value:
                            self.messenger.hub.setParam(name = param.name, value = param.value)
                        else:
                            rospy.logwarn(f"The value of the {param.name} parameter is equal to the desired value")
                        exist = True
                        break
                if not exist:
                    rospy.logwarn(f"{param.name} parameter does not exist")
        except:
            return SetParametersListResponse(False) # если не получен ответ возвращаем код ошибки
        self.__get_param_from_ap()
        return SetParametersListResponse(True) # возвращаем True - команда выполнена

    def __on_fields_changed(self, device, fields):
        if self.messenger.hub[device].name == 'FlightManager':
            event = self.messenger.hub['FlightManager']['event'].value
            self.messenger.hub['FlightManager']['event'].write(value = event, callback = None, blocking = False)
            if ((event != self.state_callback_event) and (event != 255)):
                self.callback_event_publisher.publish(self.callback_event_messages.index(event))
                self.state_callback_event = event
        elif self.messenger.hub[device].name == 'UavMonitor':
            if self.messenger.hub['UavMonitor']['mode'].value == 2:
                self.messenger.hub['FlightManager']['event'].write(value = 255, callback = None, blocking = False)
                if self.state_callback_event != 255:
                    self.state_callback_event = 255
                    self.callback_event_publisher.publish(self.callback_event_messages.index(self.state_callback_event))

    def __get_param_from_ap(self):
        try:
            self.autopilot_params = []
            for i in range(0, self.messenger.hub.getParamCount()):
                parameter = Parameter()
                parameter.name, parameter.value = self.messenger.hub.getParam(i)
                if parameter.name == 'Flight_com_navSystem':
                    self.navSystem = int(parameter.value)
                self.autopilot_params.append(parameter)
        except:
            self.disconnect()

    def connect(self):
        self.__send_log("try to connect")
        rospy.loginfo("Try to connect ...")
        self.messenger = Messenger(SerialStream(self.uart, 57600))
        self.messenger.connect()
        if ((self.messenger.hub.model == 12) and not self.live):
            self.__get_param_from_ap()

            for _ in range(0,4):
                self.state_board_led.append(ColorRGBA())

            for _ in range(0,25):
                self.state_module_led.append(ColorRGBA())

            self.messenger.hub['FlightManager']['event'].write(255)
            self.messenger.hub['LedBar']['color'].write(0 | (0 << 8) | (0 << 16) | (255 << 24) )
            self.messenger.hub.onFieldsChanged = self.__on_fields_changed

            self.state_event = -1

            rospy.loginfo("Board start connect - done")
            self.live = True

    def data_exchange(self):
        if self.messenger != None:
            if self.live:
                try:
                    self.__send_log("send: Battery state request")
                    battery_state = SimpleBatteryState()
                    battery_state.header.stamp = rospy.Time.now()
                    battery_state.charge = self.messenger.hub['CBoard']['VoltBatt'].read()[0] / 1000.0
                    self.__send_log(f"response: Battery state - {battery_state.charge}")
                    self.battery_publisher.publish(battery_state)
                except:
                    pass

                try:
                    self.__send_log("send: Gyro request")
                    gyro = Point()
                    gyro.x = self.messenger.hub['SensorMonitor']['gyroX'].read()[0] / 1e3
                    gyro.y = self.messenger.hub['SensorMonitor']['gyroY'].read()[0] / 1e3
                    gyro.z = self.messenger.hub['SensorMonitor']['gyroZ'].read()[0] / 1e3
                    self.__send_log(f"response: Gyro - [{gyro.x}, {gyro.y}, {gyro.z}]")
                    self.gyro_publisher.publish(gyro)
                except:
                    pass

                try:
                    self.__send_log("send: Accel request")
                    accel = Point()
                    accel.x = self.messenger.hub['SensorMonitor']['accelX'].read()[0] / 1e3
                    accel.y = self.messenger.hub['SensorMonitor']['accelY'].read()[0] / 1e3
                    accel.z = self.messenger.hub['SensorMonitor']['accelZ'].read()[0] / 1e3
                    self.__send_log(f"response: Accel - [{accel.x}, {accel.y}, {accel.z}]")
                    self.accel_publisher.publish(accel)
                except:
                    pass

                try:
                    self.__send_log("send: Orientation request")
                    orientation = Orientation()
                    orientation.roll = self.messenger.hub['UavMonitor']['roll'].read()[0] / 1e2
                    orientation.pitch = self.messenger.hub['UavMonitor']['pitch'].read()[0] / 1e2
                    orientation.azimuth = self.messenger.hub['UavMonitor']['yaw'].read()[0] / 1e2
                    self.__send_log(f"response: Orientation - [{orientation.roll}, {orientation.pitch}, {orientation.azimuth}]")
                    self.orientation_publisher.publish(orientation)
                except:
                    pass

                try:
                    self.__send_log("send: Altitude request")
                    altitude = self.messenger.hub['UavMonitor']['altitude'].read()[0] / 1e3
                    self.__send_log(f"response: Altitude - {altitude}")
                    self.altitude_publisher.publish(altitude)
                except:
                    pass

                if self.navSystem == 0:
                    try:
                        self.__send_log("send: Global position request")
                        global_point = PointGPS()
                        global_point.latitude = self.messenger.hub['Ublox']['latitude'].read()[0] / 1e7
                        global_point.longitude = self.messenger.hub['Ublox']['longitude'].read()[0] / 1e7
                        global_point.altitude = self.messenger.hub['Ublox']['altitude'].read()[0] / 1e3
                        self.__send_log(f"response: Global position - [{global_point.latitude}, {global_point.longitude}, {global_point.altitude}]")
                        self.global_position_publisher.publish(global_point)

                        self.__send_log("send: Mag request")
                        mag = Point()
                        mag.x = self.messenger.hub['SensorMonitor']['magX'].read()[0] / 1e3
                        mag.y = self.messenger.hub['SensorMonitor']['magY'].read()[0] / 1e3
                        mag.z = self.messenger.hub['SensorMonitor']['magZ'].read()[0] / 1e3
                        self.__send_log(f"response: Mag - [{mag.x}, {mag.y}, {mag.z}]")
                        self.mag_publisher.publish(mag)

                        sat = SatellitesGPS()
                        try:
                            sat.gps = self.messenger.hub['Ublox']['satGps'].read()[0]
                        except:
                            sat.gps = 0

                        try:
                            sat.glonass = self.messenger.hub['Ublox']['satGlonass'].read()[0]
                        except:
                            sat.glonass = 0
                        self.satellites_publisher.publish(sat)

                        self.__send_log("send: Global status")
                        status = self.messenger.hub['Ublox']['status'].read()[0]
                        self.__send_log(f"response: Global status - {status}")
                        self.global_status_publisher.publish(status)

                    except TypeError as e:
                        rospy.logerr(str(e))
                        rospy.loginfo("Restarting to activate Gnns module")
                        self.restart_board()
                    except:
                        self.__navSystem_except("Gnns Module")
                elif self.navSystem == 1:
                    try:
                        self.__send_log("send: Local position request")
                        local_point = Point()
                        local_point.x = self.messenger.hub['USNav_module']['x'].read()[0] * 0.001
                        local_point.y = self.messenger.hub['USNav_module']['y'].read()[0] * 0.001
                        local_point.z = self.messenger.hub['USNav_module']['z'].read()[0] * 0.001
                        self.__send_log(f"response: Local position - [{local_point.x}, {local_point.y}, {local_point.z}]")
                        self.local_position_publisher.publish(local_point)

                        self.__send_log("send: LPS yaw request")
                        yaw = self.messenger.hub['USNav_module']['yaw'].read()[0]
                        self.__send_log(f"response: LPS yaw - {yaw}")
                        self.local_yaw_publisher.publish(yaw)

                        self.__send_log("send: LPS velocity request")
                        lps_vel = Point()
                        lps_vel.x = self.messenger.hub['USNav_module']['velX'].read()[0]
                        lps_vel.y = self.messenger.hub['USNav_module']['velY'].read()[0]
                        lps_vel.z = self.messenger.hub['USNav_module']['velZ'].read()[0]
                        self.__send_log(f"response: LPS velocity - [{lps_vel.x}, {lps_vel.y}, {lps_vel.z}]")
                        self.local_velocity_publisher.publish(lps_vel)
                    except TypeError:
                        rospy.loginfo("Restarting to activate LPS module")
                        self.restart_board()
                    except:
                        self.navSystem_except("LPS")
                elif self.navSystem == 2:
                    try:
                        self.__send_log("send: OpticalFlow velocity request")
                        velocity = OptVelocity()
                        velocity.x = self.messenger.hub['SensorMonitor']['optFlowX'].read()[0]
                        velocity.y = self.messenger.hub['SensorMonitor']['optFlowY'].read()[0]
                        velocity.range = self.messenger.hub['SensorMonitor']['optFlowRange'].read()[0] / 1e3
                        self.__send_log(f"response: OpticalFlow velocity - [{velocity.x}, {velocity.y}, {velocity.range}]")
                        self.opt_velocity_publisher.publish(velocity)
                    except:
                        self.__navSystem_except("OpticalFlow Module")
            else:
                self.live = False
                self.disconnect()
        else:
            self.live = False

    def spin(self):
        if not self.restart:
            if ((self.messenger == None) and not self.live):
                try:
                    self.connect()
                except ValueError:
                    rospy.logfatal("Serial port not specified")
                    return False
                except:
                    self.messenger = None
                    self.__send_log("board: offline")
                    rospy.loginfo("Board is offline")
            else:
                self.data_exchange()
        if self.rate is not None:
            self.rate.sleep()
        return True

if __name__ == "__main__":
    rospy.init_node("ros_plaz_node") # инициализируем ноду
    try:
        uart = rospy.get_param(rospy.search_param("port")) # получение имени порта, как параметра ноды
    except:
        uart = "/dev/ttyS0"
    
    rate = rospy.Rate(100)
    ros_plaz_node = ROSPlazNode(uart, rate)

    while not rospy.is_shutdown():
        if not ros_plaz_node.spin():
            break
    
    ros_plaz_node.live = False
    ros_plaz_node.disconnect()
