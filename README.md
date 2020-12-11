# Описание пакета gs_core (Ранее gs_nodes)

## Описание:
В данном пакете находятся необходимые основные ноды для корректной работы Geoscan Pioneer Max

## Состав пакета:
1. Ноды:
* ros_plaz_node
2. Файлы запуска:
* pioneer.launch - запуск системы

## Описание нод:
### 1. ros_plaz_node
Нода связи по протоколу plazlink между полетным контроллером Geoscan Pioneer и микрокомпьютером

#### Параметры:
* port(string) - имя UART порта (пример: /dev/ttyS0), обязательный параметр 

#### Сервисы:
* geoscan/alive (gs_interfaces/Live) - возвращает статус соединения
* geoscan/get_log (gs_interfaces/Log) - возвращает лог
* geoscan/flight/set_event (gs_interfaces/Event) - приказывает автопилоту выполнить Event
* geoscan/flight/set_yaw (gs_interfaces/Yaw) - приказывает автоплоту выполнить рысканье
* geoscan/flight/set_local_position (gs_interfaces/Position) - приказывает автопилоту выполнить перемещение в локальных координатах
* geoscan/flight/set_global_position (gs_interfaces/PositionGPS) - приказывает автопилоту выполнить перемещение в глобальных координатах
* geoscan/led/board/set (gs_interfaces/Led) - управление светодиодами на плате Geoscan Pioneer
* geoscan/led/module/set (gs_interfaces/Led) - управление светодиодами на LED модуле
* geoscan/cargo/set (gs_interfaces/Cargo) - управление модулем магнитного захвата
* geoscan/board/get_info (gs_interfaces/Info) - возвращает бортовой номер
* geoscan/board/get_time (gs_interfaces/Time) - возвращает время с момента включения коптера
* geoscan/board/get_uptime (gs_interfaces/Time) - возвращает время запуска для системы навигации
* geoscan/board/get_flight_time (gs_interfaces/Time) - возвращает время с начала полета
* geoscan/navigation/get_system (gs_interfaces/NavigationSystem) - возвращает текущую систему позиционирования

#### Топики:
* geoscan/log (std_msgs/String) - последнее сообщение лога
* geoscan/battery_state (gs_interfaces/SimpleBatteryState) - состояние АКБ
* geoscan/navigation/local/position (geometry_msgs/Point) - локальные координаты в ситеме LPS
* geoscan/navigation/local/yaw (std_msgs/Float32) - угол поворота в системе LPS
* geoscan/navigation/local/velocity (geometry_msgs/Point) - скорость коптера возвращаемая LPS
* geoscan/navigation/global/position (gs_interfaces/PointGPS) - глобальные координаты GPS
* geoscan/navigation/global/status (std_msgs/Int8) - статус GPS модуля
* geoscan/navigation/satellites (gs_interfaces/SatellitesGPS)- количество спутников
* geoscan/navigation/opt/velocity (gs_interfaces/OptVelocity) - данные с модуля оптического потока (OPT)
* geoscan/flight/callback_event (std_msgs/Int32) - события вытопилота
* geoscan/sensors/gyro (geometry_msgs/Point) - данные c гироскопа
* geoscan/sensors/accel (geometry_msgs/Point) - данные c акселерометра
* geoscan/sensors/orientation (gs_interfaces/Orientation) - данные положения
* geoscan/sensors/altitude (std_msgs/Float32) - данные высоты по барометр
* geoscan/sensors/mag (geometry_msgs/Point) - данные магнитометра

## Необходимые пакеты:
1. Python:
    * PySerial
2. ROS:
    * gs_interfaces
    * geometry_msgs
    * std_msgs

## Использование:

 ```rosparam set ros_plaz_node/port /dev/ttyS0```
 
 ```rosrun gs_core ros_plaz_node.py ```
