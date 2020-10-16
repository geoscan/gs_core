# Описание пакета gs_core (Ранее gs_nodes)

## Описание:
В данном пакете находятся необходимые основные ноды для корректной работы Geoscan Pioneer Max

## Состав пакета:
1. Ноды:
* ros_serial_node
2. Файлы запуска:
* pioneer.launch - запуск системы

## Описание нод:
### 1. ros_serial_node
Нода связи по протоколу UART между полетным контроллером Geoscan Pioneer и микрокомпьютером

#### Параметры:
* port(string) - имя UART порта (пример: /dev/ttyS0), обязательный параметр 

#### Сервисы:
* geoscan/alive (gs_interfaces/Live) - возвращает статус соединения
* geoscan/log_service (gs_interfaces/Log) - возвращает лог
* geoscan/flight/event_service (gs_interfaces/Event) - приказывает автопилоту выполнить Event
* geoscan/flight/yaw (gs_interfaces/Yaw) -приказывает автоплоту выполнить рысканье
* geoscan/flight/local_position_service (gs_interfaces/Position) - приказывает автопилоту выполнить перемещение в локальных координатах
* geoscan/flight/gps_position_service (gs_interfaces/PositionGPS) - риказывает автопилоту выполнить перемещение в GPS координатах
* geoscan/led/board/control_service (gs_interfaces/Led) - управление светодиодами на плате Geoscan Pioneer
* geoscan/led/module/control_service (gs_interfaces/Led) - управление светодиодами на LED модуле
* geoscan/board/info_service (gs_interfaces/Info) - возвращает бортовой номер
* geoscan/board/time_service (gs_interfaces/Time) - возвращает время с момента включения коптера
* geoscan/board/delta_time_service (gs_interfaces/Time) - возвращает разницу в секундах между временем включения коптера и глобальным временем системы навигации
* geoscan/board/launch_time_service (gs_interfaces/Time) - возвращает время запуска для системы навигации
* geoscan/sensors/lpsvel_service (gs_interfaces/LpsVel) - возвращает скорость коптера возвращаемую LPS
* geoscan/sensors/lpsyaw_service (gs_interfaces/LpsYaw) - возвращает угол поворота в системе LPS
* geoscan/sensors/gyro_service (gs_interfaces/Gyro) - возвращает данные c гироскопа
* geoscan/sensors/accel_service (gs_interfaces/Accel) - возвращает данные c акселерометра
* geoscan/sensors/orientation_service (gs_interfaces/Orientation) - возвращает данные положения
* geoscan/sensors/altitude_service (gs_interfaces/Altitude) - возвращает данные высоты по барометр
* geoscan/sensors/altitude_service (gs_interfaces/Cargo) - управление модулем магнитного захвата
* geoscan/navigation/system (gs_interfaces/NavigationSystem) - возвращает текущую систему позиционирования

#### Топики:
* geoscan/log_topic (std_msgs/String) - последнее сообщение лога
* geoscan/battery_state (gs_interfaces/SimpleBatteryState) - состояние АКБ
* geoscan/local_position (geometry_msgs/Point) - локальные координаты в ситеме LPS
* geoscan/global_position (gs_interfaces/PointGPS) - глобальные координаты GPS
* geoscan/opt_velocity (gs_interfaces/OptVelocity) - данные с модуля оптического потока (OPT)

## Необходимые пакеты:
1. Python:
    * PySerial
2. ROS:
    * gs_interfaces
    * geometry_msgs
    * std_msgs

## Использование:

 ```rosparam set ros_serial_node/port /dev/ttyS0```
 
 ```rosrun gs_core ros_serial_node.py ```
