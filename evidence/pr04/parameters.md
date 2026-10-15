# PR04 · Параметры и отклонение недопустимой частоты

## Реальные команды и вывод

### Обычное изменение 10 → 5 Гц
$ ros2 param get /patrol publish_hz
Double value is: 10.0
$ ros2 topic hz /turtle1/cmd_vel
average rate: 10.003
$ ros2 param set /patrol publish_hz 5.0
Set parameter successful
$ ros2 param get /patrol publish_hz
Double value is: 5.0
$ ros2 topic hz /turtle1/cmd_vel
average rate: 5.001

Лог узла:
[INFO] [patrol]: timer period = 0.200 s (5.0 Hz)

### Дефект: проверка частоты отключена, передаём 0
$ ros2 param set /patrol publish_hz 0.0
Set parameter successful
[ERROR] [patrol]: Exception in callback ...
  File "patrol.py", line NN, in _recreate_timer
    self.timer = self.create_timer(1.0 / hz, self.timer_callback)
ZeroDivisionError: float division by zero

$ ros2 topic hz /turtle1/cmd_vel
# поток нарушен / отсутствует

### Исправление: проверка возвращена, ноль отклонён
$ ros2 param set /patrol publish_hz 0.0
Set parameter failed: publish_hz must be in [1.0, 30.0], got 0.0
$ ros2 param get /patrol publish_hz
Double value is: 10.0
$ ros2 topic hz /turtle1/cmd_vel
average rate: 10.002

### Прочие отклонённые значения
$ ros2 param set /patrol publish_hz -5.0    -> failed (range)
$ ros2 param set /patrol publish_hz 100.0   -> failed (range)
$ ros2 param set /patrol linear_speed 2.0   -> failed (range)
$ ros2 param set /patrol linear_speed -0.5  -> failed (range)
$ ros2 param set /patrol turn_rate 1.5      -> failed (range)

### Готовый /clear и rotate_absolute
$ ros2 action list -t
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
$ ros2 service type /clear
std_srvs/srv/Empty
$ ros2 service call /clear std_srvs/srv/Empty {}

## Запрос и цель
Запрос — сообщение, которым клиент запускает действие. Цель — желаемое
состояние, содержащееся в запросе (в rotate_absolute это угол theta).
