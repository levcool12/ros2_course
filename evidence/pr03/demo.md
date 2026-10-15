# PR03 Demo: patrol node

## Роли init, spin, callback, Ctrl+C

- `rclpy.init()` — инициализирует контекст ROS 2. Без него нельзя создать ни один узел.
- `rclpy.spin(node)` — блокирующий цикл обработки событий: входящие сообщения (pose_callback) и срабатывания таймера (timer_callback).
- `pose_callback` — вызывается подпиской при получении `/turtle1/pose`; сохраняет последнее сообщение в `self.last_pose`.
- `timer_callback` — вызывается таймером каждые 0.1 с; берёт позу, вызывает `compute_command` и публикует Twist.
- Ctrl+C — прерывает `spin`, узел корректно уничтожается. При этом turtlesim продолжает движение по инерции, пока не истечёт время или не придёт нулевая команда: исчезновение процесса не означает мгновенную остановку.

## Стадия 1 · Собрать

Пакет `patrol` создан через `ros2 pkg create --build-type ament_python --node-name patrol patrol --dependencies rclpy geometry_msgs`.
- Подписка на `/turtle1/pose` хранит последнюю позу как поле объекта.
- Таймер 0.1 с публикует `geometry_msgs/msg/Twist`.
- Чистая функция `compute_command(pose)` возвращает нулевую команду без позы и `linear.x=0.5, angular.z=0.3` при наличии позы.

## Стадия 2 · Сломать

Публикация идёт в **относительный** `cmd_vel`, который не соединён с `/turtle1/cmd_vel`.
Проверено: `ros2 topic info /turtle1/cmd_vel` — `Publisher count: 0`; черепашка не двигается.

## Стадия 3 · Доказать

Remap `-r cmd_vel:=/turtle1/cmd_vel` соединяет узел с turtlesim. Черепашка начинает двигаться.
Измеренная частота: `ros2 topic hz /turtle1/cmd_vel` ≈ 10 Гц за 10 с.

## Наблюдение после Ctrl+C

После остановки ноды turtlesim продолжает движение по инерции — подтверждает, что исчезновение процесса не команда торможения.
