# NTO Aerial Robotics — README


## 0. Установка симулятора Clover

Следуйте официальной инструкции:
[https://clover.coex.tech/ru/simulation.html](https://clover.coex.tech/ru/simulation.html)

## 1. Клонирование репозитория

```bash
git clone https://github.com/marmandala/NTO-Aerial-Robotics.git
cd NTO-Aerial-Robotics
```

---

## 2. Запуск скрипта для настройки симулятора

```bash
python3 setup_simulation.py
```

---

## 3. Генерация случайного мира

```bash
python3 generate_world.py
```

---

## 4. Запуск симулятора Gazebo

Запускаем симулятор Clover c PX4 SITL:

```bash
roslaunch clover_simulation simulator.launch
```
---

## 5. Запуск основной ноды управления дроном

```bash
python3 main_controller.py
```
---

## 6. Панель управления миссией

[http://localhost:8000](http://localhost:8000)

Откройте в браузере. На панели доступно:

* Автоматически обновляемая 2d карта
* Кнопки управления миссией (Start Mission, Stop Mission, Kill Switch)

---
