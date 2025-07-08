# /// script
# dependencies = [
#   "matplotlib",
#   "numpy",
#   "tornado",
#   "pyqt6",
# ]
# ///

import time

import matplotlib.pyplot as plt
import numpy as np

import matplotlib


matplotlib.use("qtagg")

# Настройки карты
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-4, 4)
ax.set_ylim(-4, 4)
ax.set_title("Рисование курсором (удерживайте ЛКМ)")
ax.grid(True)

# Контейнер для данных
data_list = []
pressed = False  # Состояние кнопки мыши
start_time = time.time()  # Начальное время


# Обработчики событий мыши
def on_press(event):
    global pressed
    if (
        event.inaxes == ax and event.button == 1
    ):  # Левая кнопка в области графика
        pressed = True
        record_point(event.xdata, event.ydata, 1)


def on_release(event):
    global pressed
    if pressed and event.button == 1:
        pressed = False
        record_point(event.xdata, event.ydata, 0)


def on_move(event):
    if pressed and event.inaxes == ax:
        record_point(event.xdata, event.ydata, 1)


def record_point(x, y, z):
    timestamp = time.time() - start_time  # Относительное время
    data_list.append([x, y, z, timestamp])
    # Рисуем точку
    ax.plot(x, y, "ro" if z == 1 else "bo", markersize=5)
    fig.canvas.draw_idle()


# Сохранение при закрытии окна
def on_close(event):
    if data_list:
        np_data = np.array(data_list)
        np.save("data.npy", np_data)
        print(f"Данные сохранены в data.npy, записей: {len(data_list)}")


# Подключение обработчиков
fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("motion_notify_event", on_move)
fig.canvas.mpl_connect("close_event", on_close)

plt.show()
print(data_list)
