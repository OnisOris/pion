# /// script
# dependencies = [
#   "matplotlib",
#   "numpy",
#   "scikit-learn",
#   "pandas",
#   "scipy",
#   "pyqt6",
# ]
# ///
import os
import shutil
import sys
from os.path import isdir

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.preprocessing import StandardScaler

path = "./data/"
save = "./save/"
files = os.listdir(path)
args = sys.argv
discription = ""
without_acc = False
without_moving = False
if len(args) > 1:
    if "-d" in args:
        discription = f"{args[args.index('-d') + 1]}"
    if "-na" in args:
        without_acc = True
    if "-nm" in args:
        without_moving = True

for file in files:
    path2 = f"{save}{file[:-4]}/"
    if not isdir(path2):
        from os import makedirs

        makedirs(path2, exist_ok=True)

    array = np.load(f"{path}{file}")
    print(array.shape)
    df = pd.DataFrame(
        array,
        columns=[
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "Vz",
            "pitch",
            "roll",
            "yaw",
            "pitch_speed",
            "roll_speed",
            "yaw_speed",
            "vx_c",
            "vy_c",
            "vz_c",
            "v_yaw_c",
            "t",
        ],
    )
    title = f"{file}\n{discription}"
    try:
        ### График 1 (все данные)
        ax = df.plot(x="t", title=title, figsize=(10, 6))
        ax.legend(fontsize=8)
        plt.locator_params(axis="x", nbins=30)
        ax.set_xlabel("Время с начала инициализации [с]")
        ax.set_ylabel("Расстояние [м]")
        ax.grid(True)

        # Обычная числовая ось X
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.savefig(f"{path2}all.png", dpi=300)

        ### График 2 (полиномиальная регрессия)
        tu = array[:, 16]
        x = array[:, 0]
        vx = array[:, 3]
        t = tu.reshape(-1, 1)

        # Масштабируем время t
        scaler = StandardScaler()
        t_scaled = scaler.fit_transform(t)

        # Перебор вручную для нахождения лучшей степени полинома
        best_degree_x = 1
        best_degree_vx = 1
        best_score_x = float("-inf")
        best_score_vx = float("-inf")

        plt.figure(figsize=(10, 6))
        plt.scatter(t, vx, color="green", label="Исходные данные vx")
        plt.xlabel("Время (t)")
        plt.ylabel("Скорость (vx)")
        plt.legend()
        plt.savefig(f"{path2}random_forest.png", dpi=300)

        ### График 4 (скорость и ускорение с разными осями Y)

        plt.figure(figsize=(10, 6))

        # Создаем основной график для скорости
        fig, ax1 = plt.subplots(figsize=(10, 6))

        # Первая ось Y для скорости
        ax1.set_xlabel("Время (с)")
        ax1.set_ylabel("Скорость (м/с)", color="green")
        ax1.plot(df["t"], df["vx_c"], label="Vx (control)", color="red")
        ax1.plot(df["t"], df["vx"], label="Vx (Locus)", color="green")
        ax1.tick_params(axis="y", labelcolor="green")

        # Ограничим масштаб по скорости для улучшенной визуализации
        ax1.set_ylim(-2, 2)

        # Создаем вторую ось Y для ускорения
        ax2 = ax1.twinx()

        # Ограничим масштаб по ускорению
        ax2.set_ylim(-10, 10)

        # Добавим легенды для обоих графиков
        fig.tight_layout()
        fig.suptitle(f"График скоростей и ускорений\n{discription}", y=1.05)

        # Сетка и настройки оси X
        ax1.grid(True)
        plt.locator_params(axis="x", nbins=30)
        plt.xticks(rotation=45)
        plt.legend()
        # Сохранение графика
        plt.tight_layout()
        plt.savefig(f"{path2}velocity_acceleration_dual_axis.png", dpi=300)

        if len(args) > 1:
            if "-p" in args:
                plt.show()
            if "-d" in args:
                discription = f"{args[args.index('-d') + 1]}"

        with open(f"{path2}discription.txt", "w") as file_stat:
            file_stat.write(discription)
        if not without_moving:
            shutil.move(f"{path}{file}", f"{path2}{file}")
    except:
        print("Error...noving .npy file in ./error/")
        shutil.move(f"{path}{file}", f"{'./error/'}{file}")
