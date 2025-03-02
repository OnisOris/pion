import socket
import threading
from queue import Queue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from pion.datagram import DDatagram
import time

class SwarmVisualizer2D:
    def __init__(self, port=37020):
        self.port = port
        self.data_queue = Queue()
        self.drones = {}
        self.colors = {}
        self.trails_length = 30
        self.running = True
        self.lock = threading.Lock()  # Блокировка для синхронизации доступа

        # UDP сервер
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        
        # Запуск потока приёма данных
        self.receiver_thread = threading.Thread(target=self.receive_data)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

        # Инициализация графика
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.setup_plot()

    def setup_plot(self):
        self.ax.set_xlim(-5.5, 5.5)
        self.ax.set_ylim(-5.5, 5.5)
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Real-time 2D Drone Swarm Visualization')
        self.ax.grid(True)
        self.ax.set_aspect('equal')

    def receive_data(self):
        decoder = DDatagram()
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                valid, payload = decoder.read_serialized(data)
                # Проверяем, что данных достаточно (1 - IP, 3 - позиция, 3 - скорость)
                if valid and len(payload.data) >= 7:
                    self.process_payload(payload, addr)
            except Exception as e:
                print(f"Receive error: {e}")

    def process_payload(self, payload, addr):
        with self.lock:
            drone_id = payload.id
            if len(payload.data) < 7:
                return

            try:
                ip_num = int(payload.data[0])
                ip = socket.inet_ntoa(ip_num.to_bytes(4, byteorder='big'))
            except (OverflowError, IndexError):
                ip = "Invalid IP"

            # Позиция: индексы 1, 2, 3
            position = np.array([
                payload.data[1],  # X
                payload.data[2],  # Y
                payload.data[3]   # Z
            ])
            
            # Скорость: индексы 4, 5, 6
            velocity = np.array([
                payload.data[4],  # Vx
                payload.data[5],  # Vy
                payload.data[6]   # Vz
            ])
            
            if drone_id not in self.drones:
                self.colors[drone_id] = np.random.rand(3,)
                self.drones[drone_id] = {
                    'position': position,
                    'velocity': velocity,
                    'trail': [],
                    'ip': ip,
                    'last_update': time.time()
                }
            else:
                self.drones[drone_id]['position'] = position
                self.drones[drone_id]['velocity'] = velocity
                self.drones[drone_id]['last_update'] = time.time()
                
                trail = self.drones[drone_id]['trail']
                trail.append(position.copy()[:2])
                if len(trail) > self.trails_length:
                    trail.pop(0)

    def update_plot(self, frame):
        with self.lock:
            self.ax.clear()
            self.setup_plot()
            
            current_time = time.time()
            to_delete = []
            
            # Итерация по дронам
            for drone_id in list(self.drones.keys()):
                data = self.drones[drone_id]
                
                # Удаление неактивных дронов (без обновлений >5 сек)
                if current_time - data['last_update'] > 5:
                    to_delete.append(drone_id)
                    continue
                    
                color = self.colors[drone_id]
                pos = data['position']
                trail = np.array(data['trail'])
                velocity = data['velocity']
                
                # Рисуем дрона (точка)
                size = 80 + pos[2] * 5
                self.ax.scatter(
                    pos[0], 
                    pos[1], 
                    c=[color],
                    s=size,
                    marker='o',
                    edgecolors='k',
                    label=f'Drone {drone_id}'
                )
                
                # Текст с высотой
                self.ax.text(
                    pos[0] + 0.3, 
                    pos[1] + 0.3, 
                    f'{pos[2]:.1f}m',
                    color=color,
                    fontsize=8
                )
                
                # Рисуем траекторию (след)
                if len(trail) > 1:
                    self.ax.plot(
                        trail[:,0], 
                        trail[:,1], 
                        c=color, 
                        linestyle=':',
                        alpha=0.6,
                        linewidth=1
                    )
                
                # Рассчитываем угол (yaw) из вектора скорости (если ненулевой)
                if np.linalg.norm(velocity[:2]) > 0.001:
                    yaw = np.arctan2(velocity[1], velocity[0])
                else:
                    yaw = 0
                    
                # Рисуем стрелку направления (yaw) с фиксированной длиной
                self.draw_orientation(pos[:2], yaw, color)
                
                # Рисуем вектор скорости (масштабированный по модулю)
                self.draw_velocity_vector(pos[:2], velocity, color)
            
            # Удаление неактивных дронов
            for did in to_delete:
                del self.drones[did]
                del self.colors[did]
                
            if self.drones:
                self.ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1))

        return self.ax

    def draw_orientation(self, position, yaw, color):
        arrow_length = 1.2  # фиксированная длина стрелки направления
        dx = arrow_length * np.cos(yaw)
        dy = arrow_length * np.sin(yaw)
        
        self.ax.quiver(
            position[0],
            position[1],
            dx,
            dy,
            color=color,
            angles='xy',
            scale_units='xy',
            scale=1,
            width=0.003,
            headwidth=4,
            headlength=5
        )
    
    def draw_velocity_vector(self, position, velocity, color):
        # Масштабирование вектора скорости для наглядности
        factor = 0.5
        arrow_dx = velocity[0] * factor
        arrow_dy = velocity[1] * factor
        
        self.ax.quiver(
            position[0],
            position[1],
            arrow_dx,
            arrow_dy,
            color=color,
            angles='xy',
            scale_units='xy',
            scale=1,
            width=0.005,
            headwidth=5,
            headlength=7,
            alpha=0.8
        )

    def run(self):
        ani = FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=500,
            cache_frame_data=False
        )
        plt.show()

    def shutdown(self):
        self.running = False
        self.sock.close()
        plt.close('all')

if __name__ == "__main__":
    visualizer = SwarmVisualizer2D(port=37020)
    try:
        visualizer.run()
    except KeyboardInterrupt:
        visualizer.shutdown()
    finally:
        visualizer.shutdown()

