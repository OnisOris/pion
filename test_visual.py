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
        self.lock = threading.Lock()  # Добавлена блокировка

        # UDP server setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        
        # Start threads
        self.receiver_thread = threading.Thread(target=self.receive_data)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

        # Initialize plot
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
                if valid and len(payload.data) >= 4:  # Проверка длины данных
                    self.process_payload(payload, addr)
            except Exception as e:
                print(f"Receive error: {e}")

    def process_payload(self, payload, addr):
        with self.lock:  # Блокировка доступа
            drone_id = payload.id
            if len(payload.data) < 4:  # Защита от неполных данных
                return

            try:
                ip_num = int(payload.data[0])
                ip = socket.inet_ntoa(ip_num.to_bytes(4, byteorder='big'))
            except (OverflowError, IndexError):
                ip = "Invalid IP"

            position = np.array([
                payload.data[1],  # X
                payload.data[2],  # Y
                payload.data[3]   # Z
            ])
            
            attitude = np.array(payload.data[4:7]) if len(payload.data) >=7 else np.zeros(3)
            
            if drone_id not in self.drones:
                self.colors[drone_id] = np.random.rand(3,)
                self.drones[drone_id] = {
                    'position': position,
                    'attitude': attitude,
                    'trail': [],
                    'ip': ip,
                    'last_update': time.time()
                }
            else:
                self.drones[drone_id]['position'] = position
                self.drones[drone_id]['attitude'] = attitude
                self.drones[drone_id]['last_update'] = time.time()
                
                trail = self.drones[drone_id]['trail']
                trail.append(position.copy()[:2])
                if len(trail) > self.trails_length:
                    trail.pop(0)

    def update_plot(self, frame):
        with self.lock:  # Блокировка доступа
            self.ax.clear()
            self.setup_plot()
            
            current_time = time.time()
            to_delete = []
            
            # Итерация по копии списка ключей
            for drone_id in list(self.drones.keys()):
                data = self.drones[drone_id]
                
                if current_time - data['last_update'] > 5:
                    to_delete.append(drone_id)
                    continue
                    
                color = self.colors[drone_id]
                pos = data['position']
                trail = np.array(data['trail'])
                
                # Отрисовка дрона
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
                
                # Траектория
                if len(trail) > 1:
                    self.ax.plot(
                        trail[:,0], 
                        trail[:,1], 
                        c=color, 
                        linestyle=':',
                        alpha=0.6,
                        linewidth=1
                    )
                
                # Ориентация
                yaw = data['attitude'][2]
                self.draw_orientation(pos[:2], yaw, color)
            
            # Удаление неактивных дронов
            for did in to_delete:
                del self.drones[did]
                del self.colors[did]
                
            if self.drones:
                self.ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1))

        return self.ax

    def draw_orientation(self, position, yaw, color):
        arrow_length = 1.2
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
