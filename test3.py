from collections import deque
from rich.console import Console
from rich.table import Table

# Пример логов
logs = {
    1: "Система запущена",
    2: "Произошла ошибка сети",
    3: "Подключение восстановлено",
    4: "Начата обработка данных",
    5: "Обработка завершена",
    6: "Отправка уведомления",
    7: "Сеанс завершен"
}

console = Console()

def print_latest_logs(log_dict: dict, n: int = 5):
    # Берем последние n записей из словаря
    latest_logs = deque(log_dict.items(), maxlen=n)

    # Создаем таблицу для красивого вывода
    table = Table(title="Последние логи")

    table.add_column("ID", style="cyan", justify="right")
    table.add_column("Сообщение", style="green")

    for log_id, message in latest_logs:
        table.add_row(str(log_id), message)

    console.print(table)

# Вывод последних 5 сообщений
print_latest_logs(logs)
