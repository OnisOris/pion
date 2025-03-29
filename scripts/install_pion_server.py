import argparse

from src.pion.gpion import Gpion


# python test_gpion.py 206 --ssh_host 10.1.100.206
def main():
    parser = argparse.ArgumentParser(
        description="Установка pion_server на raspberry pi zero 2w"
    )
    parser.add_argument(
        "--ssh_host",
        type=str,
        default=None,
        help="SSH хост для проверки Pion server (например, 10.1.100.121)",
    )
    parser.add_argument(
        "--ssh_user",
        type=str,
        default="pi",
        help="SSH пользователь (по умолчанию 'pi')",
    )
    parser.add_argument(
        "--ssh_password",
        type=str,
        default="raspberry",
        help="SSH пароль (по умолчанию 'raspberry')",
    )
    args = parser.parse_args()
    ip = args.ssh_host
    drone = Gpion(
        ip=ip, mavlink_port=5656, name="Drone", dt=0.1, start_from_init=False
    )
    drone.remove_existing_pion_service(
        args.ssh_host, args.ssh_user, args.ssh_password
    )
    drone.check_pion_server_raspb(
        args.ssh_host, args.ssh_user, args.ssh_password
    )


main()
