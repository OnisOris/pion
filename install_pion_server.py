import argparse
from pion.gpion import Gpion
# python test_gpion.py 206 --ssh_host 10.1.100.206
def main():
    parser = argparse.ArgumentParser(
        description="Установка pion_server на raspberry pi zero 2w"
    )
    parser.add_argument(
        "--ssh_host",
        type=str,
        default=None,
        help="SSH хост для проверки Pion server (например, 10.1.100.121)"
    )
    parser.add_argument(
        "--ssh_user",
        type=str,
        default="pi",
        help="SSH пользователь (по умолчанию 'pi')"
    )
    parser.add_argument(
        "--ssh_password",
        type=str,
        default="raspberry",
        help="SSH пароль (по умолчанию 'raspberry')"
    )
    args = parser.parse_args()
    ip = f"10.1.100.{args.device_number}"
    drone = Gpion(ip=ip, mavlink_port=5656, name=f"Drone_{args.device_number}", dt=0.1)
    drone.remove_existing_pion_service(args.ssh_host, args.ssh_user, args.ssh_password)
    drone.check_pion_server_raspb(args.ssh_host, args.ssh_user, args.ssh_password)
    
 
