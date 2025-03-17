from src.swarm_server.control_server import ControlServer

#
if __name__ == "__main__":
    control_server = ControlServer(broadcast_port=37020)
    control_server.console_loop()

