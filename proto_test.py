from pion.control_server import ControlServer
from pion.server import SwarmCommunicator
import time
#
if __name__ == "__main__":
    control_server = ControlServer(broadcast_port=37020)
    control_server.console_loop()

