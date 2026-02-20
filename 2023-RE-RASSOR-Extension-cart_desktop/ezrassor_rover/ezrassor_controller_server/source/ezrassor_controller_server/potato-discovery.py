import logging
import socket
import time
from zeroconf import Zeroconf, ServiceInfo

def get_valid_ip():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            if ip:
                return ip
        except:
            pass
        time.sleep(1)  # wait for a second before retrying


logging.basicConfig(level=logging.INFO, format='%(message)s')

ip_address = get_valid_ip()
logging.info(f"Valid IP obtained: {ip_address}")

service_type = "_potato._tcp.local."
service_name = f"{socket.gethostname()}"
port = 8080
txt = {'ip': ip_address}

zeroconf = Zeroconf()
info = ServiceInfo(service_type, f"{service_name}.{service_type}",
                   addresses=[socket.inet_aton(ip_address)], port=port, properties=txt)

zeroconf.register_service(info)
logging.info(f"Potato device is now advertised on the network with IP: {ip_address} and port: {port}")

try:
    while True:
        time.sleep(3600)
except KeyboardInterrupt:
    pass
finally:
    zeroconf.unregister_service(info)
    zeroconf.close()
