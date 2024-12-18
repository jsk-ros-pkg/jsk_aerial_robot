import socket
import struct
import netifaces
import datetime

def get_wifi_ip():
    # Get the default gateway interface
    gateway_interface = netifaces.gateways().get('default')
    if gateway_interface:
        gateway_interface_name = gateway_interface[2][-1]

        # Get the IP address of the Wi-Fi interface
        wifi_ip = netifaces.ifaddresses(gateway_interface_name).get(netifaces.AF_INET)
        if wifi_ip:
            return wifi_ip[0]['addr']
    return None  # Return None if Wi-Fi interface is not found

def send_igmp_v2_membership_report(group_ip, interface_ip):
    # IGMPv2 Membership Report packet format
    # Type (1 byte), Max Resp Time (1 byte), Checksum (2 bytes), Group Address (4 bytes)

    IGMP_MEMBERSHIP_REPORT_TYPE = 0x16  # IGMPv2 Membership Report
    MAX_RESP_TIME = 0  # Not used in Membership Report
    CHECKSUM_PLACEHOLDER = 0

    # Convert group IP to bytes
    group_bytes = socket.inet_aton(group_ip)

    # Create IGMP packet with a placeholder checksum
    igmp_packet = struct.pack('!BBH4s',
                               IGMP_MEMBERSHIP_REPORT_TYPE,
                               MAX_RESP_TIME,
                               CHECKSUM_PLACEHOLDER,
                               group_bytes)

    # Calculate checksum
    def calculate_checksum(data):
        if len(data) % 2:
            data += b'\x00'
        checksum = sum(struct.unpack('!%dH' % (len(data) // 2), data))
        checksum = (checksum >> 16) + (checksum & 0xFFFF)
        checksum += checksum >> 16
        return ~checksum & 0xFFFF

    checksum = calculate_checksum(igmp_packet)

    # Rebuild the packet with the correct checksum
    igmp_packet = struct.pack('!BBH4s',
                               IGMP_MEMBERSHIP_REPORT_TYPE,
                               MAX_RESP_TIME,
                               checksum,
                               group_bytes)

    # Create a raw socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_IGMP)

    # Bind to the interface
    sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(interface_ip))

    # Send the IGMP packet
    sock.sendto(igmp_packet, (group_ip, 0))
    dt_now = datetime.datetime.now()
    print("[{}] IGMPv2 Membership Report sent to {} from {}".format(dt_now, group_ip, interface_ip))

# Example usage
group_ip = "239.255.42.99"  # Multicast group address
interface_ip = get_wifi_ip()  # Automatically detect the IP address of the Wi-Fi interface
if interface_ip:
    send_igmp_v2_membership_report(group_ip, interface_ip)
else:
    dt_now = datetime.datetime.now()
    print("[{}] Wi-Fi interface not found.".format(dt_now))
