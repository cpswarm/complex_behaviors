#!/usr/bin/env python3

import rospy
import socket
import ipaddress as ip
from scapy.all import IP,UDP,send
from scapy.config import conf


def decode(data):
    """
    Decode a ZRE beacon message.

    # Parameters
    - data: Data frame of the ZRE beacon.

    # Returns
    - ZeroMQ UUID of the sender.
    - Network socket port number.
    """
    # invalid packet length
    if len(data) != 22:
        return 0,0

    # invalid header
    if data[0:3].decode() != "ZRE" or data[3] != 1:
        return 0,0

    # decode packet
    uuid = data[4:20].hex()
    port = int(data[20:22].hex(), 16)

    return uuid,port

def main():
    """
    Forward ZRE beacons to discovery server and broadcast remote beacons locally.
    """
    # initialize ros node
    rospy.init_node("proxy_client")

    # read parameters
    server = rospy.get_param("~discovery_server")
    port_local = rospy.get_param("~port_local", 5670)
    port_in = rospy.get_param("~port_in", 6780)
    port_out = rospy.get_param("~port_out", 7890)
    network = rospy.get_param("~network", 16)

    # determine ip address
    temp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    temp.connect((server, port_out))
    my_ip = temp.getsockname()[0]
    temp.close()

    # determine broadcast address
    net = ip.IPv4Network(my_ip + "/" + str(network), False)
    bc_ip = str(net.broadcast_address)

    # network socket for incoming local udp beacon broadcast
    bc_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    bc_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    bc_in.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    bc_in.bind((bc_ip, port_local))
    bc_in.settimeout(0.2)

    # network socket for incoming udp server beacons
    sv_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sv_in.bind((my_ip, port_in))
    sv_in.settimeout(0.2)

    # network socket for outgoing udp server beacons
    sv_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # disable scapy output
    conf.verb=0

    # keep running as long ros is running
    rate = rospy.Rate(1) # in hertz
    while not rospy.is_shutdown():
        # listen to local beacon broadcast
        try:
            # receive beacon
            data, address = bc_in.recvfrom(1024)

            # forward locally originating broadcast messages to server
            if address[0] == my_ip:
                # debug output
                rospy.logdebug(f"Forwarded beacon from {address[0]}:{address[1]} to {server}:{port_out}: {decode(data)}")

                # send beacon to server
                sv_out.sendto(data, (server, port_out))

        # nothing received
        except socket.error as e:
            rospy.logdebug(f"No local broadcast beacon received: {e}")

        # listen to beacons received from server
        try:
            # receive beacon
            data, address = sv_in.recvfrom(1024)

            # debug output
            rospy.logdebug(f"Broadcast beacon from {address[0]}:{address[1]} locally: {decode(data)}")

            # broadcast beacons received from server locally
            message = IP(src=address[0], dst=bc_ip) / UDP(dport=port_local) / data
            send(message)

        # nothing received
        except socket.error as e:
            rospy.logdebug(f"No beacon received from server: {e}")

        rate.sleep()

if __name__ == "__main__":
    try:
      main()
    except rospy.ROSInterruptException:
        pass
