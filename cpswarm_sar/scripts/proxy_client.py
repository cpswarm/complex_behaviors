#!/usr/bin/env python3

import socket
import rospy

def main():
    """
    Forward ZRE beacons to discovery server.
    """
    # initialize ros node
    rospy.init_node("proxy_client")

    # read parameters
    server = rospy.get_param("~discovery_server")
    port_in = rospy.get_param("~port_in", 5670)
    port_out = rospy.get_param("~port_out", 6780)

    # network socket for incoming udp beacons (broadcast)
    incoming = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    incoming.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    incoming.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    incoming.bind(("", port_in))

    # network socket for outgoing udp beacons
    outgoing = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # keep running as long ros is running
    rate = rospy.Rate(1) # in hertz
    while not rospy.is_shutdown():
        # receive beacon
        data, address = incoming.recvfrom(1024)

        # only consider locally originating messages
        if address[0] == socket.gethostbyname(socket.gethostname()):
            # debug output
            rospy.logerr(f"Forwarded beacon from {address[0]}:{address[1]} to {server}:{port_out}: {data}")

            # send beacon to server
            outgoing.sendto(data, (server, port_out))

        rate.sleep()

if __name__ == "__main__":
    try:
      main()
    except rospy.ROSInterruptException:
        pass
