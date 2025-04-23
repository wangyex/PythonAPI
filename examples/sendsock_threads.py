# To break: CNTL-FN-Pause

import time
import socket
import struct
import numpy as np
import platform
import threading
from datetime import datetime
from dataclasses import dataclass

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.254.254.254', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()

    return IP

dataclass 
class telemetry_carla:
    cid: float
    pt: float
    totmcs: float

    posX: float
    posY: float
    posZ: float

    rotX: float
    rotY: float
    rotZ: float

dataclass 
class cars:
    cID: float
    cNO: float
    cTOT: float

    cPX: float
    cPY: float
    cPZ: float

    cRX: float
    cRY: float
    cRZ: float

dataclass 
class motionSim_carla:

    # World position of the car
    posX: float
    posY: float
    posZ: float

    # Velocity of the car
    velX: float
    velY: float
    velZ: float

    # Linear acceleration of the car (*gravity not included)
    accX: float
    accY: float
    accZ: float

    # Roll, pitch and yaw positions of the car ("euler angles")
    rollPos: float
    pitchPos: float
    yawPos: float

    # Roll, pitch and yaw "velocities" of the car (angular velocity)
    rollRate: float
    pitchRate: float
    yawRate: float

    # Roll, pitch and yaw "accelerations" of the car (angular acceleration)
    rollAcc: float
    pitchAcc: float
    yawAcc: float

# (global) Variables.
motion_on   = True   # are we sending out vehicle dynamics?
tele_send   = True   # do we want to SEND telemetry?
tele_recv   = True   # do we want to RECV telemetry?
sim_dur     = 300.0   # time of execution (seconds).
tlr_socklen = 0      # length (bytes) of tlr send packet.
tlr_double  = 0      # length (bytes) of double precision array.
rec_to      = 5      # tlr timeout (seconds)
slp_mb      = 0.001      # Send sleep to MB (seconds). Maybe try 100 Hz. (0.01) moving forward.
slp_tls     = 0.001     # Send sleep to TL (seconds). Maybe try 100 Hz. (0.01) moving forward.
slp_rec_tmp = 0      # Temporary sleep on TL send - just to test timing (seconds).

# Loopback interface for motion data. 
UDP_IP = "127.0.0.1"
# UDP port for motion data.
UDP_PORT = 5005

# Multicast interface (network) and group address.
NET_IP = get_ip()
MCAST_GRP = '224.1.1.1'
# Multicast port for telemetry data.
MCAST_PORT = 5007 
MULTICAST_TTL = 4

# UNIQUE ID for each Carla participant.
GRP_ID = 5551.00

def mb_sender():
    # Setup MB socket.
    mb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Create a variable instance of the motionSim_carla() class to send via UDP (loopback).
    var_mb = motionSim_carla()

    # Initialize timing parameters.
    mb_start_time = time.time()
    mb_end_time = mb_start_time + sim_dur

    # MB global iterator
    mb_cnt = 0

    while time.time() < mb_end_time:
        # Print the MB iteration.
        print("- MB Iteration: #{:d} MB SimTime: {:.3f} sec." .format(mb_cnt, (time.time()-mb_start_time)))

        # Assign (test) values for motion class objects.
        motionSim_carla.posX = 2112.+ mb_cnt
        motionSim_carla.posY = 2113.+ mb_cnt
        motionSim_carla.posZ = 2114.+ mb_cnt
        motionSim_carla.velX = 666.+ mb_cnt
        motionSim_carla.velY = 667.+ mb_cnt
        motionSim_carla.velZ = 668.+ mb_cnt
        motionSim_carla.accX = 5150.+ mb_cnt
        motionSim_carla.accY = 5151.+ mb_cnt
        motionSim_carla.accZ = 5152.+ mb_cnt
        motionSim_carla.rollPos = 7112.+ mb_cnt
        motionSim_carla.pitchPos = 7113.+ mb_cnt
        motionSim_carla.yawPos = 7114.+ mb_cnt
        motionSim_carla.rollRate = 1666.+ mb_cnt
        motionSim_carla.pitchRate = 1667.+ mb_cnt
        motionSim_carla.yawRate = 1668.+ mb_cnt
        motionSim_carla.rollAcc = 8150.+ mb_cnt
        motionSim_carla.pitchAcc = 8151.+ mb_cnt
        motionSim_carla.yawAcc = 8152.+ mb_cnt

        # Create a structure for the Carla motion data - all floats.
        carla_vd_floats = np.array([var_mb.posX, var_mb.posY, var_mb.posZ, var_mb.velX, var_mb.velY, var_mb.velZ, var_mb.accX, var_mb.accY, var_mb.accZ, var_mb.rollPos, var_mb.pitchPos, var_mb.yawPos, var_mb.rollRate, var_mb.pitchRate, var_mb.yawRate, var_mb.rollAcc, var_mb.pitchAcc, var_mb.yawAcc], dtype=np.float32)

        # Pack the structured floats into a binary string.
        packed_data_mb = struct.pack('f' * len(carla_vd_floats), *carla_vd_floats)

        # Send the packed data for motion system.
        if motion_on == 1:
            mb_sock.sendto(packed_data_mb, (UDP_IP, UDP_PORT))

        # Sleep to maintain 100 Hz. communication with MB client program.
        time.sleep(slp_mb)

        # Increment the MB iterator.
        mb_cnt += 1

        # Print a separator.
        print("")
    
    # Close socket.
    mb_sock.close()
    print("- MB send thread done.  Socket closed.")

def tl_sender():
    # Setup Telemetry socket (send).
    tls_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    tls_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)

    # Create a variable instance of the telemetry_carla() class to send via UDP (multicast).
    var_tl = telemetry_carla()

    # Timing parameters.
    tls_start_time = time.time()
    tls_end_time = tls_start_time + sim_dur

    # TLs global iterator
    tls_cnt = 0

    while time.time() < tls_end_time:
        # Print the TLs iteration.
        print("/ TLs Iteration: #{:d} TLs SimTime: {:.3f} sec." .format(tls_cnt, (time.time()-tls_start_time)))

        # Get current (before) timestamp.
        now = datetime.now()
        b_minutes = float(now.minute)
        b_second = float(now.second)
        b_microseconds = float(now.microsecond)
        b_total = b_minutes*6e7 + b_second*1e6 + b_microseconds
        print("/ BEF: #{:d} Tot_Mcs: {:.0f}" .format(tls_cnt, b_total))

        # Assign (test) values for telemetry class objects.  Structure is 9 float 64s (ID, #, timestamp, xyz, RPY)
        telemetry_carla.cid = GRP_ID
        telemetry_carla.no = tls_cnt
        telemetry_carla.totmcs = b_total
        telemetry_carla.posX = motionSim_carla.posX
        telemetry_carla.posY = motionSim_carla.posY
        telemetry_carla.posZ = motionSim_carla.posZ
        telemetry_carla.rotX = motionSim_carla.rollPos
        telemetry_carla.rotY = motionSim_carla.pitchPos
        telemetry_carla.rotZ = motionSim_carla.yawPos

        # Create a structure for the Carla telemetry data - all float 64.
        carla_tl_double = np.array([var_tl.cid, var_tl.no, var_tl.totmcs, var_tl.posX, var_tl.posY, var_tl.posZ, var_tl.rotX, var_tl.rotY, var_tl.rotZ], dtype=np.float64)

        # Pack the structured doubles into a binary string.
        packed_data_tl = struct.pack('d' * len(carla_tl_double), *carla_tl_double)
        
        # These (global) values are required by the receiver.
        global tlr_socklen 
        tlr_socklen = len(packed_data_tl)
        global tlr_double
        tlr_double = len(carla_tl_double)

        # Send the packed data for telemetry exchange.
        if tele_send == 1:
            tls_sock.sendto(packed_data_tl, (MCAST_GRP, MCAST_PORT))

        # Sleep to maintain 100 Hz. communication with multicast group.
        time.sleep(slp_tls)

        # Increment the TLs iterator.
        tls_cnt += 1

        # Print a separator.
        print("")

    # Close socket.
    tls_sock.close()
    print("/ TL send thread done.  Socket closed.")

def tl_receiver():
    # Setup Telemetry socket (receive).
    tlr_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    tlr_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind the receive socket.
    tlr_sock.bind((NET_IP, MCAST_PORT))
    tlr_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(NET_IP))
    tlr_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(NET_IP))

    # Set a max wait time (seconds) for blocking recvfrom
    tlr_sock.settimeout(rec_to)

    # Timing parameters.
    tlr_start_time = time.time()
    tlr_end_time = tlr_start_time + sim_dur

    # TLr global iterator
    tlr_cnt = 0   

    while time.time() < tlr_end_time:
        # Print the TLr iteration.
        print("# TLr Iteration: #{:d} TLr SimTime: {:.3f} sec." .format(tlr_cnt, (time.time()-tlr_start_time)))

        # Receive (and decode) packed telemetry data from telemetry multicast.  Note: we only care about *other* client data.  (Not ours).
        if tele_recv == 1 and tlr_socklen != 0:
            try:
                data, addr = tlr_sock.recvfrom(tlr_socklen)

                # Decode (received) telemetry data from other machines.
                cars.cID, cars.cNO, cars.cTOT, cars.cPX, cars.cPY, cars.cPZ, cars.cRX, cars.cRY, cars.cRZ = struct.unpack('d' * tlr_double, data)

                # If datagram was received - print packet and latency details.
                if cars.cID != GRP_ID:
                    print(f"     * RCV socket: {cars.cID:.0f} {cars.cNO:.0f} {cars.cTOT:.0f} {cars.cPX:.2f} {cars.cPY:.2f} {cars.cPZ:.2f} {cars.cRX:.2f} {cars.cRY:.2f} {cars.cRZ:.2f}")
                    print("     * Received message from {} ".format(addr))

                    # TMP - induce a receive delay to check timing.
                    time.sleep(slp_rec_tmp)

                    # Get current (after) timestamp.  Calculate (and print) the time delta.
                    now = datetime.now()
                    a_minutes = float(now.minute)
                    a_second = float(now.second)
                    a_microseconds = float(now.microsecond)
                    a_total = a_minutes*6e7 + a_second*1e6 + a_microseconds
                    print("# AFT: #{:.0f} Tot_Mcs: {:.0f} DELTA: {:.6f} sec." .format(cars.cNO, a_total, (a_total-cars.cTOT)/1.e6))
                else:
                    print(" ***this is MY vehicle data***") 

                # Increment the TLr iterator.
                tlr_cnt += 1

                # Print a separator.
                print("")
        
            except socket.timeout:
                print("Timeout occurred: %d seconds" %rec_to)

    # Close socket.
    tlr_sock.close()
    print("# TL receive thread done.  Socket closed.")

def main():

    print("1. Initialize threads:")
    mb_sender_thread = threading.Thread(target=mb_sender)
    tl_sender_thread = threading.Thread(target=tl_sender)
    tl_receiver_thread = threading.Thread(target=tl_receiver)

    print("2. Start threads:")
    mb_sender_thread.start()
    tl_sender_thread.start()
    tl_receiver_thread.start()

    print("3. Join threads:")
    mb_sender_thread.join()
    tl_sender_thread.join()
    tl_receiver_thread.join()

if __name__ == '__main__':

    main()