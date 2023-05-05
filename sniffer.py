import socket
import struct

NAT_FRAMEOFDATA = 7


VECTOR3 = struct.Struct("<fff")
VECTOR4 = struct.Struct("<ffff")


def create_data_socket(multicast_address, local_address, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(
        socket.IPPROTO_IP,
        socket.IP_ADD_MEMBERSHIP,
        socket.inet_aton(multicast_address) + socket.inet_aton(local_address),
    )

    sock.bind((local_address, port))
    return sock


def receive_data(data_socket):
    buffer_size = 64 * 1024

    data, _ = data_socket.recvfrom(buffer_size)

    return data


def get_message_id(data):
    return int.from_bytes(data[0:2], byteorder="little")


def get_packet_size(data):
    return int.from_bytes(data[2:4], byteorder="little")


def extract_frame_number(data, offset):
    return offset + 4, int.from_bytes(memory[offset : offset + 4], byteorder="little")


def extract_marker_data(data, offset):
    num_models = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4
    models = [{} for _ in range(num_models)]

    for model in models:
        name, _, _ = bytes(data[offset:]).partition(b"\0")
        offset += len(name) + 1
        model["name"] = name

        num_markers = int.from_bytes(data[offset : offset + 4], byteorder="little")
        offset += 4
        model["markers"] = []

        for _ in range(num_markers):
            pos = VECTOR3.unpack(data[offset : offset + 12])
            offset += 12
            model["markers"].append(pos)

        num_unlabelled = int.from_bytes(data[offset : offset + 4], byteorder="little")
        offset += 4
        model["unlabelled_markers"] = []

        for i in range(num_unlabelled):
            pos = VECTOR3.unpack(data[offset : offset + 12])
            offset += 12
            model["unlabelled_markers"].append(pos)

    return offset, models


def extract_rigid_body_data(data, offset):
    num_bodies = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4
    bodies = [{} for _ in range(num_bodies)]

    for body in bodies:
        body_id = int.from_bytes(data[offset : offset + 4], byteorder="little")
        offset += 4
        body["id"] = body_id

        pos = VECTOR3.unpack(data[offset : offset + 12])
        offset += 12
        body["pos"] = pos

        rot = VECTOR4.unpack(data[offset : offset + 16])
        offset += 16
        body["rot"] = rot

        # TODO: depending on version we might need to unpack marker info here!
    return offset, bodies


def unpack_frame_data(data):
    data = memoryview(data)
    offset = 4
    result = {}

    offset, result["frame number"] = extract_frame_number(data, offset)
    print("frame number:", result["frame number"])

    offset, result["markers"] = extract_marker_data(data, offset)
    print("markers:", result["markers"])

    offset, result["rigid_bodies"] = extract_rigid_body_data(data, offset)
    print("rigid bodies:", result["rigid_bodies"])


data_socket = create_data_socket("239.255.42.99", "127.0.0.1", 1511)
data = receive_data(data_socket)

print(f"data length = {len(data)}")
if len(data) > 0:
    msg_id = get_message_id(data)
    packet_size = get_packet_size(data)
    print(msg_id)
    print(packet_size)

    if msg_id == NAT_FRAMEOFDATA:
        print("data frame received")
    else:
        print("unhandled data type ignored")
