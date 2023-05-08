import socket
import struct

NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_FRAMEOFDATA = 7

FLOATVALUE = struct.Struct("<f")
SHORT = struct.Struct("<h")
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


def get_message_id(data, offset):
    return 2, int.from_bytes(data[0:2], byteorder="little")


def get_packet_size(data, offset):
    return offset + 2, int.from_bytes(data[offset : offset + 2], byteorder="little")


def extract_frame_number(data, offset):
    return offset + 4, int.from_bytes(data[offset : offset + 4], byteorder="little")


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

    num_models = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4
    unlabelled_models = [{} for _ in range(num_models)]

    for model in unlabelled_models:
        pos = VECTOR3.unpack(data[offset : offset + 12])
        offset += 12
        model["markers"].append(pos)

    return offset, {"labelled_models": models, "unlabelled_models": unlabelled_models}


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

        marker_error = FLOATVALUE.unpack(data[offset : offset + 4])[0]
        offset += 4
        body["error"] = marker_error

        raw_valid = SHORT.unpack(data[offset : offset + 2])[0]
        offset += 2
        body["valid"] = (raw_valid & 0x01) == 1

    return offset, bodies


def unpack_frame_data(data, offset):
    data = memoryview(data)
    result = {}

    offset, result["frame number"] = extract_frame_number(data, offset)
    print("frame number:", result["frame number"])

    offset, result["markers"] = extract_marker_data(data, offset)
    print("markers:", result["markers"])

    offset, result["rigid_bodies"] = extract_rigid_body_data(data, offset)
    print("rigid bodies:", result["rigid_bodies"])
    
    return offset, result


def create_command_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", 0))
    return sock


def send_command(sock, command, command_str, address):
    data = command.to_bytes(2, byteorder="little")
    data += (len(command_str) + 1).to_bytes(2, byteorder="little")
    data += command_str.encode("utf-8") + b"\0"

    sock.sendto(data, address)


def request_server_info(sock, address, port):
    send_command(sock, NAT_CONNECT, "Ping", (address, port))


def request_model_definition(sock, address, port):
    send_command(sock, NAT_REQUEST_MODELDEF, "", (address, port))


def extract_rigid_body_definition(data, offset):
    name, _, _ = bytes(data[offset:]).partition(b"\0")
    offset += len(name) + 1
    print(name)

    model_id = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4
    print(model_id)


def unpack_model_definition(data, offset):
    num_datasets = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4
    print(num_datasets)
    for _ in range(num_datasets):
        data_type = int.from_bytes(data[offset : offset + 4], byteorder="little")
        offset += 4

        if data_type == 0:
            # Marker set
            pass
        elif data_type == 1:
            # Rigid body
            extract_rigid_body_definition(data, offset)


command_socket = create_command_socket()
request_model_definition(command_socket, "127.0.0.1", 1510)
response = receive_data(command_socket)

if len(response) > 0:
    offset = 0
    offset, msg_id = get_message_id(response, offset)
    offset, packet_size = get_packet_size(response, offset)
    if msg_id == NAT_SERVERINFO:
        print("server info received")
    elif msg_id == NAT_MODELDEF:
        print("model definition received")
        unpack_model_definition(response, offset)


data_socket = create_data_socket("239.255.42.99", "127.0.0.1", 1511)
data = receive_data(data_socket)

print(f"data length = {len(data)}")
if len(data) > 0:
    offset = 0
    offset, msg_id = get_message_id(data, offset)
    offset, packet_size = get_packet_size(data, offset)

    if msg_id == NAT_FRAMEOFDATA:
        print("data frame received")
        unpack_frame_data(data, offset)
    else:
        print("unhandled data type ignored")
