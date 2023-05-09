import socket
import struct
import time

NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_FRAMEOFDATA = 7

FLOATVALUE = struct.Struct("<f")
SHORT = struct.Struct("<h")
VECTOR3 = struct.Struct("<fff")
VECTOR4 = struct.Struct("<ffff")
FP_CAL_MATRIX_ROW = struct.Struct("<ffffffffffff")
FP_CORNERS = struct.Struct("<ffffffffffff")


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
        model["markers"] = pos

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
    offset, result["markers"] = extract_marker_data(data, offset)
    offset, result["rigid_bodies"] = extract_rigid_body_data(data, offset)

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
    result = {}
    name, _, _ = bytes(data[offset:]).partition(b"\0")
    offset += len(name) + 1
    result["name"] = name

    model_id = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4
    result["id"] = model_id

    parent_id = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4
    result["parent_id"] = parent_id

    pos_offsets = VECTOR3.unpack(data[offset : offset + 12])
    offset += 12
    result["pos_offsets"] = pos_offsets

    num_markers = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    result["markers"] = [{} for _ in range(num_markers)]

    for i in range(num_markers):
        marker_offset = VECTOR3.unpack(data[offset : offset + 12])
        offset += 12
        result["markers"][i]["offset"] = marker_offset

    for i in range(num_markers):
        label = int.from_bytes(data[offset : offset + 4], byteorder="little")
        offset += 4
        result["markers"][i]["label"] = label

    for i in range(num_markers):
        marker_name, _, _ = bytes(data[offset:]).partition(b"\0")
        offset += len(marker_name) + 1
        result["markers"][i]["name"] = marker_name

    return offset, result


def extract_marker_set_definition(data, offset):
    name, _, _ = bytes(data[offset:]).partition(b"\0")
    offset += len(name) + 1

    num_markers = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    for _ in range(num_markers):
        marker_name, _, _ = bytes(data[offset:]).partition(b"\0")
        offset += len(marker_name) + 1

    return offset, None


def extract_skeleton_definition(data, offset):
    name, _, _ = bytes(data[offset:]).partition(b"\0")
    offset += len(name) + 1

    skeleton_id = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    num_bodies = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    for _ in range(num_bodies):
        offset, rigid_body = extract_rigid_body_definition(offset, data)

    return offset, None


def extract_force_plate_definition(data, offset):
    plate_id = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    serial, _, _ = bytes(data[offset:]).partition(b"\0")
    offset += len(serial) + 1

    width = FLOATVALUE.unpack(data[offset : offset + 4])
    offset += 4
    length = FLOATVALUE.unpack(data[offset : offset + 4])
    offset += 4

    origin = VECTOR3.unpack(data[offset : offset + 12])
    offset += 12

    matrix_size = 12

    for _ in range(matrix_size):
        row = FP_CAL_MATRIX_ROW.unpack(data[offset : offset + 48])
        offset += 48

    corners = FP_CORNERS.unpack(data[offset : offset + 48])
    offset += 48

    plate_type = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    channel_type = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    num_channels = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    for _ in range(num_channels):
        channel_name, _, _ = bytes(data[offset:]).partition(b"\0")
        offset += len(channel_name) + 1

    return offset, None


def extract_device_definition(data, offset):
    device_id = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    name, _, _ = bytes(data[offset:]).partition(b"\0")
    offset += len(name) + 1

    serial, _, _ = bytes(data[offset:]).partition(b"\0")
    offset += len(serial) + 1

    device_type = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    data_type = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    num_channels = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    for _ in range(num_channels):
        channel_name, _, _ = bytes(data[offset:]).partition(b"\0")
        offset += len(channel_name) + 1

    return offset, None


def extract_camera_definition(data, offset):
    name, _, _ = bytes(data[offset:]).partition(b"\0")
    offset += len(name) + 1

    position = VECTOR3.unpack(data[offset : offset + 12])
    offset += 12

    orientation = VECTOR4.unpack(data[offset : offset + 16])
    offset += 16

    return offset, None


def unpack_model_definition(data, offset):
    rigid_bodies = []
    num_datasets = int.from_bytes(data[offset : offset + 4], byteorder="little")
    offset += 4

    for _ in range(num_datasets):
        data_type = int.from_bytes(data[offset : offset + 4], byteorder="little")
        offset += 4

        if data_type == 0:
            # Marker set
            offset, _ = extract_marker_set_definition(data, offset)
        elif data_type == 1:
            # Rigid body
            offset, body = extract_rigid_body_definition(data, offset)
            rigid_bodies.append(body)
        elif data_type == 2:
            # Skeleton
            offset, _ = extract_skeleton_definition(data, offset)
        elif data_type == 3:
            # Force plate
            offset, _ = extract_force_plate_definition(data, offset)
        elif data_type == 4:
            # Device
            offset, _ = extract_device_definition(data, offset)
        elif data_type == 5:
            # Camera
            offset, _ = extract_camera_definition(data, offset)
        else:
            raise RuntimeError("unknown model data type")

    return rigid_bodies


def get_rigid_bodies(socket, address, port, timeout=5.0):
    request_model_definition(command_socket, "127.0.0.1", 1510)
    start_time = time.monotonic()
    while time.monotonic() <= start_time + timeout:
        response = receive_data(command_socket)
        if len(response) == 0:
            continue
        offset = 0
        offset, msg_id = get_message_id(response, offset)
        offset, packet_size = get_packet_size(response, offset)
        if msg_id != NAT_MODELDEF:
            continue
        return unpack_model_definition(response, offset)

    raise RuntimeError("rigid bodies request timed out")


command_socket = create_command_socket()

rigid_bodies_map = {}
data_socket = create_data_socket("239.255.42.99", "127.0.0.1", 1511)

last_update = time.monotonic()

while True:
    if not rigid_bodies_map or time.monotonic() > last_update + 1.0:
        rigid_bodies = get_rigid_bodies(command_socket, "127.0.0.1", 1510)
        rigid_bodies_map = {body["id"]: body["name"].decode() for body in rigid_bodies}
        last_update = time.monotonic()

    data = receive_data(data_socket)
    timestamp = time.time_ns()

    if len(data) > 0:
        offset = 0
        offset, msg_id = get_message_id(data, offset)
        offset, _ = get_packet_size(data, offset)

        if msg_id == NAT_FRAMEOFDATA:
            frame_info = unpack_frame_data(data, offset)
            # TODO: generate flatbuffers

    time.sleep(0.001)
