import pytest

from communication_interfaces.sockets import UDPSocketConfiguration, UDPClient, UDPServer
from communication_interfaces.exceptions import SocketConfigurationError


@pytest.fixture
def udp_config():
    return UDPSocketConfiguration("127.0.0.1", 5000, 100)


@pytest.fixture
def server(udp_config):
    socket = UDPServer(udp_config)
    yield socket
    socket.close()


@pytest.fixture
def client(udp_config):
    socket = UDPClient(udp_config)
    yield socket
    socket.close()


def test_send_receive(server, client):
    send_string = "Hello world"

    try:
        server.open()
    except SocketConfigurationError as e:
        pytest.fail(e)

    try:
        client.open()
    except SocketConfigurationError as e:
        pytest.fail(e)

    assert client.send_bytes(send_string)
    response = server.receive_bytes()
    assert response
    assert response.decode("utf-8").rstrip("\x00") == send_string


def test_timeout(udp_config):
    udp_config.timeout_duration_sec = 2.0

    server = UDPServer(udp_config)
    server.open()

    assert server.receive_bytes() is None
    server.close()


def test_port_reuse(udp_config, server):
    server.open()

    server2 = UDPServer(udp_config)
    with pytest.raises(Exception):
        server2.open()


def test_open_close(udp_config):
    buffer = ""
    udp_config.timeout_duration_sec = 0.5
    server = UDPServer(udp_config)
    with pytest.raises(SocketConfigurationError):
        server.send_bytes(buffer)
    with pytest.raises(SocketConfigurationError):
        server.receive_bytes()
    server.open()
    assert server.send_bytes("test")
    assert not server.receive_bytes()

    server.close()
    with pytest.raises(SocketConfigurationError):
        server.send_bytes(buffer)
    with pytest.raises(SocketConfigurationError):
        server.receive_bytes()

    client = UDPClient(udp_config)
    with pytest.raises(SocketConfigurationError):
        client.send_bytes(buffer)
    with pytest.raises(SocketConfigurationError):
        client.receive_bytes()
    client.open()
    assert client.send_bytes("test")
    assert not client.receive_bytes()

    client.close()
    with pytest.raises(SocketConfigurationError):
        client.send_bytes(buffer)
    with pytest.raises(SocketConfigurationError):
        client.receive_bytes()
