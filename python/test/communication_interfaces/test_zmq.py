import pytest
import time

from communication_interfaces.exceptions import SocketConfigurationError
from communication_interfaces.sockets import ZMQContext, ZMQSocketConfiguration, ZMQPublisher, ZMQSubscriber, \
    ZMQCombinedSocketsConfiguration, ZMQPublisherSubscriber


@pytest.fixture
def zmq_context():
    return ZMQContext()


@pytest.fixture
def zmq_config(zmq_context):
    return ZMQSocketConfiguration(zmq_context, "127.0.0.1", "4000")


def test_send_receive(zmq_config):
    send_string = "Hello world"

    publisher = ZMQPublisher(zmq_config)
    zmq_config.bind = False
    subscriber = ZMQSubscriber(zmq_config)

    publisher.open()
    subscriber.open()

    response = subscriber.receive_bytes()
    while response is None:
        assert publisher.send_bytes(send_string)
        response = subscriber.receive_bytes()
        time.sleep(0.01)

    assert response
    assert response.decode("utf-8") == send_string

    publisher.close()
    subscriber.close()


def test_send_receive_combined(zmq_context):
    server_send_string = "Hello client"
    client_send_string = "Hello server"

    server_config = ZMQCombinedSocketsConfiguration(
        zmq_context, "127.0.0.1", "5001", "5002")
    server = ZMQPublisherSubscriber(server_config)

    client_config = ZMQCombinedSocketsConfiguration(
        zmq_context, "127.0.0.1", "5002", "5001", False, False)
    client = ZMQPublisherSubscriber(client_config)

    server.open()
    client.open()

    response = client.receive_bytes()
    while response is None:
        assert server.send_bytes(server_send_string)
        response = client.receive_bytes()
        time.sleep(0.01)

    assert response.decode("utf-8") == server_send_string

    response = server.receive_bytes()
    while response is None:
        assert client.send_bytes(client_send_string)
        response = server.receive_bytes()
        time.sleep(0.01)

    assert response.decode("utf-8") == client_send_string

    server.close()
    client.close()


def test_open_close(zmq_config):
    buffer = ""
    socket = ZMQPublisher(zmq_config)
    with pytest.raises(SocketConfigurationError):
        socket.send_bytes(buffer)
    with pytest.raises(SocketConfigurationError):
        socket.receive_bytes()

    socket.open()
    assert socket.send_bytes("test")
    socket.close()
    with pytest.raises(SocketConfigurationError):
        socket.send_bytes(buffer)
