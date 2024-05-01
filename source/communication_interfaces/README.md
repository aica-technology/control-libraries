# Communication interfaces

A library for simple socket communication. It currently implements sockets for UPD, TCP, and ZMQ communication.

## Socket interface

The `ISocket` class is an interface for simple socket communication, defining functions for opening a socket,
sending and receiving bytes, and closing the socket connection.

The `ISocket` class defines an `open()` method to perform configuration steps to open the socket for communication.
If opening the socket fails, an exception is thrown. The `close()` method is also provided to perform steps to disconnect
and close the socket communication.

The functions `receive_bytes(std::string&)` and `send_bytes(const std::string&)` perform the read and write logic of the socket
respectively.

### Implementing a derived socket class

To use this class, create a subclass that inherits from it and implement its pure virtual functions. The pure virtual
functions are `on_open()`, `on_receive_bytes(std::string&)`, and `on_send_bytes(const std::string&)`.

Configuration parameters should be passed with a configuration struct, resulting in a single argument constructor.

The `on_close()` function can optionally be overridden to perform steps to disconnect and close the socket communication.
If a derived class defines any cleanup behavior in `on_close()`, it should also be invoked statically and explicitly
in the destructor of the derived class.

An example is given below.

```c++
// DerivedSocket.hpp

struct DerivedSocketConfig {
  int param1;
  double param2;
};

class DerivedSocket : ISocket {
public:
  DerivedSocket(DerivedSocketConfig configuration);
  
  ~DerivedSocket() override;
  
private:
  void on_open() override;
  
  bool on_receive_bytes(std::string& buffer) override;

  bool on_send_bytes(const std::string& buffer) override;
  
  void on_close() override;
}
```

```c++
// DerivedSocket.cpp
DerivedSocket::DerivedSocket(DerivedSocketConfig configuraiton) {
  // save configuration parameters for later use
}

DerivedSocket::~DerivedSocket() {
  DerivedSocket::on_close();
}

void DerivedSocket::on_open() {
  // Configure and open the socket
}

bool DerivedSocket::on_receive_bytes(std::string& buffer) {
  // Read the contents of the socket into the buffer and return true on success. Otherwise, return false.
  return true;
}

bool DerivedSocket::on_send_bytes(const std::string& buffer) {
  // Write the contents of the buffer onto the socket and return true on success. Otherwise, return false.
  return true;
}

void DerivedSocket::on_close() {
  // Perform clean-up steps here
}
```

## Notes on ZMQ communication

It can be difficult to set up a working configuration for ZMQ communication. The examples below assume that there are
two ZMQ sockets, one that has the robot state is on port 1601 and the command on 1602:

#### Everything runs in one container / on the same host (network independent)

If all applications run in the same container, or on the same host, the situation is:

- The robot publishes its state on `0.0.0.0:1601` and listens to commands on `0.0.0.0:1602` with both sockets
  non-binding.
- The controller sends the command on `*:1602` and receives the state on `*:1601` with both sockets binding.

#### One or more containers and host, all on host network and with no hostname

Same as above.

#### Several containers, user-defined bridge network with hostnames

If the containers all run on a user-defined bridge network, the connecting sockets need to be provided with the hostname
of the binding sockets. For example, if the containers are running on network *aicanet* and have hostnames *robot* and
*controller*, respectively.

- The robot publishes its state on `controller.aicanet:1601` and listens to commands on `controller.aicanet:1602` with
  both sockets non-binding.
- The controller sends the command on `*:1602` and receives the state on `*:1601` with both sockets binding.

#### Note

- This list of combinations is not exhaustive.
- The binding sockets always have a URI like `*:port` whilst the connecting sockets need to provide the complete address
  version (`0.0.0.0:port` if on localhost or `hostname.network:port` if on bridge network).
