# Python Bindings

This directory defines Python bindings for the control libraries.

## Structure

[PyBind11](https://PyBind11.readthedocs.io/en/stable/index.html) is used to generate
Python bindings for the classes and functions in control libraries.

The generated package is named `control-libraries`, but contains specific modules for importing.
These are named the same as the standard modules of control libraries (e.g. `state_representation`).

The contents of the [`source`](./source) directory define the bindings between
each Python module and the respective C++ library. The source files to bind each module are
contained within a subdirectory of the same name.

The `setup.py` and `pyproject.toml` files are used to configure the build and installation
of the Python bindings. The `.toml` file allows `pip` to automatically fetch the 
installation dependencies (namely `setuptools` and `pybind11`) in a temporary cache,
allowing the subsequent `setup.py` to be evaluated without needing a local installation of `pybind11`.
This feature requires a [`pip`](https://pypi.org/project/pip/) version 10.0 or newer.

The [`test`](./test) directory contains some Python scripts that import and check the bindings
using the Python `unittest` and `pytest` frameworks. They are not currently comprehensive.

## Usage

You can import the modules with an optional short alias:

```python
#!/usr/bin/env python
import state_representation as sr
import dynamical_systems

print(sr.__version__)
print(dynamical_systems.__version__)

A = sr.CartesianState().Random("A")
print(A)
```

Or, directly import specific classes from the module.

```python
#!/usr/bin/env python
from state_representation import JointState
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE

B = JointState().Random("B", 3)
ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
```

The `clproto` Python module can be used to encode and decode objects into bytes of serialized data.

```python
#!/usr/bin/env python
from state_representation import JointState
import clproto

B = JointState().Random("B", 3)
encoded_msg = clproto.encode(B, clproto.MessageType.JOINT_STATE_MESSAGE)

decoded_object = clproto.decode(encoded_msg)
```

### Note on the communication interfaces

The Python bindings require an additional step of sanitizing the data when sending and receiving bytes. To illustrate
this, an example is provided here.

```python
# First a server and a client is connected
context = ZMQContext()
server = ZMQPublisher(ZMQSocketConfiguration(context, "127.0.0.1", "5001", True))
client = ZMQSubscriber(ZMQSocketConfiguration(context, "127.0.0.1", "5001", False))
server.open()
client.open()
```

```python
# Then a string is sent through
str_msg = "Hello!"
server.send_bytes(str_msg)
received_str_msg = client.receive_bytes()
if received_str_msg is not None:
    print(received_str_msg)
```

Here we expect the printed value to be `Hello!`, however due to the way strings and bytes are processed, the string
message is left as a byte literal and `b'Hello!'` is printed instead. We can correct this as follows:

```python
# Instead, decode the value
print(received_str_msg.decode("utf-8")) # will print Hello! as expected
```
