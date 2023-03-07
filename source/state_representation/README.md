# State Representation

This library provides a set of classes to represent **states**, which are specific data points (state variables)
associated with a name.

In robotics, the most useful types of state refer to spatial properties. The joint angles of a robot arm, the 3D
position and velocity of a flying drone, or the measurement of an accelerometer or force-torque sensor are all examples
of instantaneous state variables. The [`CartesianState`](#cartesian-state) and [`JointState`](#joint-state) classes
can represent such values in an internally consistent way while providing useful functions for conversions,
transformations and other manipulations.

State classes are designed to be extensible to support more abstract state variables or properties. Another type of
state is a **parameter**, which this library defines as a container class to for generic named variables.

The following sections describe the properties of the main state classes in the library.

## Table of contents:

* [State](#state)
    * [Name](#name)
    * [State Type](#state-type)
    * [Timestamp](#timestamp)
    * [Emptiness](#emptiness)
* [Cartesian state](#cartesian-state)
    * [Reference frame](#reference-frames)
    * [Construction](#construction)
    * [Cartesian getters and setters](#cartesian-getters-and-setters)
    * [Cartesian addition and subtraction](#cartesian-addition-and-subtraction)
    * [Cartesian transforms: changing the reference frame](#cartesian-transforms--changing-the-reference-frame)
    * [Cartesian distances and norms](#cartesian-distances-and-norms)
* [Derived Cartesian classes](#derived-cartesian-classes)
    * [Cartesian pose](#cartesian-pose)
    * [Cartesian twist](#cartesian-twist)
    * [Cartesian acceleration](#cartesian-acceleration)
    * [Cartesian wrench](#cartesian-wrench)
* [Joint state](#joint-state)
    * [Joint state operations](#joint-state-operations)
    * [Conversion between joint state variables](#conversion-between-joint-state-variables)
* [The Jacobian matrix](#the-jacobian-matrix)
    * [Conversion between JointVelocities and CartesianTwist](#conversion-between-jointvelocities-and-cartesiantwist)
    * [Conversion between JointTorques and CartesianWrench](#conversion-between-jointtorques-and-cartesianwrench)
    * [Matrix multiplication](#matrix-multiplication)
    * [Changing the Jacobian reference frame](#changing-the-jacobian-reference-frame)

## State

The `State` base class defines the following attributes common to all derived states:

- name: the name associated with the state
- type: the specific StateType of the object
- timestamp: an internal timestamp which refers to the last modification

### Name

The name is used to label a specific state instance, and can be used to disambiguate multiple states or check their
compatibility. The name can be accessed or modified with `get_name()` and `set_name()` respectively.

### State Type

A state can hold different data depending on its type. The available state types are defined by the
`state_representation::StateType` enumeration. The type field is accessible using the `get_type()` method and allows
introspection when working with state pointers. It is a readonly property, determined by the constructor implementation.

```c++
void check_state_type(const std::shared_ptr<state_representation::State>& state) {
  if (state->get_type() == ...) {
    // 
  }
}
```

### Timestamp

The timestamp of a state records when it was last modified. Any non-const operations or mutations,
including copy constructions, will reset the timestamp.

`get_age()` returns the time since last modification in seconds, while `is_deprecated(time_delay)` can be used to
check the age of a state against a maximum time delay.

### Emptiness

In addition, all states have a concept of "emptiness". The `State` base class does not refer to any specific state
variable, and therefore holds no data. Derived classes that implement state variables do hold data. If a state is empty,
it indicates that the data is uninitialized or otherwise invalid. The `is_empty()` method can be used to check
emptiness, or otherwise the state object can be evaluated directly.

```c++
state_representation::State state;

if (state) {
  // the state is not empty
} else {
  // the state is empty
}
```

A state can be marked as "empty" by calling `reset()`. The state remains empty until any data is set.

## Cartesian state

A `CartesianState` represents a spatial frame in 3D space, containing the following spatial and dynamic properties:

- `position`
- `orientation`
- `linear_velocity`
- `angular_velocity`
- `linear_acceleration`
- `angular_acceleration`
- `force`
- `torque`

Each state variable is represented as a 3D vector (`Eigen::Vector3d`), except for the orientation, which is represented
by a unit quaternion (`Eigen::Quaterniond`). The values are assumed to be in standard SI units (meters, radians,
seconds, and Newtons).

The state variables can also be considered in pairs:

- `pose` (`position` and `orientation`)
- `twist` (`linear_velocity` and `angular_velocity`)
- `accelerations` (`linear_acceleration` and `angular_acceleration`)
- `wrench` (`force` and `torque`)

The linear terms always come first, followed by the angular terms.

Each _paired_ state variable is represented as a 6D vector (`Eigen::VectorXd(6)`), except for the pose, which is
represented as a 7D vector (3 for `position` and 4 for `orientation`).

### Reference frames

The spatial properties are expressed relative to a named reference frame. For a `CartesianState` with name "A" and
reference frame "B", each state variable represents the instantaneous spatial property measured at or around frame A
from the perspective of frame B.

In some contexts, `twist` or `wrench` vectors may be interpreted differently. For example, there is a concept of
"body twist" and "spatial twist". See the sections on [`CartesianTwist`](#cartesian-twist) and
[`CartesianWrench`](#cartesian-wrench) for more details.

### Construction

`CartesianState` constructors take a name and an optional reference frame; by default, the reference frame is "world".

Constructing a state without any data results in an empty state. To set initial data, the static constructors
`Identity()` or `Random()` can be used. The former sets all vectors to zero and sets the orientation to the null
quaternion. The latter sets all state variables to a unit random state within a uniform distribution.

```c++
state_representation::CartesianState s1("A"); // frame A expressed in world (default)
state_representation::CartesianState s2("B", "A"); // frame B expressed in A

auto s3 = state_representation::CartesianState::Identity("I"); // identity frame I expressed in world
auto s4 = state_representation::CartesianState::Random("R", "B"); // random frame R expressed in B
```

### Cartesian getters and setters

Each state variable has a corresponding getter and setter to access or modify the data after construction.

```c++
auto state = state_representation::CartesianState::Random("A");

// get the position as a 3D vector
Eigen::Vector3d xyz = state.get_position();

// set the position with a 3D vector
xyz = Eigen::Vector3d(1, 0, 0); // 1 meter in X
state.set_position(xyz);

// or, set X, Y, Z values directly
state.set_position(0, 0.001, 0); // 1 millimeter in Y
```

The orientation operates with a quaternion instead of a 3D vector.

```c++
// get the orientation as a unit quaternion
Eigen::Quaterniond q = state.get_orientation();

// set the orientation with a quaternion object
q = Eigen::Quaterniond(0, 1, 0, 0); // 180º rotation around X
state.set_orientation(q);

// or, set W, X, Y, Z values directly
state.set_orientation(0, 0, 0, 1); // 180º rotation around Z
```

Whenever setting the orientation, the values are automatically normalized to ensure a unit quaternion.

```c++
s2.set_orientation(1, 1, 0, 0); // 90º rotation around X
s2.get_orientation(); // Eigen::Quaterniond(0.70710678, 0.70710678, 0., 0.)
```

Every other 3D state variable has equivalent getters and setters to the position:

- `get_linear_velocity()`, `set_linear_velocity(xyz)` or `set_linear_velocity(x, y, z)` in meters per second
- `get_angular_velocity()`, `set_angular_velocity(xyz)` or `set_angular_velocity(x, y, z)` in radians per second
- `get_linear_acceleration()`, `set_linear_acceleration(xyz)` or `set_linear_acceleration(x, y, z)` in meters per second
  squared
- `get_angular_acceleration()`, `set_angular_acceleration(xyz)` or `set_angular_acceleration(x, y, z)` in radians per
  second squared
- `get_force()`, `set_force(xyz)` or `set_force(x, y, z)` in Newtons
- `get_torque()`, `set_torque(xyz)` or `set_torque(x, y, z)` in Newton-meters

The paired state variables (`pose`, `twist`, `acceleration`, `wrench`) also have their own getters and setters to
modify the underlying terms in one operation.

```c++
Eigen::VectorXd pose = state.get_pose(); // 7D vector of position and orientation
pose = {x, y, z, qw, qx, qy, qz};
s2.set_pose(pose); // update the position and orientation

Eigen::VectorXd twist = state.get_twist(); // 6D vector of linear and angular velocity
state.set_twist(Eigen::VectorXd::Random(6)); // update the linear and angular velocity

Eigen::VectorXd acceleration = state.get_acceleration(); // 6D vector of linear and angular acceleration
state.set_acceleration(Eigen::VectorXd::Random(6)); // update the linear and angular acceleration

Eigen::VectorXd wrench = state.get_wrench(); // 6D vector of force and torque
state.set_wrench(Eigen::VectorXd::Random(6)); // update the force and torque
```

### Cartesian addition and subtraction

In robotics and control, a state variable can represent a command or desired value, rather than a real measurement.
For this reason, it's often desirable to combine and manipulate state variables in different ways.

For example, a simple controller might be driving the linear velocity of a robot to approach a moving object. Then,
the desired velocity of the robot would be the velocity of the object _plus_ some additional velocity in the direction
of the object.

If two `CartesianState` objects can be combined with addition or subtraction, **provided they are expressed in the same
reference frame**.

```c++
auto s1 = state_representation::CartesianState::Random("a"); // reference frame is world by default
auto s2 = state_representation::CartesianState::Random("b");

auto sum = s1 + s2;
auto diff = s1 - s2;

// if the states are not in the same reference frame, it will raise an exception
s1 + state_representation::CartesianState::Random("c", "other");
```

For all state variables except orientation, the result of the operation is applied to each state variable element-wise.
For example, If `s1` has position `(x1, y1, z1)` and `s2` has position `(x1, y1, z1)`, then `s1 + s2` has position
`(x1 + x2, y1 + y2, z1 + z2)`. The same applies for subtraction and is true for all state variables represented as 3D
vectors.

In the case of orientation, the addition and subtraction operations are **not commutative**.

Addition of orientation uses the quaternion product; the addition `s1 + s2` corresponds to the rotation of `s1` followed
by the rotation in `s2`, while `s2 + s1` corresponds to the rotation of `s2` followed by the rotation of `s1`.

Subtraction of orientation uses the quaternion inverse (quaternion conjugate); `-s1` yields the inverse orientation of
`s1`, and `s1 - s2` is equivalent to the rotation of `s1` followed by the inverse rotation of `s2`.

### Cartesian transforms: changing the reference frame

A `CartesianState` represents the spatial properties of a frame as viewed from a certain reference frame.
Observing the same frame from a different reference frame can change the relative values of each state variable.

If two observers are looking at the same frame from different locations, they will observe the frame with a different
relative position and orientation. Similarly, if one observer is moving while the other is stationary, they will observe
the frame with different relative velocities.

If some frame A is observed from reference frame B, and frame B is itself observed from another reference frame C,
then it is possible to express frame A relative to reference frame C. Expressing the spatial properties of a frame in
different reference frames is known as a **transformation** and is one of the most useful operations of the
`CartesianState` class.

A transformation affects all state variables as a combination of the two connected frames.
In the simplest case, the relative distance between each frame is combined to yield a new position value.
In another example, if frame A is rotated by 45 degrees relative to B and B is rotated 45 degrees relative to C, then A
is rotated 90 degrees relative to C.

When states have non-zero position, orientation, velocity or acceleration offsets, the transformation involves a more
complicated combination of the different state variables.

If there is an orientation difference between the frames, the vectors representing each state variable are rotated into
the new reference frame accordingly.

Transformations including twist will include radially-induced linearly velocity that scales with the distance between
frames and their relative angular velocities. Finally, the transformation of accelerating frames will include
Coriolis and centrifugal effects.

Transformation applies only to the relative spatial properties of the frame (pose, twist, and acceleration).
The wrench is not transformed in the same way because forces applied to spatial frames do not "add together" like
relative positions or velocities. A measured force or torque will therefore not change in magnitude when expressed
in different reference frames, and is only rotated to the new reference frame coordinate system.

The concept of a wrench transform does exist in the context of static forces across rigid bodies. This is described
further in the [`CartesianWrench`](#cartesian-wrench) section.

#### The transform operator

For clarity in the examples below, the name given to each `CartesianState` variable will have the reference name
as the prefix and the frame name as a suffix. This will highlight the "chain rule" that occurs during frame
transformations.

```c++
// frame "a" expressed in reference frame "world"
state_representation::CartesianState wSa("a", "world");
// frame "b" expressed in reference frame "a"
state_representation::CartesianState aSb("b", "a");
```

The multiplication operator is used to transform states by chaining together connected frames and reference frames.

```c++
// the state of frame "b" expressed in reference frame "world" is found by
// combining frame "a" in "world" with frame "b" in "a"
auto wSb = wSa * aSb;

// the chain rule applies whenever the "inner frames" are compatible
auto wSd = wSb * bSc * cSd;
```

As can be seen in the above examples, the frame of the left state should match the reference frame of the right state,
causing the inner frames to cancel out. This is analogous to the product of two matrices, wherein the inner sizes must
be compatible. If the frame names do not match, the states are not compatible and an exception will be raised.

```c++
// error: inner frames are not compatible!
wSa * bSc;
```

In some cases, there may be enough information to perform a transformation, but the inner frames are not compatible.

```c++
// inner frames are not compatible, but it should still be possible to find "b" relative to "world"
wSa * bSa;
```

To resolve this case, inversion can be used.

#### Inversion

It is possible to invert a state using the `inverse()` operator, which essentially reverses the frame and reference
frame. Instead of observing a frame "A" from a reference frame "B", the inverse expresses "B" as observed from "A".

```c++
aSb = bSa.inverse();
```

In the context of the transformation chain rule, this can be used to resolve otherwise incompatible operations.
This is analogous to transposing a matrix so that the inner sizes of a product match.

```c++
// wSa * bSa is not possible, but wSa * aSb would be
wSb = wSa * bSa.inverse() = wSa * aSb
```

An inversion sets the new orientation as the quaternion conjugate and effectively negates and rotates all other state
variable vectors.

As with transformation, the wrench is the special case. For a state `aSb`, the wrench describes the force and torque
measured at `b` as seen from `a`. For the inverted state `bSa`, the wrench at `a` is unknown without further
underlying assumptions. For this reason, the inverse operator sets the resulting wrench to zero.

### Cartesian distances and norms

As a `CartesianState` represents a spatial transformation, distance between states and norms computations have been
implemented. The distance functions is represented as the sum of the distance over all the state variables:

```c++
using namespace state_representation;
auto cs1 = state_representation::CartesianState::Random("test");
auto cs2 = state_representation::CartesianState::Random("test");

double d = cs1.dist(cs2);
// alternatively one can use the friend type notation
d = state_representation::dist(cs1, cs2)
```

By default, the distance is computed over all the state variables by combining the Euclidean distance between each 
state vector and adding the angular distance in radians in the case of orientation.

Because the distance is summed over independent spatial terms, the final value has no physical units but can still
be useful when comparing the relative similarity of two states.

To find a difference in a specific state variable, the `CartesianStateVariable` enumeration can be used:

```c++
// only the distance in meters
double distance = cs1.dist(cs2, CartesianStateVariable::POSITION);
// only the angular distance in radians
double angle = cs1.dist(cs2, CartesianStateVariable::ORIENTATION);

// only the angular velocity difference in radians per second
double angular_rate = cs1.dist(cs2, CartesianStateVariable::ANGULAR_VELOCITY);
```

The Euclidean norm of individual state variables returns the vector magnitude (or angular displacement in the case of
orientation). To get the magnitude of all state variables as a vector of norms, use the `norms()` operator.

```c++
auto state = state_representation::CartesianState::Random("test");

// the default usage returns the magnitude of all state variables in an 8D vector
std::vector<double> norms = state.norms();

// to return 2D vectors of norms, filter by the combined state variables
std::vector<double> distance_and_angle = state.norms(CartesianStateVariable::POSE);
std::vector<double> speeds = state.norms(CartesianStateVariable::TWIST);

// other single state variables give a 1D vector
std::vector<double> distance = state.norms(CartesianStateVariable::POSITION);
```

Finally, state variables can be scaled to a unit vector state using the `normalize()` / `normalized()` operations. 
This does not affect the orientation, which is always expressed as a unit quaternion.

The former normalizes a state in place, while the latter returns a normalized copy without modifying the original state.

As with the other operations, the normalization can be selectively applied to specific state variables.

```c++
// normalize a state all the state variables
state.normalize();

// normalize the position of a state to a unit direction vector
state.normalize(CartesianStateVariable::POSITION);

// copied state with only linear velocity normalized
CartesianState normalized_state = state.normalized(CartesianStateVariable::LINEAR_VELOCITY);
```

## Derived Cartesian classes

The `CartesianState` class contains all spatial and dynamic state variables of a frame. In some cases, it is convenient
to operate only with specific state variables. The following derived classes are defined:

- `CartesianPose`
- `CartesianTwist`
- `CartesianAcceleration`
- `CartesianWrench`

### Cartesian pose

The `CartesianPose` class defines only the position and orientation of a frame.

It provides the following constructors:

```c++
CartesianPose::Identity("name", "reference_frame");
CartesianPose::Random("name", "reference_frame");

// the position can be supplied to the constructor as a vector or as individual terms (with null orientation)
xyz = Eigen::Vector3d(1.0, 2.0, 3.0);
CartesianPose("name", xyz, "reference_frame");
CartesianPose("name", 1.0, 2.0, 3.0, "reference_frame");

// the orientation can be supplied to the constructor as a vector or as individual terms (with zero displacement)
q = Eigen::Quaterniond(0, 1, 0, 0);
CartesianPose("name", q, "reference_frame");

// the position and orientation can also be supplied together
CartesianPose("name", xyz, q, "reference_frame");
```

Addition and subtraction is supported between `CartesianPose` and `CartesianState`. Recall that these operations are
not commutative in orientation; the order of operations matters.

The return type of each compatible operation is shown below.

```c++

CartesianPose r1 = pose + other_pose;
CartesianState r2 = pose + state;
CartesianState r3 = state + pose;

CartesianPose r4 = pose - other_pose;
CartesianState r5 = pose - state;
CartesianState r6 = state - pose;

pose += other_pose; // equivalent to pose = pose + other_pose
pose += state; // equivalent to pose = pose + state

pose -= other_pose;
pose -= state;
```

A `CartesianPose` can be used to transform any other Cartesian class by applying the position and orientation offset.
Recall that the frame and reference frames must be compatible according to the chain rule as described in the
[Cartesian transformation](#cartesian-transforms--changing-the-reference-frame) section.

```c++
CartesianPose transformed_pose = pose * other_pose;
CartesianTwist transformed_twist = pose * twist;
CartesianAcceleration transformed_acceleration = pose * acceleration;
CartesianWrench transformed_wrench = pose * wrench;
CartesianState transformed_state = pose * state
    
pose *= other_pose; // equivalent to pose = pose * other_pose
pose *= state; // equivalent to pose = pose * state
```

The time derivative of a `CartesianPose` is a `CartesianTwist`. Because `CartesianPose` represents a displacement in
position and orientation, it can be converted into linear and angular velocity through division by a time period.

Operations with time use `std::chrono::duration` types, such as `std::chrono::milliseconds`, `std::chrono::seconds`, or
definitions with `std::literals::chrono_literals`.

```c++
// take a pose with a displacement of 1 meter in the X axis and 90º (π/2) rotation around the Z axis
state_representation::CartesianPose pose("a", Eigen::Vector3d(1, 0, 0), Eigen::Quaterniond(0.707, 0, 0, 0.707));

// define a 2 second time duration
std::chrono::seconds dt(2);

// dividing pose by time yields a twist with a linear velocity of 0.5 meters per second in the X axis
// and an angular velocity of π/4 radians per second (45º/s) around the Z axis
state_representation::CartesianTwist twist = pose / dt;
twist.get_linear_velocity(); // (0.5, 0, 0)
twist.get_angular_velocity(); // (0, 0, 0.785)
```

### Cartesian twist

The `CartesianTwist` class defines only the linear and angular velocity of a frame.

It provides the following constructors:

```c++
CartesianTwist::Zero("name", "reference_frame");
CartesianTwist::Random("name", "reference_frame");

// the linear or angular velocities can be supplied to the constructor as vectors
linear_velocity = Eigen::Vector3d(1.0, 2.0, 3.0);
CartesianTwist("name", linear_velocity, "reference_frame");

angular_velocity = Eigen::Vector3d(4.0, 5.0, 6.0);
CartesianTwist("name", linear_velocity, angular_velocity, "reference_frame");

// the full 6D twist vector can also be supplied
twist = Eigen::VectorXd(6);
twist << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
CartesianTwist("name", twist, "reference_frame");
```

Addition and subtraction is supported between `CartesianTwist` and `CartesianState`.

The return type of each compatible operation is shown below.

```c++

CartesianTwist r1 = twist + other_twist;
CartesianState r2 = twist + state;
CartesianState r3 = state + twist;

CartesianTwist r4 = twist - other_twist;
CartesianState r5 = twist - state;
CartesianState r6 = state - twist;

twist += other_twist; // equivalent to twist = twist + other_twist
twist += state; // equivalent to twist = twist + state

twist -= other_twist;
twist -= state;
```

The time derivative of a `CartesianTwist` is a `CartesianAcceleration`. Because `CartesianTwist` represents linear and
angular velocity in meters and radians per seconds, it can be converted into linear and angular acceleration in meters
and radians per second squared through division by a time period.

Similarly, the time integral of a `CartesianTwist` is a `CartesianPose`. It can be converted into a position and
orientation displacement through multiplication by a time period.

Operations with time use `std::chrono::duration` types, such as `std::chrono::milliseconds`, `std::chrono::seconds`, or
definitions with `std::literals::chrono_literals`.

```c++
// take a twist with a linear velocity of 1 meter per second in the X axis
// and angular velocity of 1 radian per second in the Z axis 
state_representation::CartesianTwist twist("a", Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1));

// define a 0.5 second time duration
std::chrono::milliseconds dt(500);

// dividing twist by time yields an acceleration with a linear term of 2 meters per second squared in the X axis
// and angular term of 2 radians per second squared around the Z axis
state_representation::CartesianAcceleration acc = twist / dt;
acc.get_linear_acceleration(); // (2, 0, 0) 
acc.get_angular_acceleration(); // (0, 0, 2)

// multiplying twist by time yields a pose with a position of 0.5 meters in the X axis
// and a rotation of 0.5 radians around the Z axis
state_representation::CartesianPose pose = twist * dt;
pose.get_position(); // (0.5, 0, 0)
pose.get_orientation(); // Eigen::Quaterniond(0.9689124, 0, 0, 0.247404)
```

Note that the result of the integration is a displacement from a null (identity) pose. To offset the integration,
simply add an initial pose in the same reference frame as the twist.

```c++
// add an initial pose to offset the integration 
pose = initial_pose + twist * dt;

// a pose can also be updated through a continuous integration of twist
pose += twist * dt;
```

#### Representations of twist

<!-- This section uses GitHub markdown syntax for rendering mathematical expressions -->

A `CartesianTwist("B", "A")` represents linear and angular velocity of the body frame B with respect to fixed frame A,
as viewed from A.

$$^A\mathcal{V}_B =
\begin{bmatrix}
^Av_B \\
^A\omega_B
\end{bmatrix}$$

The *body twist* $^B\mathcal{V}^b_{AB}$ is a special 6-vector that represents linear and angular velocity of the body
frame B with respect to fixed frame A, *as viewed from B*. The relationship between ${^A}\mathcal{V_B}$ and
${^B}\mathcal{V}{^b}_{AB}$ is given by the rotation matrix $^BR_A = ({^A}R_B)^T$.

$$^B\mathcal{V}^b_{AB} =
\begin{bmatrix}
v_b \\
\omega_b
\end{bmatrix} =
\begin{bmatrix}
^BR_A * {^A}v_B \\
^BR_A * {^A}\omega_B
\end{bmatrix}$$

For example, a spinning top (body frame) on a table (spatial reference frame) has angular velocity about its local Z
axis. While the top is vertical (aligned with the table), the `CartesianTwist` and the body twist are equivalent. If the
top begins to precess and tips over, then the `CartesianTwist` (expressed in the table frame) will show an angular
velocity with components in X and Y, but the body twist will remain expressed in the local body Z axis. Importantly, the
magnitude of the twist is the same in both representations, because they are both measuring the twist of the top with
respect to the table.

There is another type of twist called the *spatial twist*. Just like `CartesianTwist`, it represents angular
velocity of the body frame B with respect to fixed frame A, as viewed from A. However, the spatial linear velocity $v_s$
represents the velocity of an imaginary point at A as if it were attached to the body B, measured with respect to A, as
viewed from A.

$$^A\mathcal{V}^s_{AB} =
\begin{bmatrix}
v_s \\
\omega_s
\end{bmatrix} =
\begin{bmatrix}
{^A}v_B + {^A}t_B \times {^A}\omega_B \\
{^A}\omega_B
\end{bmatrix}$$

The quantity $v_s$ is the body linear velocity plus the radially induced velocity as the cross product of the distance
from A to B ${^A}t_B$ with the body angular velocity.

The internal `CartesianTwist` representation is arguably the most intuitive out of all three options presented here.
Still, depending on the geometric operations involved, both the body and spatial twist vectors can be useful.

The equations above have shown the derivations of each in terms of the original `CartesianTwist`. For completeness,
it is also worth mentioning the direct transformation between body and spatial representations, using the Adjoint map.

$$\mathcal{V}_s = [Ad_{T}] \mathcal{V}_b$$

The Adjoint map is defined for a given transformation matrix $T$ (with rotation matrix $R$ and displacement
vector $t$) as the adjoint matrix $[Ad_{T}]$:

$$[Ad_{T}] =
\begin{bmatrix}
0 & R \\
R & [t]_{\times}R
\end{bmatrix}$$

The inverse of the adjoint matrix maps from spatial to body frame, and can be found by using the inverse transformation
matrix.

$$^A\mathcal{V}^s_{AB} = [Ad_{^AT_B}] ^B\mathcal{V}^b_{AB}$$

$$^B\mathcal{V}^b_{AB} = [Ad_{^BT_A}] ^A\mathcal{V}^s_{AB}$$

### Cartesian acceleration

The `CartesianAcceleration` class defines only the linear and angular acceleration of a frame.

It provides the following constructors:

```c++
CartesianAcceleration::Zero("name", "reference_frame");
CartesianAcceleration::Random("name", "reference_frame");

// the linear or angular accelerations can be supplied to the constructor as vectors
linear_acceleration = Eigen::Vector3d(1.0, 2.0, 3.0);
CartesianAcceleration("name", linear_acceleration, "reference_frame");

angular_acceleration = Eigen::Vector3d(4.0, 5.0, 6.0);
CartesianAcceleration("name", linear_acceleration, angular_acceleration, "reference_frame");

// the full 6D acceleration vector can also be supplied
acceleration = Eigen::VectorXd(6);
acceleration << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
CartesianAcceleration("name", acceleration, "reference_frame");
```

Addition and subtraction is supported between `CartesianAcceleration` and `CartesianState`.

The return type of each compatible operation is shown below.

```c++

CartesianAcceleration r1 = acceleration + other_acceleration;
CartesianState r2 = acceleration + state;
CartesianState r3 = state + acceleration;

CartesianAcceleration r4 = acceleration - other_acceleration;
CartesianState r5 = acceleration - state;
CartesianState r6 = state - acceleration;

acceleration += other_acceleration; // equivalent to acceleration = acceleration + other_acceleration
acceleration += state; // equivalent to acceleration = acceleration + state

acceleration -= other_acceleration;
acceleration -= state;
```

The time integral of a `CartesianAcceleration` is a `CartesianTwist`. Because `CartesianAcceleration` represents linear
and angular acceleration in meters and radians per seconds squared, it can be converted into a relative linear and
angular velocity in meters and radians per second through multiplication by a time period.

Operations with time use `std::chrono::duration` types, such as `std::chrono::milliseconds`, `std::chrono::seconds`, or
definitions with `std::literals::chrono_literals`.

```c++
// take an acceleration with a linear acceleration of 1 meter per second squared in the X axis
// and angular acceleration of 1 radian per second squared in the Z axis 
state_representation::CartesianAcceleration acc("a", Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1));

// define a 0.1 second time duration
std::chrono::milliseconds dt(100);

// multiplying acceleration by time yields a twist with a linear term of 0.1 meters per second in the X axis
// and an angular term of 0.1 radians per second around the Z axis
state_representation::CartesianPose twist = acceleration * dt;
pose.get_position(); // (0.1, 0, 0)
pose.get_orientation(); // (0, 0, 0.1)
```

Note that the result of the integration assumes zero initial velocity. To offset the integration, simply add an initial
twist in the same reference frame as the acceleration.

```c++
// add an initial twist to offset the integration 
twist = initial_twist + acc * dt;

// a twist can also be updated through a continuous integration of acceleration
twist += acc * dt;
```

### Cartesian wrench

The `CartesianWrench` class defines only the force and torque as applied to a frame.

It provides the following constructors:

```c++
CartesianWrench::Zero("name", "reference_frame");
CartesianWrench::Random("name", "reference_frame");

// the force or torque can be supplied to the constructor as vectors
force = Eigen::Vector3d(1.0, 2.0, 3.0);
CartesianWrench("name", force, "reference_frame");

torque = Eigen::Vector3d(4.0, 5.0, 6.0);
CartesianWrench("name", force, torque, "reference_frame");

// the full 6D wrench vector can also be supplied
wrench = Eigen::VectorXd(6);
wrench << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
CartesianWrench("name", wrench, "reference_frame");
```

Addition and subtraction is supported between `CartesianWrench` and `CartesianState`.

The return type of each compatible operation is shown below.

```c++

CartesianWrench r1 = wrench + other_wrench;
CartesianState r2 = wrench + state;
CartesianState r3 = state + wrench;

CartesianWrench r4 = wrench - other_wrench;
CartesianState r5 = wrench - state;
CartesianState r6 = state - wrench;

wrench += other_wrench; // equivalent to wrench = wrench + other_wrench
wrench += state; // equivalent to wrench = wrench + state

wrench -= other_wrench;
wrench -= state;
```

#### Considerations of wrench

The wrench is distinct from the other state variables as it is not a relative spatial property. It is considered as
the measurement of a force/torque sensor at the frame, as seen from the reference frame.

Changing the observer does not change magnitude of the force or wrench measured at the frame. It only changes the
apparent direction of the wrench vector, depending on the orientation of the reference frame.

Because the `CartesianWrench` is different from other spatial properties, it is handled uniquely in the transform
and inverse operations.

A transformation of states involving a wrench will only preserve the wrench measured at the last frame, rotated to
the new reference frame. The wrench of the intermediate (inner) frame is discarded.

The inverse of a state will set the wrench to zero.

<!-- TODO: discuss wrench transform with the transposed adjoint matrix, similar to twist
#### Representations of wrench
-->

## Joint state

`JointState` follows the same logic as `CartesianState` but for representing robot states.
Similarly to the `CartesianState` the class `JointState`, `JointPositions`, `JointVelocities` and `JointTorques` have
been developed.
The API follows exactly the same logic with similar operations implemented.

A `JointState` is defined by the name of the corresponding robot and the name of each joints.

```c++
// create a state for myrobot with 3 joints
state_representation::JointState js("myrobot", std::vector<string>({ "joint0", "joint1", "joint2" }));
```

Note that if the joints of the robot are named `{"joint0", "joint1", ..., "jointN"}` as above,
you can also use the constructor that takes the number of joints as input which will name them accordingly:

```c++
// create a state for myrobot with 3 joints named {"joint0", "joint1", "joint3"}
state_representation::JointState js("myrobot", 3);
```

All the getters and setters for the `positions`, `velocities`, `accelerations` and `torques` are defined for both
`Eigen::VectorXd` and `std::vector<double>`:

```c++
js.set_positions(Eigen::Vector3d(.5, 1., 0.));
js.set_positions(std::vector<double>{.5, 1., 0.});
```

Note that when using those setters, the size of the input vector should correspond to the number of joints of the state:

```c++
js.set_positions(Eigen::Vector4d::Random()); // will throw an IncompatibleSizeException
```

### Joint state operations

Basic operations such as addition, subtraction and scaling have been implemented:

```c++
state_representation::JointState js1("myrobot", 3);
state_representation::JointState js2("myrobot", 3);
double lambda = 0.5;

// for those operation to be valid both js1 and js2
// should correspond to the same robot and have the
// same number of joints
state_representation::JointState jssum = js1 + js2;
state_representation::JointState jsdiff = js1 - js2;
state_representation::JointState jsscaled = lambda * js1;
```

Multiplication of joint states doesn't have a physical meaning and is, therefore, not implemented.

### Conversion between joint state variables

Similarly to `CartesianState`, the conversion between `JointPositions` and `JointVelocities`
happens through operations with `std::chrono_literals`.

```c++
using namespace std::chrono_literals;
auto period = 1h;

// create a state for myrobot with 3 joints named {"joint0", "joint1", "joint3"}
// and provide the position values
state_representation::JointPositions jp("myrobot", Eigen::Vector3d(1, 0, 0));

// result are velocities of 1 rad/h for joint0 expressed in rad/s
state_representation::JointVelocities jv = jp / period;
```

```c++
using namespace std::chrono_literals;
auto period = 10s;

// create a state for myrobot with 3 joints named {"joint0", "joint1", "joint3"}
// and provide the velocities values
state_representation::JointVelocities wVa("a", Eigen::Vector3d(1, 0, 0));

state_representation::JointPositions jp = period * jv; // note that jv * period is also implemented
```

## The Jacobian matrix

The `Jacobian` matrix of a robot ensures the conversion between both `CartesianState` and `JointState`.
Similarly to the `JointState`, a `Jacobian` is associated to a robot and defined by the robot and the number of joints.
As it is a mapping between joint and task spaces, as for the `CartesianState`, it is also defined by an associated
frame name and a reference frame.

```c++
// create a Jacobian for myrobot with 3 joints, associated to frame A and expressed in B
state_representation::Jacobian jac("myrobot", std::vector<string>({ "joint0", "joint1", "joint2" }), "A", "B");
```

The API is the same as the `JointState`, hence the constructor can also accept the number of joints to initialize the
joint names vector.

```c++
// create a Jacobian for myrobot with 3 joints named {"joint0", "joint1", "joint3"}, associated to frame A and
// expressed in world (default value of the reference frame when not provided)
state_representation::Jacobian jac("myrobot", 3, "A");
```

The `Jacobian` is simply a `6 x N` matrix where `N` is the number of joints.
Therefore, the data can be set from an `Eigen::MatrixXd` of correct dimensions.

```c++
jac.set_data(Eigen::MatrixXd::Random(6, 3)); // throw an IncompatibleSizeException if the size is not correct
```

All the functionalities of the `Jacobian` have been implemented such as `transpose`, `inverse` or `pseudoinverse`
functions.

```c++
/// returns the 3 x 6 transposed matrix
state_representation::Jacobian jacT = jac.transpose();
// will throw an error as a 6 x 3 matrix is not invertible
state_representation::Jacobian jacInv = jac.inverse();
// compute the pseudoinverse without the need of being invertible
state_representation::Jacobian jacPinv = jac.pseudoinverse();
```

Those operations are very useful to convert `JointState` from `CartesianState` and vice versa.

### Conversion between JointVelocities and CartesianTwist

The simplest conversion is to transform a `JointVelocities` into a `CartesiantTwist` by multiplication with
the `Jacobian`

```c++
state_representation::Jacobian jac("myrobot", 3, "eef_frame", "base_frame");
state_representation::JointVelocities jv("myrobot", 3);
// compute the twist of eef_frame in base_frame from the joint velocities
state_representation::CartesianTwist eef_twist = jac * jv;
```

The opposite transformation, from `CartesianTwist` to `JointVelocities` requires the multiplication with the `inverse`
(or `pseudoinverse`).

```c++
state_representation::Jacobian jac("myrobot", 3, "eef");
state_representation::CartesianTwist eef_twist("eef")
state_representation::JointVelocities jv = jac.pseudoinverse() * eef_twist;
// in case of non matching frame or reference frame throw an IncompatibleStatesException
state_representation::CartesianTwist link2_twist("link2", "link0")
state_representation::JointVelocities jv = jac.pseudoinverse() * link2_twist;
```

Note that the `inverse` or `pseudoinverse` functions are computationally expensive and the `solve` function that relies
on the solving of the system `Ax = b` using `Eigen` has been implemented.

```c++
state_representation::CartesianTwist eef_twist("eef")
// faster than doing jac.pseudoinverse() * eef_twist
state_representation::JointVelocities jv = jac.solve(eef_twist);
```

### Conversion between JointTorques and CartesianWrench

The other conversion that is implemented is the transformation from `CartesianWrench` to `JointTorques`, this one using
the `transpose`.

```c++
state_representation::CartesianWrench eef_wrench("eef")
// faster than doing jac.pseudoinverse() * eef_twist
state_representation::JointTorques jt = jac.transpose() * eef_wrench;
```

### Matrix multiplication

The `Jacobian` object contains an underlying `Eigen::MatrixXd` which can be retrieved using the `Jacobian::data()`
method.
Direct multiplication of the `Jacobian` object with another `Eigen::MatrixXd` has also been implemented and returns the
`Eigen::MatrixXd` product.

```c++
state_representation::Jacobian jac("myrobot", 3, "eef", Eigen::MatrixXd::Random(6, 3));
// alternatively can use the Random static constructor
state_representation::Jacobian jac = state_representation::Jacobian::Random("myrobot", 3, "eef");
Eigen::MatrixXd mat = jac.data();
Eigen::MatrixXd res = jac * Eigen::MatrixXd(3, 4); // equivalent to  jac.data() * Eigen::MatrixXd(3, 4);
```

### Changing the Jacobian reference frame

As stated earlier, the `Jacobian` is expressed in a reference frame and can, therefore, use the operations with
`CartesianPose` to be modified. It relies on the usage of the overloaded `operator*` or the `set_reference_frame`
function for inplace modifications. It is equivalent to multiply each columns of the `Jacobian` by the `CartesianPose`
on both the linear and angular part of the matrix. For this operation to be valid, the `CartesianPose` name has to
match the current `Jacobian` reference frame.

```c++
state_representation::Jacobian jac = state_representation::Jacobian::Random("myrobot", 3, "eef", "base_frame");
state_representation::CartesianPose pose = state_representation::CartesianPose::Random("base_frame", "world");
// the result is the Jacobian expressed in world
state_representation::Jacobian jac_in_world = pose * jac;
// alternatively, one case use the set_reference_frame function for inplace modifications
jac.set_reference_frame(pose);
// in case of non matching operation throw an IncompatibleStatesExceptions
jac.set_reference_frame(state_representation::CartesianPose::Random("link0", "world"));
```