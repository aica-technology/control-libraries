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
    * [Construction](#cartesian-state-construction)
    * [Cartesian getters and setters](#cartesian-getters-and-setters)
    * [Cartesian addition and subtraction](#cartesian-addition-and-subtraction)
    * [Cartesian transforms and changing the reference frame](#cartesian-transforms-changing-the-reference-frame)
    * [Cartesian distances and norms](#cartesian-distances-and-norms)
* [Derived Cartesian classes](#derived-cartesian-classes)
    * [Cartesian pose](#cartesian-pose)
    * [Cartesian twist](#cartesian-twist)
    * [Cartesian acceleration](#cartesian-acceleration)
    * [Cartesian wrench](#cartesian-wrench)
* [Joint state](#joint-state)
    * [Joint names](#joint-names)
    * [Construction](#joint-state-construction)
    * [Joint state addition and subtraction](#joint-state-addition-subtraction-and-scaling)
* [Derived joint state classes](#derived-joint-state-classes)
    * [Joint positions](#joint-positions)
    * [Joint velocities](#joint-velocities)
    * [Joint accelerations](#joint-accelerations)
    * [Joint torques](#joint-torques)
* [Jacobian](#jacobian)
    * [Reference frame and joint names](#jacobian-construction)
    * [Construction](#jacobian-construction)
    * [Jacobian matrix operations](#jacobian-matrix-operations)
    * [JointVelocities to CartesianTwist](#jointvelocities-to-cartesiantwist)
    * [CartesianTwist to JointVelocities](#cartesiantwist-to-jointvelocities)
    * [CartesianWrench to JointTorques](#cartesianwrench-to-jointtorques)
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

<!-- an HTML header is here used in place of ### to create a unique reference anchor for the generic header name -->
<h3 id="cartesian-state-construction">Construction</h3>

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

Two `CartesianState` objects can be combined with addition or subtraction, **provided they are expressed in the same
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
For example, if `s1` has position `(x1, y1, z1)` and `s2` has position `(x1, y1, z1)`, then `s1 + s2` has position
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
[Cartesian transformation](#cartesian-transforms-changing-the-reference-frame) section.

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

A `JointState` represents the instantaneous properties of a collection of joints, containing the following spatial and
dynamic properties:

- `positions`
- `velocities`
- `accelerations`
- `torques`

Each state variable is represented as an N-dimensional vector (`Eigen::VectorXd`), where N is the number of joints.

By design, a `JointState` most appropriately describes a serial linkage of revolute joints as found in typical
robot arms or manipulators. The values are assumed to be in standard SI units (radians, seconds, Newton-meters).

### Joint names

Each joint in a `JointState` collection has a name. This can make it easier to reference the state variable value of a
specific joint in the collection.

The names can be set on construction or using the `set_names()` method. As an example, a three-link robot might be
given joint names `{"shoulder", "elbow", "wrist"}`.

By default, the names are assigned based on their index, starting from 0: `{"joint0", "joint1", ..., "jointX"}`.

<!-- an HTML header is here used in place of ### to create a unique reference anchor for the generic header name -->
<h3 id="joint-state-construction">Construction</h3>

`JointState` constructors take a name and either a vector of joint names or an integer number of joints. The name
refers to the whole joint state collection, and so often corresponds to the name of the robot it represents.

```c++
// create a joint state for a robot with 3 joints
std::vector<string> joint_names = { "shoulder", "elbow", "wrist" };
state_representation::JointState js1("my_robot", joint_names);

state_representation::JointState js2("my_robot", 3); // joint names are defaulted to "joint0", "joint1", "joint2"
```

Constructing a state without any data results in an empty state. To set initial data, the static constructors `Zero()`
or `Random()` can be used. The former sets all state variables of each joint values zero. The latter sets all state
variables to a unit random state within a uniform distribution. As with the regular constructor, a vector of joint names
or an integer number of joints can be supplied.

```c++
// initialize the joint state to zero values
state_representation::JointState::Zero("my_robot", joint_names);
state_representation::JointState::Zero("my_robot", 3);

// initialize the joint state to random values
state_representation::JointState::Random("my_robot", joint_names);
state_representation::JointState::Random("my_robot", 3);
```

### Joint getters and setters

Each state variable has a corresponding getter and setter to access or modify the data after construction.

The following groups of variables can be accessed or set as a vector ordered by the joint indexes / names:

- `get_positions()`, `set_positions({...})` in radians
- `get_velocities()`, `set_velocities({...})` in radians per second
- `get_accelerations()`, `set_accelerations({...})` in radians per second squared
- `get_torques()`, `set_torques({...})` in Newton-meters

The vector setters are defined for both `Eigen::VectorXd` and `std::vector<double>`:

```c++
state_representation::JointState js("my_robot", 3);
js.set_positions(Eigen::Vector3d(.5, 1., 0.));
js.set_positions(std::vector<double>{.5, 1., 0.});
```

When setting a vector of state variables, the size of the input vector must match the number of joints.

```c++
js.set_positions(Eigen::Vector4d::Random()); // will throw an IncompatibleSizeException
```

The state variable values of each individual joint can also be accessed as scalars, using either the integer joint
index or the string joint name as the identifier:

- `get_position(id)`, `set_position(x, id)` in radians
- `get_velocity(id)`, `set_velocity(x, id)` in radians per second
- `get_acceleration(id)`, `set_acceleration(x, id)` in radians per second squared
- `get_torque(id)`, `set_torques(x, id)` in Newton-meters

```c++
auto js = state_representation::JointState::Random("my_robot", { "hip", "knee" });

js.get_position(0); // get the position of the hip joint
js.get_velocity("hip"); // get the velocity of the hip joint
js.set_acceleration(0.5, 1); // set the acceleration of the knee joint to 5 rad/s^2
js.set_torque(2.0, "knee"); // set the torque of the knee joint to 2 Nm
```

### Joint state addition, subtraction and scaling

Two `JointState` objects can be combined with addition or subtraction, provided they have the same name and joint names.

```c++
state_representation::JointState js1("myrobot", 3);
state_representation::JointState js2("myrobot", 3);

// for those operation to be valid both js1 and js2 must have the same name and matching joint names
state_representation::JointState jssum = js1 + js2;
state_representation::JointState jsdiff = js1 - js2;
```

For all state variables, the result of the operation is applied to each state variable element-wise. For example, if
`js1` has joint positions `(x1, x2, x3)` and `js2` has joint positions `(y1, y2, y3)`, then `js1` + `js2` has joint
positions `(x1 + y1, x2 + y2, x3 + y3)`. The same applies for subtraction and is true for all state variable vectors.

A `JointState` can also be multiplied or divided by a scalar to scale each state variable element-wise.

```c++
state_representation::JointState double_state = 2.0 * js1;
state_representation::JointState half_state = js1 / 2.0;
```

## Derived joint state classes

The `JointState` class contains all spatial and dynamic state variables of a joint collection. In some cases, it is
convenient to operate only with specific state variables. The following derived classes are defined:

- `JointPositions`
- `JointVelocities`
- `JointAccelerations`
- `JointTorques`

### Joint positions

The `JointPositions` class defines only the positions of joints.

In addition to the constructors inherited from `JointState`, it can be constructed with a vector of initial positions.

```c++
Eigen::VectorXd initial_positions(3);
initial_positions << 1.0, 2.0, 3.0;

// create a 3-axis robot with initial positions and default joint names
state_representation::JointPositions("my_robot", initial_positions);

// assign joint names alongside the initial positions
state_representation::JointPositions("my_robot", { "shoulder", "elbow", "wrist" }, initial_positions);
```

Addition and subtraction is supported between `JointPositions` and `JointState`, provided that they are compatible.

The return type of each compatible operation is shown below.

```c++
JointPositions r1 = joint_positions + other_joint_positions;
JointState r2 = joint_positions + joint_state;
JointState r3 = joint_state + joint_positions;

JointPositions r4 = joint_positions - other_joint_positions;
JointState r5 = joint_positions - joint_state;
JointState r6 = joint_state - joint_positions;

joint_positions += other_joint_positions; // equivalent to joint_positions = joint_positions + other_joint_positions
joint_positions += joint_state; // equivalent to joint_positions = joint_positions + joint_state

joint_positions -= other_joint_positions;
joint_positions -= joint_state;
```

The time derivative of `JointPositions` are `JointVelocities`. The angular displacement of the joints can be converted
into angular velocity through division by a time period.

Operations with time use `std::chrono::duration` types, such as `std::chrono::milliseconds`, `std::chrono::seconds`, or
definitions with `std::literals::chrono_literals`.

```c++
// take a robot with a displacement of 1 radian around the first joint
state_representation::JointPositions positions("my_robot", Eigen::Vector3d(1, 0, 0));

// define a 2 second time duration
std::chrono::seconds dt(2);

// dividing position by time yields an angular velocity 0.5 radians per second around the first joint
state_representation::JointVelocities velocities = positions / dt;
velocities.get_velocities(); // (0.5, 0, 0)
```

### Joint velocities

The `JointVelocities` class defines only the velocities of joints.

In addition to the constructors inherited from `JointState`, it can be constructed with a vector of initial velocities:

```c++
Eigen::VectorXd initial_velocities(3);
initial_velocities << 1.0, 2.0, 3.0;

// create a 3-axis robot with initial velocities and default joint names
state_representation::JointVelocities("my_robot", initial_velocities);

// assign joint names alongside the initial velocities
state_representation::JointVelocities("my_robot", { "shoulder", "elbow", "wrist" }, initial_velocities);
```

Addition and subtraction is supported between `JointVelocities` and `JointState`, provided that they are compatible.

The return type of each compatible operation is shown below.

```c++
JointVelocities r1 = joint_velocities + other_joint_velocities;
JointState r2 = joint_velocities + joint_state;
JointState r3 = joint_state + joint_velocities;

JointVelocities r4 = joint_velocities - other_joint_velocities;
JointState r5 = joint_velocities - joint_state;
JointState r6 = joint_state - joint_velocities;

joint_velocities += other_joint_velocities; // equivalent to joint_velocities = joint_velocities + other_joint_velocities
joint_velocities += joint_state; // equivalent to joint_velocities = joint_velocities + joint_state

joint_velocities -= other_joint_velocities;
joint_velocities -= joint_state;
```

The time derivative of `JointVelocities` are `JointAccelerations`. The angular velocity of the joints can be converted
into angular acceleration through division by a time period.

Similarly, the time integral of `JointVelocities` are `JointPositions`. The angular velocity of the joints can be
converted into an angular displacement through multiplication by a time period.

Operations with time use `std::chrono::duration` types, such as `std::chrono::milliseconds`, `std::chrono::seconds`, or
definitions with `std::literals::chrono_literals`.

```c++
// take a robot with an angular velocity of 1 radian per second around the first joint
state_representation::JointVelocities velocities("my_robot", Eigen::Vector3d(1, 0, 0));

// define a 0.5 second time duration
std::chrono::milliseconds dt(500);

// dividing velocity by time yields an angular acceleration 2 radians per second squared around the first joint
state_representation::JointAccelerations accelerations = velocities / dt;
accelerations.get_accelerations(); // (2, 0, 0)

// multiplying velocity by time yields joint positions with a displacement of 0.5 radians around the first joint
state_representation::JointPositions positions = velocities * dt;
positions.get_positions(); // (0.5, 0, 0)
```

Note that the result of the integration is the displacement from an initial (zero) position. To offset the integration,
simply add initial joint positions with the same name and joint names.

```c++
// add initial positions to offset the integration 
positions = initial_positions + velocities * dt;

// joint positions can also be updated through a continuous integration of joint velocities
positions += velocities * dt;
```

### Joint accelerations

The `JointAccelerations` class defines only the accelerations of joints.

In addition to the constructors inherited from `JointState`, it can be constructed with a vector of initial
accelerations:

```c++
Eigen::VectorXd initial_accelerations(3);
initial_accelerations << 1.0, 2.0, 3.0;

// create a 3-axis robot with initial accelerations and default joint names
state_representation::JointAccelerations("my_robot", initial_accelerations);

// assign joint names alongside the initial accelerations
state_representation::JointAccelerations("my_robot", { "shoulder", "elbow", "wrist" }, initial_accelerations);
```

Addition and subtraction is supported between `JointAccelerations` and `JointState`, provided that they are compatible.

The return type of each compatible operation is shown below.

```c++
JointAccelerations r1 = joint_accelerations + other_joint_accelerations;
JointState r2 = joint_accelerations + joint_state;
JointState r3 = joint_state + joint_accelerations;

JointAccelerations r4 = joint_accelerations - other_joint_accelerations;
JointState r5 = joint_accelerations - joint_state;
JointState r6 = joint_state - joint_accelerations;

joint_accelerations += other_joint_accelerations; // equivalent to joint_accelerations = joint_accelerations + other_joint_accelerations
joint_accelerations += joint_state; // equivalent to joint_accelerations = joint_accelerations + joint_state

joint_accelerations -= other_joint_accelerations;
joint_accelerations -= joint_state;
```

The time integral of `JointAccelerations` are `JointVelocities`. The angular acceleration of the joints can be converted
into an angular velocities through multiplication by a time period.

Operations with time use `std::chrono::duration` types, such as `std::chrono::milliseconds`, `std::chrono::seconds`, or
definitions with `std::literals::chrono_literals`.

```c++
// take a robot with an angular acceleration of 1 radian per second squared around the first joint
state_representation::JointAccelerations accelerations("my_robot", Eigen::Vector3d(1, 0, 0));

// define a 0.5 second time duration
std::chrono::milliseconds dt(500);

// multiplying acceleration by time yields an angular velocity of 0.5 radians around the first joint
state_representation::JointVelocities velocities = accelerations * dt;
velocities.get_velocities(); // (0.5, 0, 0)
```

Note that the result of the integration assumes zero initial joint velocities. To offset the integration, simply add
initial joint velocities with the same name and joint names.

```c++
// add initial velocities to offset the integration 
velocities = initial_velocities + accelerations * dt;

// joint velocities can also be updated through a continuous integration of joint accelerations
velocities += accelerations * dt;
```

### Joint torques

The `JointTorques` class defines only the torques of joints.

In addition to the constructors inherited from `JointState`, it can be constructed with a vector of initial torques.

```c++
Eigen::VectorXd initial_torques(3);
initial_torques << 1.0, 2.0, 3.0;

// create a 3-axis robot with initial torques and default joint names
state_representation::JointTorques("my_robot", initial_torques);

// assign joint names alongside the initial torques
state_representation::JointTorques("my_robot", { "shoulder", "elbow", "wrist" }, initial_torques);
```

Addition and subtraction is supported between `JointTorques` and `JointState`, provided that they are compatible.

The return type of each compatible operation is shown below.

```c++
JointTorques r1 = joint_torques + other_joint_torques;
JointState r2 = joint_torques + joint_state;
JointState r3 = joint_state + joint_torques;

JointTorques r4 = joint_torques - other_joint_torques;
JointState r5 = joint_torques - joint_state;
JointState r6 = joint_state - joint_torques;

joint_torques += other_joint_torques; // equivalent to joint_torques = joint_torques + other_joint_torques
joint_torques += joint_state; // equivalent to joint_torques = joint_torques + joint_state

joint_torques -= other_joint_torques;
joint_torques -= joint_state;
```

## Jacobian

In robotics, the Jacobian is used to map state variables between joint space and Cartesian space. Mathematically, it
is a matrix containing the set of partial derivatives for a vector function.

Most commonly, it is used to convert joint velocities of a robot into the Cartesian velocity of the end-effector.
Conversely, the transpose can map a Cartesian wrench applied at the end-effector to the associated torques at each
joint. The Jacobian matrix has additional uses for inverse kinematics and null-space control.

The `Jacobian` class is a wrapper for the underlying matrix that works directly with `JointState` and `CartesianState`
types while providing additional methods and operators.

### Reference frame and joint names

Because the `Jacobian` class is designed to map state between joint and Cartesian space, it shares the properties of
both `JointState` and `CartesianState`; the accessor methods are listed below:

- `get_name()` returns the "robot name", i.e. the name of the corresponding `JointState`
- `get_frame()` returns the end-effector frame name, i.e. the name of the corresponding `CartesianState`
- `get_joint_names()` returns the vector of joint names of the corresponding `JointState`
- `get_reference_frame()` returns the reference frame of the corresponding `CartesianState`

Each property is initialized on construction and has a corresponding setter for post-construction modifications.

<!-- an HTML header is here used in place of ### to create a unique reference anchor for the generic header name -->
<h3 id="jacobian-construction">Construction</h3>

`Jacobian` constructors take a name, a number of joints or optional vector of joint names, a frame name and an optional
reference frame. As with `JointState`, joint names are default initialized based on their index, starting from 0:
`{"joint0", "joint1", ..., "jointX"}`. As with `CartesianState`, the reference frame is "world" by default.

Constructing a `Jacobian` without any data results in an empty state. The initial data can be set from an
`Eigen::MatrixXd` matrix of size `6 x N`, where `N` is the number of joints. Alternatively, the  `Random()` static
constructor can be used, which sets all matrix elements to a unit random state within a uniform distribution.

```c++
// 3-axis robot "my_robot" with end-effector frame "A" expressed in "world" (default)
state_representation::Jacobian j1("my_robot", 3, "A");
j1.get_joint_names(); // {"joint0", "joint1", "joint2"}

// the reference frame can be supplied to any constructor as the last argument:
// 3-axis robot "my_robot" with end-effector "A" expressed in "B"
state_representation::Jacobian("my_robot", 3, "A", "B");

// 3-axis robot "my_robot" with specific joint names and end-effector frame "A" expressed in "world"
std::vector<std::string> joint_names = { "shoulder", "elbow", "wrist" };
state_representation::Jacobian j2("my_robot", joint_names, "A");
j2.get_joint_names(); // {"shoulder", "elbow", "wrist"}

// same as above but in reference frame "B"
state_representation::Jacobian j2("my_robot", joint_names, "A", "B");



auto data = Eigen::MatrixXd::Zero(6, 3);
// 3-axis robot "my_robot" with end-effector frame "A" with specific data matrix and default joint names
state_representation::Jacobian("my_robot", "A", data); // expressed in "world"
state_representation::Jacobian("my_robot", "A", data, "B"); // expressed in reference frame "B"

// 3-axis robot "my_robot" with end-effector frame "A" with specific data matrix and custom joint names
state_representation::Jacobian("my_robot", joint_names, "A", data); // expressed in "world"
state_representation::Jacobian("my_robot", joint_names, "A", data, "B"); // expressed in reference frame "B"

// 3-axis robot "my_robot" with end-effector frame "A" with random data matrix and default joint names
state_representation::Jacobian::Random("my_robot", 3, "A"); // expressed in "world"
state_representation::Jacobian::Random("my_robot", 3, "A", "B"); // expressed in reference frame "B"

// 3-axis robot "my_robot" with end-effector frame "A" with random data matrix and custom joint names
state_representation::Jacobian::Random("my_robot", joint_names, "A"); // expressed in "world"
state_representation::Jacobian::Random("my_robot", joint_names, "A", "B"); // expressed in reference frame "B"
```

#### Construction from robot model

A `Jacobian` that accurately maps from joint space to Cartesian space depends on the structure of the robot (the spatial
offset between each joint) in addition to the current joint positions. Advanced users may calculate or supply the matrix
manually using the appropriate constructor or data setters. The easiest way to construct an accurate `Jacobian` is to
use the `robot_model::Model` class. See the documentation of the `robot_model` module for more details; an example is
given below.

```c++
robot_model::Model robot("my_robot", "/examples/my_robot.urdf");

auto joint_positions = state_representation::JointPositions::Random("my_robot", robot.get_joint_frames());
auto jacobian = model.compute_jacobian(joint_positions);
```

### Jacobian matrix operations

The `Jacobian` type is a wrapper for matrix data of type `Eigen::MatrixXd`. The matrix always has 6 rows, corresponding
to the 6 Cartesian degrees of freedom (linear X, Y, Z and angular X, Y, Z, in that order). The matrix has a number
of columns corresponding to the number of joints; a 3-axis robot will have a `6 x 3` Jacobian matrix while a 6-axis
robot will have a square `6 x 6` matrix.

The `data()` method returns the underlying matrix, while `set_data()` can be used to overwrite the matrix. When setting
the data on a previously constructed `Jacobian`, the matrix must have the correct size.

```c++
state_representation::Jacobian jacobian("my_robot", 3, "A");
jacobian.set_data(Eigen::MatrixXd::Random(6, 3)); 

// an IncompatibleSizeException will be thrown if the size is not correct
jacobian.set_data(Eigen::MatrixXd::Random(6, 4)); // error!
jacobian.set_data(Eigen::MatrixXd::Random(5, 3)); // error!
```

Variants of the matrix such as the transpose or inverse can be accessed with the following methods.

```c++
state_representation::Jacobian j3("3_axis_robot", 3, "A");
state_representation::Jacobian j6("6_axis_robot", 6, "A");

/// for a 6 x 3 Jacobian, returns the 3 x 6 transposed matrix
Eigen::MatrixXd jT = j3.transpose();

// for a square Jacobian, returns the inverted matrix
Eigen::MatrixXd j_inv = j6.inverse();
j3.inverse(); // throws an IncompatibleSizeException as non-square matrices are not directly invertible

// for non-square Jacobians, use the pseudo-inverse instead
Eigen::MatrixXd j_pinv = j3.pseudoinverse();
```

### JointVelocities to CartesianTwist

To transform `JointVelocities` into a `CartesiantTwist`, simply multiply the `Jacobian` by the `JointVelocities`.

For the transformation to be valid, the Jacobian matrix data must be set according to the current robot configuration.
See also the section [Construction from robot model](#construction-from-robot-model). In addition, the `JointVelocites`
must be compatible with the `Jacobian`, with the same name, number of joints and joint names.

```c++
state_representation::Jacobian jacobian("my_robot", 3, "end_effector", "base_frame");
jacobian.set_data(...); // set the Jacobian data matrix for the current robot configuration

// get joint velocities for the same robot
auto joint_velocities = state_representation::JointVelocities::Random("my_robot", 3);

// compute the twist of the "end_effector" frame expressed in "base_frame"
state_representation::CartesianTwist end_effector_twist = jacobian * joint_velocities;
```

### CartesianTwist to JointVelocities

The transformation from `CartesianTwist` to `JointVelocities` requires the inverse of the Jacobian matrix (or, in the
case of non-square matrices, the pseudo-inverse).

One approach is to use the matrix results of the `inverse()` or `pseudoinverse()` methods and multiply them directly
with data vectors.

```c++
state_representation::Jacobian j3("3_axis_robot", 3, "end_effector", "base_frame");
state_representation::Jacobian j6("6_axis_robot", 6, "end_effector", "base_frame");
j3.set_data(...); // set the 6 x 3 Jacobian data matrix for the current robot configuration
j6.set_data(...); // set the 6 x 6 Jacobian data matrix for the current robot configuration

auto raw_end_effector_twist = Eigen::VectorXd::Random(3);
Eigen::VectorXd raw_joint_velocities = j3.pseudoinverse() * raw_end_effector_twist; // vector of 3 joint velocities

raw_end_effector_twist = Eigen::VectorXd::Random(6);
Eigen::VectorXd raw_joint_velocities = j6.inverse() * raw_end_effector_twist; // vector of 6 joint velocities
```

However, this approach is not ideal for two reasons; it doesn't operate directly on `CartesianTwist` and
`JointVelocities` types, and is mathematically inefficient in the case of a square Jacobian (i.e. a 6-axis robot).
For the equation `W = inv(J) * V`, W can be solved for more efficiently by using QR decomposition.

Instead of accessing the transformed matrix and multiplying it by a data vector, instead use the overloaded functions
directly with `CartesianTwist` as illustrated below. Note that the `CartesianTwist` must be compatible with the
`Jacobian`, with a matching frame and reference frame.

```c++
// get the Cartesian twist of the same frame and reference frame
auto end_effector_twist = state_representation::CartesianTwist::Random("end_effector", "base_frame");

// compute the joint velocities of the 3-axis robot from the Cartesian twist using the pseudo-inverse
state_representation::JointVelocities j3_velocities = j3.pseudoinverse(end_effector_twist);

// compute the joint velocities of the 3-axis robot from the Cartesian twist using the inverse
state_representation::JointVelocities j6_velocities = j6.inverse(end_effector_twist);
```

### CartesianWrench to JointTorques

The transformation of a `CartesianWrench` into `JointTorques` uses the transpose of the Jacobian matrix.

As with the conversion from [CartesianTwist to JointVelocities](#cartesiantwist-to-jointvelocities), the operations
can either be done using the matrix result of `transpose()`, or by providing the `CartesianWrench` as the input to the
overloaded `tranpose()` function. The latter has the benefit of working directly with the abstract state classes
instead of raw data vectors. Note that the `CartesianWrench` must be compatible with the `Jacobian`, with a matching
frame and reference frame.

```c++
state_representation::Jacobian jacobian("my_robot", 3, "end_effector", "base_frame");
jacobian.set_data(...); // set the Jacobian data matrix for the current robot configuration

// get the Cartesian wrench of the same frame and reference frame
auto end_effector_wrench = state_representation::CartesianTwist::Random("end_effector", "base_frame");

// compute the torques of each joint from the Cartesian twist
state_representation::JointTorques joint_torques = jacobian.transpose(end_effector_wrench);

// the matrix product should only be used on raw data vectors
Eigen::VectorXd raw_joint_torques = jacobian.transpose() * end_effector_wrench.get_wrench();
```

### Changing the Jacobian reference frame

Whenever the `Jacobian` is used to convert between joint and Cartesian space, the Cartesian state variables must be
expressed in the same reference frame as the `Jacobian`. If the `Jacobian` is expressed in a robot base frame while
a `CartesianTwist` is measured in a world reference frame, then that twist cannot be directly used to find the
corresponding joint velocities.

```c++
auto jacobian = state_representation::Jacobian::Random("my_robot", 3, "end_effector", "base_frame");
auto twist_in_world = state_representation::CartesianTwist::Random("end_effector", "world");

jacobian.pseudoinverse(twist_in_world); // error! IncompatibleStatesException
```

If the transformation between the robot base frame and world is known and expressed as a `CartesianPose`, it could be
used to transform the `CartesianTwist` relative to the robot base frame (see the section describing
[Cartesian transforms](#cartesian-transforms-changing-the-reference-frame) for more information).

```c++
auto pose = state_representation::CartesianPose::Random("base_frame", "world");
auto twist_in_base_frame = pose.inverse() * twist_in_world;

state_representation::JointVelocities joint_velocities = jacobian.pseudoinverse(twist_in_base_frame);
```

This can also be applied to the reverse case:

```c++
state_representation::CartesianTwist recalculated_twist_in_base_frame = jacobian * joint_velocities;
auto recalculated_twist_in_world = pose * recalculated_twist_in_base_frame;
```

However, transforming the Cartesian state variables before and after manipulation with the `Jacobian` can be
inefficient. Consider the case of high-frequency sensor data measured in a different reference frame needing to be
transformed before every Jacobian operation.

Instead, a `CartesianPose` can be used to change the reference frame of the `Jacobian` through a corresponding linear
transformation of the underlying data matrix. As a result, previously incompatible operations automatically work in
the new reference frame.

```c++
state_representation::Jacobian jacobian_in_world = pose * jacobian;
jacobian.get_reference_frame(); // world

state_representation::JointVelocities joint_velocities = jacobian_in_world.pseudoinverse(twist_in_world);

state_representation::CartesianTwist recalculated_twist_in_world = jacobian_in_world * joint_velocities;
```
