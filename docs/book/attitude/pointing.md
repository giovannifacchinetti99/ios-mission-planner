# Pointing at the target

The two previous chapters controlled and estimated the attitude with
respect to a target chosen once. During an approach the target attitude is
not fixed: the deputy must keep a sensor, and later its docking port,
pointed at a chief whose direction changes continuously as the relative
trajectory unfolds. This chapter connects the attitude to the relative
motion of {doc}`../relative/hcw`. It shows how a commanded attitude is
built from the relative position, and what it costs to update that command
only from time to time.

## Building a commanded attitude from two directions

A pointing requirement specifies directions, not a quaternion. The typical
one reads: the sensor boresight, a given axis of the body, must point at
the chief, and a second body axis must stay as close as possible to a
reference direction, for example the orbit normal, to fix the rotation
about the boresight.

This is the same mathematical problem as determining an attitude from two
observed directions, read in the opposite sense. In
{doc}`estimation`, two directions known in the reference frame and measured
in the body frame gave the attitude the body has. Here two directions
chosen in the reference frame, and the two body axes that must align with
them, give the attitude the body should have. The TRIAD construction solves
both. With the boresight axis $\hat{\vect a}_1$ and the secondary axis
$\hat{\vect a}_2$ in body coordinates, and the direction to the chief
$\hat{\vect d}_1 = -\vect\rho/|\vect\rho|$ and the reference
$\hat{\vect d}_2$ in the reference frame, the commanded attitude is

$$
R_t = \mathrm{TRIAD}\left(\hat{\vect a}_1,\hat{\vect a}_2;\;
\hat{\vect d}_1,\hat{\vect d}_2\right),
$$

converted to a quaternion. As in the determination problem, the first pair
is matched exactly and the second only as far as geometry allows: the
boresight points exactly at the chief, and the secondary axis lies in the
plane of the two directions.

## The pointing error

The quantity of interest is not the full attitude error but the angle
between the boresight and the direction to the chief, because that is what
decides whether the chief is inside the field of view of the sensor. With
the body axis $\hat{\vect a}_1$ rotated into the reference frame by the
actual attitude,

$$
\varepsilon = \arccos\left[\left(R^{\mathsf T}\hat{\vect a}_1\right)
\cdot\hat{\vect d}_1\right].
$$

This check is different from the approach corridor of
{doc}`../safety/safety`. The corridor asks whether the deputy is inside a
cone with its vertex on the chief, and depends only on the trajectory. The
pointing error asks whether the chief is inside a cone with its vertex on
the deputy, and depends on the attitude that the control actually achieves.

## Updating the command at intervals

A guidance computer does not recompute the commanded attitude continuously.
It updates it at a fixed cadence and the controller tracks a piecewise
constant command. Between two updates the direction to the chief moves and
the command goes stale, so the pointing error has the shape of a sawtooth:
it grows between updates and drops at each one.

The size of the teeth depends on how fast the line of sight rotates. For a
relative velocity $\vect v$ at a distance $\rho$, the line of sight turns
at the rate

$$
\dot\lambda = \frac{|\vect\rho\times\vect v|}{\rho^2},
$$

which grows as the distance shrinks. A cadence that is ample at five
kilometres falls behind in the last hundreds of metres, exactly when
pointing matters most. The remedies are a faster cadence near the target,
or a command that includes the rate of the line of sight as a feedforward,
so that the controller tracks a moving target instead of chasing a sequence
of fixed ones. The closed-loop simulation of the next part recomputes the
command at every control step and supplies the rate of the commanded
attitude to the controller.

## The frame of the command

In this chapter the commanded attitude is computed in the LVLH frame of the
chief and that frame is treated as if it were inertial. It is not: it turns
once per orbit, about 0.06 degrees per second in low Earth orbit. The
simplification is acceptable for a study of the cadence, because the
rotation of the frame is slow compared with the bandwidth of the attitude
control, but it leaves out a real effect. A deputy that keeps pointing at a
chief from a hold point on the V-bar must turn once per orbit in inertial
space, and its controller must provide that rate. The closed-loop simulator
composes the command in the inertial frame and removes the simplification.

## In the code

The functions are in `dynamics/attitude/target_pointing.py`.
`target_quaternion_toward_chief` builds the commanded quaternion from a
relative position with the TRIAD construction, with the body axes and the
secondary reference as optional arguments. `boresight_pointing_error_deg`
returns the angle between the boresight and the direction to the chief for
a given attitude and relative position.

`fly_with_periodic_retargeting` integrates the relative motion, with the
HCW equations, and the attitude, with the pointing control law of
{doc}`dynamics`, over a sequence of intervals. At the start of each
interval it recomputes the commanded quaternion from the current relative
position and holds it until the next. It returns the histories of relative
state, attitude and pointing error.

## In the notebook

{doc}`../examples/13_attitude_coupled_pointing/attitude_coupled_pointing`
flies a two-impulse transfer from a hold point 5 km behind the chief toward
a closed ellipse, with the attitude tracking the chief. It shows the
sawtooth of the pointing error and its growth as the deputy closes in, and
then repeats the flight with different update intervals to quantify how the
cadence drives the error.

## References

1. F. L. Markley, J. L. Crassidis, *Fundamentals of Spacecraft Attitude
   Determination and Control*, Springer, 2014.
2. B. Wie, *Space Vehicle Dynamics and Control*, 2nd edition, AIAA, 2008.
3. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
