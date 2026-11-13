# The 3D viewer

The results of a simulation have so far been numbers and plots, each
showing one quantity. This chapter describes the last tool of the book, a
viewer that replays a simulation in three dimensions: the two spacecraft in
the frame of the chief, with the Earth below, the Sun where it was, and the
deputy turning, firing and closing in as the simulation computed.

The viewer adds no result. It displays the output of {doc}`../gnc/simulator`
and computes nothing of its own, so what it shows is exactly as right as
the simulation behind it. Its purpose is to make the behaviour of the whole
system visible at once, which no single plot does, and to show a mission to
someone who will not read the plots.

The run below is the reference mission. It can be played, paused and
dragged along the timeline; the buttons change the camera.

<iframe src="../examples/21_docking_3d_viewer/docking_viewer.html"
        width="100%" height="620" style="border:1px solid #ccc; border-radius:6px;"
        title="3D viewer of the docking" loading="lazy"></iframe>

## A page that carries its data

The viewer is a single HTML file. It contains the data of the run and the
code that draws it, and it opens in a browser with no server and nothing to
install. The drawing uses three.js, a library for 3D graphics in the
browser, loaded from a public content delivery network when the page is
opened.

A simulation of the reference mission has several thousand steps, and the
playback does not need all of them. Far from the chief the motion is slow
on the scale of the scene and one sample every few seconds is enough;
inside the last 100 m every step is kept. The page interpolates between
samples, linearly for the position and on the sphere of unit quaternions
for the attitude, so the motion is smooth at any playback speed.

## The frame of the scene

The scene is drawn in the LVLH frame of the chief, the frame of every
relative-motion plot of this book. The chief is fixed at the center. The
Earth is below and turns underneath once per orbit, because in a frame that
rotates with the chief it is everything else that rotates, the Sun and the
stars included.

A graphics library has its own axes, with $Y$ up. The scene maps the LVLH
axes $(x, y, z)$, radial, along-track and orbit normal, to

$$
(X, Y, Z) = (y,\; x,\; -z),
$$

so that the direction of flight is to the right and the radial direction is
up, as in the plots. The matrix of this mapping has determinant $+1$: it is
a rotation, and it preserves the handedness of everything it transforms. A
mapping with determinant $-1$ would mirror the scene, and a mirrored
spacecraft turns the wrong way.

The attitude needs the same care as in {doc}`../attitude/estimation`. The
attitude chapters use the rotation from the inertial frame to the body
frame, a passive rotation that changes the components of a fixed vector.
A graphics library wants the opposite, the active rotation that places an
object in the scene, $\vect v_{scene} = R\,\vect v_{body}$, as a quaternion
with the scalar part last. The matrix is $R = M\,R_{lb}$, with $M$ the
mapping above and $R_{lb}$ the rotation from body to LVLH axes that the
simulation records. Its conversion to a quaternion is the transpose of the
one used in the attitude chapters, and it has its own function so that the
two conventions never mix.

## What is drawn

The light of the scene comes from the direction of the Sun at each instant,
computed by the functions of {doc}`../mission/illumination`, and it goes
out while the chief is in eclipse. The lighting seen in the viewer is
therefore the lighting of the simulated epoch: a final approach flown in
the dark is dark.

The trajectory is drawn as a faint line, with the part already flown in
yellow. The purple wireframe is the approach corridor of
{doc}`../gnc/guidance_control`. The green cone is the field of view of the
relative sensor, and the small green wire sphere is the position that the
navigation estimates: far from the chief it is visibly apart from the
deputy, and it converges onto it as the range decreases, which is the
behaviour of {doc}`../gnc/navigation` made visible. A plume appears on a
face of the deputy when the thrusters of that axis fire.

A panel gives the phase of the mission, the distance, the closing speed and
the lateral offset between the ports, the attitude error, the velocity
change spent, the fraction of the wheel capacity in use and the lighting.
At the end of the run a second panel lists the conditions at contact
against their limits.

Four cameras are available. The overview frames both spacecraft and moves
in as they get closer. The side view looks along the orbit normal and
reproduces the plane of the trajectory plots. The chase view follows the
deputy from behind. The port camera looks from the front of the deputy
along the axis of its relative sensor, and shows what an optical sensor
would see: the target next to the port of the chief stays under the
crosshair for the whole approach, because the attitude control keeps it
there.

## In the code

`export_viewer` in `viz/viewer.py` writes the page for a `SimulationResult`.
`build_viewer_data` reduces the result to the sampled arrays and to the
metadata that the page needs, and `rotation_to_scene_quaternion` converts
the attitude to the convention of the scene. The page itself is a template,
`viz/templates/viewer.html`, into which the data are inserted.

The state of the page can be set in its address, which makes it possible to
link to a moment of a run: the parameters are `t`, the time in seconds,
`cam`, one of `overview`, `side`, `chase` and `port`, `play`, 0 or 1,
`speed`, and `az`, `el` and `zoom` for the overview camera.

## In the notebook

{doc}`../examples/21_docking_3d_viewer/docking_3d_viewer` exports the
viewer of the reference mission, checks the exported attitude and position
at the instant of contact against the simulation, and exports a second
viewer for a run in which the relative sensor is lost and the deputy
performs the collision avoidance maneuver.

## References

1. three.js, a JavaScript library for 3D graphics, <https://threejs.org>.
2. F. L. Markley, J. L. Crassidis, *Fundamentals of Spacecraft Attitude
   Determination and Control*, Springer, 2014.
