#set document(
  author: "Ren Lin",
  title: "Position, Trust, and the Algebra of Telemetry",
)
#set page(paper: "us-letter", margin: (x: 1.25in, y: 1in), numbering: "1")
#set text(font: "New Computer Modern", size: 11pt)
#set heading(numbering: "1.")
#show heading: it => {
  v(0.8em)
  it
  v(0.4em)
}
#show math.equation.where(block: true): it => {
  v(0.3em)
  it
  v(0.3em)
}

#import "@preview/fletcher:0.5.3": diagram, edge, node
#import "@preview/cetz:0.3.4"

#align(center)[
  #text(13pt)[Position, Trust, and the Algebra of Telemetry] \
  #v(0.5em)
  #text(
    11pt,
    style: "italic",
  )[From Ren Lin's Notebook, formatted and rewritten by Claude | Team 1280 EECS]
]

#v(1em)

#pagebreak()
#outline()
#pagebreak()

= Prerequsites
simple category theory, group theory, and HOTT generally makes everything easier to understand. It's written and by me and checked with Claude to make the literature better and easier to understand.

= The Deceptively Simple Question

Every match starts the same way: a 120-pound aluminum chassis is set down on a field,
and a countdown begins. In those first seconds, the robot has no idea where it is. Not
roughly. Not approximately. Literally nothing---a blank slate. Before it can score, before
it can navigate, before it can do anything useful, it needs an answer to the oldest
question in robotics: _where am I?_

The naive answer is "count the wheel rotations." If you know how far each wheel has
turned and which way it was pointed, you can dead-reckon your position from a known
starting point. And it works. Until a wheel spins in place during a collision. Until the
gyroscope drifts. Until another robot shoves you sideways and the encoders report
confident, pristine nonsense. The question is not just _how_ to estimate position---it's
how to know when to _trust_ the estimate.

This document is an account of how this robot answers both questions at once.

= Poses Live in a Lie Group

Before any algorithms, there is geometry. A robot moving on a flat field has three
degrees of freedom: two translations $(x, y)$ and one rotation $theta$. Together these
form a *pose*, which is not merely a tuple of numbers---it is an element of the *special
Euclidean group* $S E(2)$.

A pose can be represented as a $3 times 3$ matrix:

$
  p = mat(cos theta, -sin theta, x; sin theta, cos theta, y; 0, 0, 1) in S E(2)
$

This representation makes composition natural. If the robot is at pose $p_1$ and moves
to pose $p_2$ *relative to its own frame*, the new world pose is simply matrix
multiplication $p_1 dot p_2$. The group axioms fall out automatically: there is an
identity (the zero pose), every pose has an inverse (the "undo" transformation), and
composition is associative. WPILib's `Pose2d` class carries exactly this structure, and
the pose estimator maintains a running element of $S E(2)$ updated each control loop.

The *Lie algebra* $frak(s e)(2)$ is the tangent space at the identity---it describes
instantaneous velocities rather than positions. An element $xi = (v_x, v_y, omega)$
is precisely a `ChassisSpeeds` object: forward velocity, strafe velocity, and angular
velocity. The exponential map $exp: frak(s e)(2) -> S E(2)$ converts a constant twist
applied for time $Delta t$ into the resulting pose displacement:

$ p_(t + Delta t) approx p_t dot exp(xi dot Delta t) $

For small $Delta t$, this is the familiar Euler integration step, but the group-theoretic
framing clarifies why you must compose (multiply) rather than add: poses are not vectors,
and naive coordinate addition ignores the rotational coupling.

= Swerve Kinematics: Four Modules, One Body

A swerve drivetrain has four independently steered and driven wheels. Each module $i$
sits at position $bold(r)_i = (r_(i x), r_(i y))$ relative to the robot center and can
point its wheel in any direction $phi_i$. Given a target chassis twist
$xi = (v_x, v_y, omega)$, the velocity each module must produce is:

$ bold(v)_i = mat(1, 0, -r_(i y); 0, 1, r_(i x)) mat(v_x; v_y; omega) $

This is the *forward kinematics map*: a linear function from the three-dimensional twist
space to the two-dimensional velocity space of each module. Stacking all four modules
gives an $8 times 3$ Jacobian $A$, and the full system is:

$ bold(b) = A xi $

where $bold(b) in RR^8$ holds the eight measured wheel velocity components. Running this
*backward*---from measured wheel velocities to estimated twist---is the core of encoder
odometry, solved in the least-squares sense:

$ hat(xi) = (A^top A)^(-1) A^top bold(b) $

The SwerveDriveKinematics class in WPILib solves exactly this system, wrapping the
pseudoinverse in an efficient form. The integrated pose is then updated by composing
$exp(hat(xi) dot Delta t)$ onto the current estimate at up to 250 Hz on a dedicated
odometry thread, decoupled from the 50 Hz robot loop via a concurrent queue architecture.

= The Problem with Certainty

Dead-reckoning from wheel encoders is fast, smooth, and wrong in interesting ways. A
wheel lifted off the ground still reports rotation. A robot shoved sideways by a defender
accumulates position error silently. A collision bounces the gyroscope reading for a
fraction of a second---long enough to corrupt dozens of integration steps.

The tempting response is to also use a camera: mount it on the robot, detect AprilTag
fiducial markers placed around the field, and compute an absolute pose from their known
positions. This vision pipeline is accurate and drift-free, but it is also noisy,
ambiguous, and sometimes just wrong. A single tag seen at a glancing angle can produce
two plausible pose solutions. A distant tag produces a noisier estimate than a close one.

Neither source is better than the other. They are complementary, and what the system
needs is a principled way to decide, moment by moment, how much to trust each one.

= Evidence: A Commutative Monoid

Trust is modeled as *evidence*---a value in $[0, 1]$ where $1$ means "no reason to
doubt" and $0$ means "completely disbelieved." The key design insight is that evidence
values compose multiplicatively:

$ e_1 times.o e_2 = e_1 dot e_2 $

This makes $([0, 1], times.o, 1)$ a *commutative monoid*: the operation is associative
and commutative, and $1$ is the unit (no evidence reduces to full trust by default).
Composing evidence from multiple independent checks yields a value that is no larger
than any individual check---the intersection of all reasons to trust.

A *sensor disagreement* $epsilon gt.eq 0$ (an absolute error between two sources) is
lifted into evidence space by a *Gaussian morphism*:

$ mu_sigma: RR_(>=0) -> [0,1], quad mu_sigma (epsilon) = e^(-epsilon \/ sigma) $

The parameter $sigma$ is the characteristic scale: disagreements much smaller than
$sigma$ produce evidence near $1$; disagreements much larger than $sigma$ collapse
evidence toward $0$. Crucially, $mu_sigma$ is itself a monoid morphism from
$(RR_(>=0), +, 0)$ to $([0,1], times.o, 1)$, since:

$
  mu_sigma (epsilon_1 + epsilon_2) = e^(-(epsilon_1 + epsilon_2)\/sigma)
  = mu_sigma(epsilon_1) times.o mu_sigma(epsilon_2)
$

Additive errors in sensor space translate directly into multiplicative reductions in
trust space. This is not an arbitrary design choice---it is the statement that the
diagram below *commutes*: it does not matter whether you add the errors and then lift,
or lift each error individually and then compose.

#figure(
  diagram(
    node-stroke: 0.5pt,
    node-corner-radius: 3pt,
    spacing: (5em, 2.8em),
    node((0, 0), $(RR_(>=0)^2, +)$),
    node((1, 0), $(RR_(>=0), +, 0)$),
    node((0, 1), $([0,1]^2, times.o)$),
    node((1, 1), $([0,1], times.o, 1)$),
    edge((0, 0), (1, 0), $+$, "->"),
    edge((0, 0), (0, 1), $mu_sigma times mu_sigma$, "->", label-side: left),
    edge((1, 0), (1, 1), $mu_sigma$, "->"),
    edge((0, 1), (1, 1), $times.o$, "->"),
  ),
  caption: [
    The Gaussian morphism $mu_sigma$ as a monoid homomorphism. Both paths from the
    top-left to the bottom-right agree: $mu_sigma(epsilon_1 + epsilon_2)
    = mu_sigma(epsilon_1) times.o mu_sigma(epsilon_2)$. The monoid unit is preserved as
    well: $mu_sigma(0) = e^0 = 1$.
  ],
) <fig-monoid-square>

= Three Gyroscopes and a Consensus

The robot carries three independent measurements of angular velocity $omega$:

- $G_1$: the Pigeon2 IMU on the CANivore bus (Phoenix 6, hardware time-synchronized)
- $G_2$: the NavX2 connected via MXP UART (three-axis MEMS gyroscope)
- $G_3$: the kinematic omega derived from swerve module velocities via $hat(xi)$

No single source is authoritative. Instead, pairwise evidence is computed for each pair:

$
  e_(12) = mu_sigma (|G_1 - G_2|), quad
  e_(13) = mu_sigma (|G_1 - G_3|), quad
  e_(23) = mu_sigma (|G_2 - G_3|)
$

The topology of this agreement network is a triangle: each source is connected to both
others by an evidence edge, and a source's *credibility* is the product of the two edges
incident to it---a source is trusted when it agrees with both partners simultaneously.

#figure(
  cetz.canvas({
    import cetz.draw: *

    let g1 = (0, 2.2)
    let g2 = (-1.9, -1.1)
    let g3 = (1.9, -1.1)

    // Edges (drawn before nodes so nodes sit on top)
    line(g1, g2, stroke: 0.8pt)
    line(g1, g3, stroke: 0.8pt)
    line(g2, g3, stroke: 0.8pt)

    // Edge labels at midpoints, offset outward from centroid (0,0)
    content((-1.3, 0.6), $e_(12)$)
    content((1.3, 0.6), $e_(13)$)
    content((0, -1.55), $e_(23)$)

    // Weight annotations at each vertex
    content((-0.85, 2.05), text(8pt)[$w_1 = e_(12) times.o e_(13)$])
    content((-3.1, -1.1), text(8pt)[$w_2 = e_(12) times.o e_(23)$])
    content((3.1, -1.1), text(8pt)[$w_3 = e_(13) times.o e_(23)$])

    // Node circles with white fill
    circle(g1, radius: 0.46, fill: white, stroke: 0.8pt)
    circle(g2, radius: 0.46, fill: white, stroke: 0.8pt)
    circle(g3, radius: 0.46, fill: white, stroke: 0.8pt)

    // Node math labels
    content(g1, $G_1$)
    content(g2, $G_2$)
    content(g3, $G_3$)

    // Source names outside the triangle
    content((0, 2.95), text(8.5pt)[Pigeon2])
    content((-2.55, -1.75), text(8.5pt)[NavX2])
    content((2.55, -1.75), text(8.5pt)[Kinematics])
  }),
  caption: [
    Pairwise agreement triangle for the three angular velocity sources.
    Each edge carries an evidence value; each node's weight is the product
    of its two incident edges. A source is discredited only when at least
    one of its neighbors disagrees with it.
  ],
) <fig-gyro-triangle>

Each source is then weighted by how well it agrees with _both_ others:

$
  w_1 = e_(12) times.o e_(13), quad w_2 = e_(12) times.o e_(23), quad w_3 = e_(13) times.o e_(23)
$

The *consensus angular velocity* is the trust-weighted mean:

$ hat(omega) = (w_1 G_1 + w_2 G_2 + w_3 G_3) / (w_1 + w_2 + w_3) $

If all three sources badly disagree---$w_1 + w_2 + w_3 < 10^(-9)$---the system falls
back to $G_1$ (the Pigeon2), the most hardware-accurate source. The *overall gyro
consensus evidence* is:

$ e_("gyro") = e_(12) times.o e_(13) times.o e_(23) $

a single number summarizing whether the three rotation sensors are telling a consistent
story.

= The Composite Trust Signal

Gyro consensus is one of several checks that run each control loop. The full set
of evidence sources is:

/ Wheel slip: $e_("slip") = mu_(sigma_"slip")(|omega_"odo" - hat(omega)|)$, where
  $omega_"odo" = Delta theta \/ Delta t$ from the pose delta. When wheel encoders
  report a rotation rate that disagrees with the inertial consensus, at least one
  module has lost contact with the ground.

/ Angular jerk: $e_"jerk" = mu_(sigma_"jerk")(|dot(hat(omega))|)$. A sudden spike in
  angular velocity indicates a collision or tip event. The jerk is computed as the
  finite difference of $hat(omega)$ across loop iterations.

/ Linear bump acceleration: $e_"bump" = mu_(sigma_"bump")(||bold(a)_"world"||)$,
  where $bold(a)_"world"$ is the world-linear acceleration from the NavX2 (converted
  from $g$ to m/s²). Large accelerations in the plane indicate an impact.

/ Input correlation: When the commanded chassis speeds are near zero but the robot
  is moving, something external is pushing it (defensive play). When commanded and
  actual velocities point in opposite directions, the odometry is being driven by a
  force the control system did not produce.

/ Drive motor CAN faults: If all four TalonFX motors report a supply current of
  $-1$ A---the Phoenix sentinel value for a disconnected CAN device---the drivetrain
  is electrically faulted and encoder data is meaningless.

Because $times.o$ is associative, these sources chain into a single composite value in
any order; the code applies them left-to-right, which the diagram below makes concrete.

#figure(
  diagram(
    node-stroke: 0.5pt,
    node-corner-radius: 3pt,
    spacing: (3.2em, 2em),
    node((0, 0), align(center)[
      $e_"slip"$ \
      #text(8pt, fill: luma(90))[wheel slip]
    ]),
    node((1, 0), align(center)[
      $e_"jerk"$ \
      #text(8pt, fill: luma(90))[angular jerk]
    ]),
    node((2, 0), align(center)[
      $e_"bump"$ \
      #text(8pt, fill: luma(90))[bump accel]
    ]),
    node((3, 0), align(center)[
      $e_"input"$ \
      #text(8pt, fill: luma(90))[input corr.]
    ]),
    node((4, 0), $e_"total"$),
    edge((0, 0), (1, 0), $times.o$, "->"),
    edge((1, 0), (2, 0), $times.o$, "->"),
    edge((2, 0), (3, 0), $times.o$, "->"),
    edge((3, 0), (4, 0), "->"),
  ),
  caption: [
    Sequential monoid composition producing the composite odometry trust.
    Because $times.o$ is associative and commutative, the order of composition does
    not affect the result---only which failure modes are active. A CAN fault
    short-circuits this chain by forcing $e_"total" = 0$ regardless.
  ],
) <fig-evidence-chain>

These sources compose into a single *composite odometry trust*:

$ e_"total" = e_"slip" times.o e_"jerk" times.o e_"bump" times.o e_"input" $

with the convention that a CAN fault forces $e_"total" = 0$. This scalar, cached as
`cachedOdometryTrust`, gates every subsequent decision about how much to believe the
wheel encoders.

= The Estimator: Covariance as a Dial

WPILib's `SwerveDrivePoseEstimator` maintains a Kalman-filter-like state: a best-guess
pose $hat(p) in S E(2)$ and a $3 times 3$ covariance matrix $Sigma$ representing
uncertainty in $(x, y, theta)$. Each vision measurement arrives with its own declared
standard deviations---the estimator fuses it with the running state proportionally to
the relative confidences.

The trust system controls the covariance passed with each vision measurement. Two
matrices bracket the range:

$
  Sigma_"best" = "diag"(0.05^2, 0.05^2, 0.04^2), quad
  Sigma_"worst" = "diag"(0.60^2, 0.60^2, 0.40^2)
$

The effective standard deviations are linearly interpolated by a combined trust factor:

$ alpha = 0.2 + 0.8 dot (e_"total" dot e_"target"), quad alpha in [0.2, 1.0] $

$ Sigma_"vision"(alpha) = (1 - alpha) Sigma_"worst" + alpha Sigma_"best" $

#figure(
  cetz.canvas({
    import cetz.draw: *

    // Gradient bar as a rectangle (light on left, dark on right)
    rect(
      (-4.5, -0.3),
      (4.5, 0.3),
      fill: gradient.linear(luma(220), luma(130)),
      stroke: 0.6pt,
    )

    // End tick marks
    line((-4.5, -0.5), (-4.5, 0.5), stroke: 0.8pt)
    line((4.5, -0.5), (4.5, 0.5), stroke: 0.8pt)

    // Alpha labels below the bar
    content((-4.5, -0.75), $alpha = 0.2$, anchor: "north")
    content((4.5, -0.75), $alpha = 1.0$, anchor: "north")

    // Sigma labels above the bar
    content(
      (-4.5, 0.65),
      align(center)[$Sigma_"worst"$ \ #text(
          8pt,
        )[$sigma_x = 0.60$ m, $sigma_theta = 0.40$ rad]],
      anchor: "south",
    )
    content(
      (4.5, 0.65),
      align(center)[$Sigma_"best"$ \ #text(
          8pt,
        )[$sigma_x = 0.05$ m, $sigma_theta = 0.04$ rad]],
      anchor: "south",
    )

    // Floor annotation
    content(
      (-4.5, -1.3),
      text(8pt)[(floor: $e_"total" = 0$ or $e_"target" = 0$)],
      anchor: "north",
    )

    // Ceiling annotation
    content(
      (4.5, -1.3),
      text(8pt)[(ceiling: $e_"total" = 1$ and $e_"target" = 1$)],
      anchor: "north",
    )

    // Horizontal arrow inside bar showing direction
    line(
      (-3.5, 0),
      (3.5, 0),
      stroke: (thickness: 0.8pt, paint: white),
      mark: (end: ">", fill: white),
      anchor: "south",
    )
    content((0, 0), text(
      8pt,
      fill: white,
    )[trust vision less $arrow.l$ $arrow.r$ trust vision more])
  }),
  caption: [
    The covariance interpolation dial. As composite odometry trust $e_"total"$ and
    per-measurement target evidence $e_"target"$ both approach $1$, the vision
    covariance shrinks toward $Sigma_"best"$ and the Kalman filter weights vision
    measurements more heavily. The floor at $alpha = 0.2$ prevents the filter from
    ignoring vision entirely even during a full odometry failure.
  ],
) <fig-covariance-dial>

The $0.2$ floor prevents the estimator from completely ignoring vision even when
odometry is degraded. The per-measurement factor $e_"target"$ accounts for AprilTag
geometry:

$
  e_"target" = cases(
    1 & "if" n_"tags" >= 2,
    1 - a \/ a_"max" & "if" n_"tags" = 1
  )
$

where $a in [0, 1]$ is the *pose ambiguity*---the ratio of the second-best to best
PnP reprojection error. A value near $1$ means two pose solutions are nearly
indistinguishable, so single-tag confidence is low. A value near $0$ means the tag
geometry is clear.

= Vision: The Hardware Stack
Our hardware stack is in a superpoised state of simplicity and uncertainty, we adopted
PhotonVision at the end of 2024 after horrible experiences with the Limelights, and it
has proven to be as accurate if not more than it's closed hardware counterparts. Hence
it's simplicity; however unlike most teams (unless you just happen to be _Mechanical
Advantage_) putting a \$400 Mac Mini on the robot is unheard of -- most teams opt for
the cheap and accessable single board aarch64 computers commonly used for vision/neural
processing purposes. The original plan was to hook up the cameras to a cv object detection
model ontop of our rudimentary localization software, however having two of those cheap
single board computers isn't going to cut it. To maxmize our processing utility we've
decided on the most powerful aarch64 CPU on the market, the Mac M series chips. We stripped
the computer down until it's just it's mainboard and the lights, added a simple 75w DC-DC
converter that filtered the dirty 12V+ incoming power in a more modulated 11.9 volts
which the mac mini happily took. While testing and ensuring the feasibility(i.e. compiling
the Asahi Kernel on nixos) the processor never drew more than 5 amps of current further
solidifying the electrical feasibility of this co-processor. Having daily drove NixOS on
an MacBook Pro M1 (2021) our original plan was to use NixOS
to run PhotonVision, but after weeks of testing and debugging something seems to be
bothering the MrCal tool of PhotonVision which hindered the 3D tracking needed for
odometry/vision measurements to be accurate. So a switch to Fedora 42(Asahi Remix) was
needed. Now, you may ask _why not MacOS?_ to that I say, systemd is way easier to manage
in our enviornment, none of our programming leads have had plesant experiences on MacOS
as daily drivers let alone a server, furthermore, I don't really know if Aqua(the DE)
would be still active while running in a headless state(which would increase the power
draw from our strict power diet) so a decision was made to make sure that it works on
a Linux operating system.


//
// #rect(
//   width: 100%,
//   inset: 12pt,
//   stroke: (dash: "dashed", paint: gray),
// )[
//   *[Author's note: hardware section to be completed.]*
//
//   _This section will describe the physical camera hardware: sensor model, resolution,
//   field of view, lens specifications, mounting positions and angles relative to the robot
//   frame, coprocessor platform (e.g., Orange Pi, Raspberry Pi, Limelight), and the
//   PhotonVision configuration (pipeline type, exposure, gain, tag family, tag
//   decimate/blur settings). The mechanical mounts and cable routing will also be covered._
// ]

= Vision: How It Plugs In

*Perspective-$n$-Point (PnP) pose estimation.* AprilTag fiducial markers are printed
at known positions on the field, encoded in an `AprilTagFieldLayout` (the 2026 Reefscape
Welded field). Each tag corner has a known 3D world coordinate $P_i$. The camera
observes 2D image projections $bold(p)_i$ of those corners. Given the camera intrinsic
matrix $K$ (focal length, principal point), PnP solves for the camera pose
$T in S E(3)$ minimizing total reprojection error:

$ T^* = arg min_T sum_i ||bold(p)_i - pi(T dot P_i)||^2 $

where $pi$ is the pinhole projection function. With a single tag (four corners, four
equations), the system is solvable but can admit two solutions rotated about the tag
plane---this is "tag flipping," measured by the ambiguity ratio. With two or more tags
the system is overdetermined and the solution is globally unique: multi-tag PnP.

*PhotonVision and the Camera abstraction.* Each physical camera runs a PhotonVision
pipeline on a coprocessor. The `PhotonCamera` Java object in the robot code polls the
pipeline results over NetworkTables. A `PhotonPoseEstimator` wraps each camera with its
robot-to-camera transform (a `Transform3d` encoding the exact mount geometry) and
converts `EstimatedRobotPose` results into the robot's world frame.

Each `Camera.update()` call returns a list of `VisionMeasurement` records:
```java
record VisionMeasurement(
    Pose2d pose, double timestampSeconds,
    double distanceMeters, double ambiguity, int numTargets)
```
These records carry everything the trust pipeline needs. The distance gates out
measurements from tags that are too close (unstable angle) or too far (noisy pixels).
The timestamp is used to detect high-latency measurements that arrived after the robot's
pose has drifted significantly.

*The injection interface.* `VisionSubsystem` holds a `VisionMeasurementConsumer`
functional interface---a callback injected at construction time pointing to
`OdometryDrivetrain::addVisionMeasurement`. Each accepted measurement travels through
range filtering, latency filtering, and trust-aware covariance interpolation before
landing in the Kalman estimator. Nothing bypasses the trust pipeline: the base-class
`addVisionMeasurement(Pose2d, double)` overload is intentionally not used anywhere in
the vision path, preventing accidental injection of unvetted measurements.

Two cameras currently run the pipeline (front and back), providing overlapping field
coverage. An auxiliary camera transform (`AUX_CAMERA_TRANSFORM`) is defined for a
potential third camera; wiring it into the subsystem is a one-line addition to the
`cameras` array.

= The System as a Whole

What makes this design satisfying is that the trust architecture is not bolted on top
of a naive estimator---it is part of the same mathematical object. Evidence is a monoid,
and monoids compose freely. Adding a new failure mode means defining a new Gaussian
morphism and composing it with `and()`. Removing a check is equally clean. The algebra
enforces a contract: every evidence value lives in $[0, 1]$, every composition can only
reduce trust, and the unit $1$ means "nothing suspicious detected."

The Kalman estimator sits at the boundary between the algebraic trust world and the
probabilistic estimation world. Evidence in $[0, 1]$ is mapped to covariance matrices
through the linear interpolation, which is admittedly a heuristic bridge---not a
derived statistical relationship. But it is a principled heuristic: monotone, bounded,
and tunable by adjusting two diagonal matrices.

The robot, by the start of autonomous, knows where it is.
