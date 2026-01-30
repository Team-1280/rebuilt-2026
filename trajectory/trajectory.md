# REBUILT Trajectory Calculations
Calculations for shooting fuel projectiles for the 2026 FIRST Robotics Competition: REBUILT.

### Goal
The goal is for the shooter to automatically aim to be shoot fuel at a target, without any human operation.

Calculate the unknowns from the given parameters with a closed-form solution (mathematically exact, no iteration). \
These calculations aim to use minimal intuition and simple operations, for accessibility.

### Math Requirements
All of the calculations heavily use vector algebra. The properties of vectors are taught in one of the below sections.

Along with vectors, knowledge of trigonometric and inverse trigonometric functions is important because of the many angles that are present.

Decent physics skills are needed to derive many of the equations.

For the last part of calculations involving rotations, which is more separate and not important to the trajectory calculations, matrix-like transformations are needed, but all definitions will be listed.

Topics:
- Trigonometry, inverse trigonometry
- Vector algebra
- Algebra
- Physics, kinematics

Difficulty: at least *AP Physics C: Mechanics* level.

### Math Preface
Try not to be intimidated by the notation and long rows of calculations.

These calculations are mostly just algebraic manipulations of an initial equation derived from physics.

There are few difficult concepts or complex operations; instead, simple equations are repeatedly transformed and altered with algebraic operations to reach a final useful result.

Fancy tricks, esoteric or technical concepts, and profound or clever insights are all avoided.

The general structure of each section of math is this:
1. Define variables: givens, unknowns, useful definitions
2. Use simple intuition or physics to create an equation with the variables
3. Perform many algebraic operations on the equation one-by-one to change it into a more useful equation
4. The final equation reveals new information, such as an expression for an unknown

## Setup
This section will teach you the definitions and the more advanced math topics that are needed to understand the calculations.

### Terminology and Conventions
We are shooting at a target. The target is either the *hub* when the robot is inside the alliance zone, or the *alliance zone* (via the trench or bump) when the robot is inside the neutral zone.

The *trajectory* of the *projectile* (fuel) starts at the *muzzle* (the point in space where the projectile exits the shooter), and ends at the target. For the duration of the trajectory, the projectile is only influenced by gravity (it is 'ballistic').

A *reference frame* is like a perspective. Imagine two cars moving in the same forward direction, Car A at 30 mph and Car B at 40 mph. From the reference frame (perspective) of Car A, Car B is moving forward away at 10mph. From the reference frame of Car B, Car A is moving backwards at 10mph. \
Coordinate systems are used to define reference frames. They are defined with an origin (where the point $\langle0,0,0\rangle$ is), a rotation (which way the X and Y axes face), and a velocity (how the entire system, or origin, is moving). \
The $z$ axis in 3D space is the vertical (positive is upwards) direction. The $x$ and $y$ axes are horizontal. All axes are perpendicular to each other.

The default coordinate system used for vectors is the field coordinate system (blue origin), abbreviated **FCS**. Just think of it as an unchanging, static, or absolute system. \
The robot coordinate system, abbreviated **RCS**, is relative to the robot. Vectors with this coordinate system are "attached" to or follow the robot's reference frame; its position and rotation. An example is the locations of the robot wheels relative to the robot.


For pitch angle, an angle of $0$ is parallel to the ground. A yaw (turn) angle of $0$ is facing forward.

### Syntax

#### Greek Letters
Names of Greek letters that will be used as variables.
- $\alpha$: alpha (lowercase)
- $\theta$: theta (lowercase)
- $\phi$: phi (lowercase)
- $\sigma$: sigma (lowercase)
- $\Delta$: Delta (uppercase) - means "difference" or "change [in]"

#### Symbols
The $\coloneqq$ (colon equals) symbol means assignment or definition, "is defined as"; true by definition, rather than a result or equality. (It has the same effect as $=$.)
```math
x\coloneqq(\text{distance})
```

The $\implies$ arrow means "implies", "it follows"; deduction, algebra.
```math
x=y^3\implies y=\sqrt[3]{x}
```

The $\hookrightarrow$ arrow shall mean that the equation is derived from intuition or physics, rather than derived algebraically from another equation.
```math
\hookrightarrow F_{net}=F_T-mg\sin\theta
```

$\text{atan2}(y,x)$ is a variant of $\arctan(y/x)$ that takes two separate numbers. The difference is that $\text{atan2}$ can give angles in any of the four quadrants of the 2D plane, while $\arctan$ can only give angles in the rightmost two quadrants.

### Vectors
A *vector* is like a list of components. Each component corresponds to a coordinate. 2D vectors have $x$ and $y$ coordinate components, 3D vectors additionally have a $z$ component. \
A regular number is called a *scalar*.

Vector variables have an arrow on top, e.g. $\vec{v}$, and have components $v_x,v_y,$ and optionally $v_z$. \
Vectors can be directly written out with their components using angled brackets: $\vec{v}_{xyz}=\langle v_x,v_y,v_z\rangle$.

For a 3D vector $\vec{v}_{xyz}$, define $\vec{v}_{xy}$ as the 2D horizontal vector of it.

A *unit vector* is a vector with magnitude $1$. They have a hat on top instead of an arrow, e.g. $\hat{w}$.

Basic vector arithmetic:
```math
c\vec{A}=\vec{A}c=\langle cA_x,\,cA_y,\,\ldots\rangle \\
\vec{A}\pm\vec{B}=\langle A_x\pm B_x,\,A_y\pm B_y,\,\ldots\rangle
```
More vector properties and operations:
```math
\lVert \vec{A} \rVert\coloneqq(\text{magnitude of vector})=\sqrt{A_x^2+A_y^2+\ldots} \\
\vec{A}\cdot\vec{B}\coloneqq(\text{dot product})=A_xB_x+A_yB_y+\ldots=\lVert \vec{A} \rVert\lVert \vec{B} \rVert \cos\theta \\
\implies
\vec{A}\cdot\vec{A}=A_x^2+A_y^2+\ldots=\lVert \vec{A} \rVert^2 \\
\angle\vec{A}_{xy}\coloneqq(\text{angle }\vec{A}\text{ makes with x+ axis})=\text{atan2}(A_y,A_x) \\
\vec{0}\coloneqq(\text{zero vector})=\langle0,0,\ldots\rangle \\
\vec{A}_{xy}\times\vec{B}_{xy}\coloneqq(\text{2D scalar cross product})=A_xB_y-A_yB_x
```

### Rotations
This section can be skipped if necessary as these skills don't appear in the primary trajectory calculations.

An $\boldsymbol{R}$ denotes a rotation (either 2D or 3D). \
Internally and mathematically, they are represented either as transformation matrices or quaternions.

These can transform vectors and other rotations via multiplication. The rotation $\boldsymbol{R}$ is *applied* to the other object.
```math
\boldsymbol{R}\vec{v}=\vec{v}' \\
\boldsymbol{R}_2\boldsymbol{R}_1=\boldsymbol{R}_1'
```

$\boldsymbol{R}(a)$ is a 2D rotation with angle $a$.

$\boldsymbol{R}(a,b,c)$ is a 3D rotation with roll $a$, pitch $b$, yaw $c$. \
The following functions will be defined to access the individual angles:
```math
\text{roll}(\boldsymbol{R})=a,\,\text{pitch}(\boldsymbol{R})=b,\,\text{yaw}(\boldsymbol{R})=c
```
(Yaw is turning side to side, pitch is turning up and down, roll is rotating around while keeping same direction.)

#### Properties

##### Forward Vector
The *forward vector* of $\boldsymbol{R}$ is a unit vector pointing in the direction that an object with rotation $\boldsymbol{R}$ is facing.

Objects that aren't rotated face in the $+x$ axis. \
Let:
```math
\hat{i}\coloneqq(\text{unit forward vector for }\boldsymbol{R}(0,0,0)\,)=\langle1,0,0\rangle \\
\boldsymbol{R}\hat{i}=(\text{unit vector for }\boldsymbol{R})
```

Forward vector for 2D rotation:
```math
\boldsymbol{R}(a)\hat{i}=\langle \cos a,\sin a \rangle
```

Forward vector for 3D rotation:
```math
\boldsymbol{R}(a,b,c)\hat{i}=\langle \cos b\cos c,\cos b\sin c,\sin b \rangle
```
Notice that there is no roll $a$ in the forward vector: this is because roll simply rotates the vector around in-place as if the vector arrow tip was a propeller, without changing its direction.

##### Algebra
This algebra applies mainly to 3D rotations since 2D rotation algebra is trivial.

Let $\boldsymbol{I}$ be the identity rotation; no rotation. \
Then $\boldsymbol{I}\vec{v}=\vec{v}$ and $\boldsymbol{I}\boldsymbol{R}=\boldsymbol{R}$.

Let $\boldsymbol{R}^{-1}$ be the inverse rotation of $\boldsymbol{R}$. \
Then, $\boldsymbol{R}^{-1}\boldsymbol{R}=\boldsymbol{R}\boldsymbol{R}^{-1}=\boldsymbol{I}$.

Rotation multiplication follows the associative property:
```math
(\boldsymbol{R}_1\boldsymbol{R}_2)\boldsymbol{R}_3=\boldsymbol{R}_1(\boldsymbol{R}_2\boldsymbol{R}_3) \\
(\boldsymbol{R}_1\boldsymbol{R}_2)\vec{v}=\boldsymbol{R}_1(\boldsymbol{R}_2\vec{v})
```

3D rotations do NOT follow the commutative property in general:
$$\boldsymbol{R}_1\boldsymbol{R}_2\ne\boldsymbol{R}_2\boldsymbol{R}_1$$

### Kinematics Equations
We will use simple equations that describe how position, velocity, and acceleration are related. \

For constant velocity:
```math
\Delta x=vt \\
v=\frac{\Delta x}{t}\\
```

For constant acceleration:
```math
\Delta v=at \\
a=\frac{\Delta v}{t} \\
```

Kinematics motion equation, for constant acceleration, where $v_0$ is initial velocity:
```math
\Delta x=v_0t+\frac{1}{2}at^2
```

## Variables
Try to remember all of the parameter symbols.

### Givens
These are the given variables that are used to calculate the unknowns.
```math
\begin{align*}
    &\vec{P}_{xyz}\coloneqq(\text{robot position}) \\
    &\vec{V}_{xy}\coloneqq(\text{robot velocity}) \\
    &\boldsymbol{R}_R\coloneqq(\text{robot 3D rotation}) \\
    &\vec{T}_{xyz}\coloneqq(\text{target position})\\
    &\vec{Q}_{xyz}\coloneqq(\text{shooter position, RCS}) \\
    &u\coloneqq(\text{projectile exit speed}) \\
    &g\coloneqq(\text{gravitational acceleration constant})\approx9.8m/s^2 \\
\end{align*}
```

### Unknowns
We are solving for the unknown variables in terms of the given parameters.
```math
\begin{align*}
    &\theta\coloneqq(\text{shooter pitch, FCS}) \\
    &\alpha\coloneqq(\text{shooter yaw, FCS}) \\
\end{align*}
```
These must then be converted to RCS.
```math
\begin{align*}
    &\alpha_r\coloneqq(\text{shooter yaw, RCS}) \\
    &\theta_r\coloneqq(\text{shooter pitch, RCS}) \\
\end{align*}
```


### Notes
The speed of the target position shall always be 0; none of our targets are moving objects whose velocity needs to be accounted for. \
The shooter has $0$ roll; it cannot roll. \
It is expected that the shooter can always match the desired $\theta$ and $\alpha$ with negligible latency.

#### Robot Velocity
Robot velocity $\vec{V}$ is necessary to know because the robot's velocity contributes to the fuel's velocity. As an example, imagine dropping a heavy object from a moving plane versus from a floating helicopter right when it is directly above a target on the ground. The object dropped from the plane will miss because it has forward velocity inherited from the plane.

#### Projectile Exit Speed
We shall keep $u$ as a parameter, instead of using it as a controllable unknown in our calculations.

The exit speed $u$ depends on $\theta$ due to the shooting mechanism:
```math
u=f(\theta)
```
We will ignore this for the purposes of the calculations. This problem must be resolved in the code.

## Calculations

### 1. Setup

#### Displacement
Many of the given parameters are just used to calculate the (known) displacement (muzzle) from the projectile exit point to the target.

```math
\vec{S}_{xyz}\coloneqq(\text{target displacement})
```
```math
\begin{align*}
    &\vec{M}_{xyz}\coloneqq(\text{muzzle position, FCS})=\vec{P}+\boldsymbol{R}_R\vec{Q} \\
    &\vec{S}=\vec{T}-\vec{M} \\
    (\text{substitute for }\vec{M})\implies&\vec{S}=\vec{T}-(\vec{P}+\boldsymbol{R}_R\vec{Q})
\end{align*}
```
Note that there are operations available in code for applying the robot pose to the shooter position, to get the muzzle position.

#### Exit Velocity
The initial exit velocity $\vec{v}$ is the shoot vector with magnitude $u$ at angles $\theta$ and $\alpha$, plus the contributing robot velocity. It is unknown.
```math
\begin{align*}
    &\hat{w}\coloneqq(\text{unit shoot direction})\coloneqq\boldsymbol{R}(0,\theta,\alpha)\hat{i} \\
    (\text{forward vector})\implies&\hat{w}=\langle\cos\theta\cos\alpha,\cos\theta\sin\alpha,\sin\theta\rangle \\
    &\vec{v}_{xyz}\coloneqq(\text{projectile exit velocity at muzzle, FCS})=u\hat{w}+\vec{V} \\
\end{align*}
```
```math
\begin{align*}
    &v_x=u\cos\theta\cos\alpha+V_x \\
    &v_y=u\cos\theta\sin\alpha+V_y \\
    &v_z=u\sin\theta \\
\end{align*}
```

#### Time
Define $t$ as the time from projectile exit to hitting the target. The main plan of the calculations is to determine $t$ and then use it to directly calculate $\alpha$ and $\theta$.
```math
\begin{align*}
    &t\coloneqq(\text{trajectory duration}) \\
    &t\in\mathbb{R}^+\enspace(\text{must be real and positive})
\end{align*}
```

#### Gravity Vector
For convenience, let $\vec{g}$ be the gravitational acceleration vector. Also note that this is the only acceleration acting on the projectile during the trajectory.
```math
\vec{g}\coloneqq\langle0,0,-g\rangle
```

### 2. Find $t$

#### Vector Kinetmatics Equation
Start with the kinematics motion equation:
```math
\hookrightarrow\vec{S}=\vec{v}t+\frac{1}{2}\vec{g}t^2
```
Rearrange:
```math
\begin{align*}
    (\text{substitute }\vec{v})\implies&\vec{S}=(u\hat{w}+\vec{V})t+\frac{1}{2}\vec{g}t^2 \\
    (\text{distribute }t)\implies&\vec{S}=u\hat{w}t+\vec{V}t+\frac{1}{2}\vec{g}t^2 \\
    (\text{move terms})\implies&\vec{S}-\frac{1}{2}\vec{g}t^2-\vec{V}t=u\hat{w}t \\
\end{align*}
```

#### Eliminate Other Unknowns $\alpha$, $\theta$
The only other unknown variable in the equation is $\hat{w}$, which contains both $\alpha$ and $\theta$ unknowns. Use vector magnitude to eliminate $\hat{w}$, since $\lVert\hat{w}\rVert=1$.

Since these two vectors are equal, their magnitudes are equal. Take the magnitude of both sides:
```math
(\text{take magnitude of sides})\implies\left\lVert\vec{S}-\frac{1}{2}\vec{g}t^2-\vec{V}t\right\rVert=\lVert u\hat{w}t \rVert \\
```
```math
(\text{right side: pull out scalars})\quad\lVert u\hat{w}t \rVert=u\lVert\hat{w}\rVert t=u(1)t=ut
```
```math
(\text{substitute right side back in})\implies\left\lVert\vec{S}-\frac{1}{2}\vec{g}t^2-\vec{V}t\right\rVert=ut \\
```
The result is an equation with only one unknown variable, $t$.

#### Convert to Scalar Quartic
The left side of the equation has a vector with components and magnitude:
```math
(\text{left side vector})\quad(\vec{S}-\frac{1}{2}\vec{g}t^2-\vec{V}t)\,=\,\langle S_x-V_xt,S_y-V_yt,S_z+\frac{1}{2}gt^2 \rangle
```
```math
(\text{magnitude formula})\quad\lVert\vec{S}-\frac{1}{2}\vec{g}t^2-\vec{V}t\rVert=\sqrt{(S_x-V_xt)^2+(S_y-V_yt)^2+(S_z+\frac{1}{2}gt^2)^2} \\
```
Substitute in the formula for the magnitude of the vector on the left.
```math
\begin{align*}
    (\text{substitute magnitude on left})\implies&\sqrt{(S_x-V_xt)^2+(S_y-V_yt)^2+(S_z+\frac{1}{2}gt^2)^2}=ut \\
    (\text{square sides})\implies&(S_x-V_xt)^2+(S_y-V_yt)^2+(S_z+\frac{1}{2}gt^2)^2=u^2t^2 \\
\end{align*}
```
Expand and rearrange into standard quartic form:
```math
\begin{align*}
    (\text{apply squaring to each group})\implies&(S_x^2-2S_xV_xt+V_x^2t^2)+(S_y^2-2S_yV_yt+V_y^2t^2)+(S_z^2+S_zgt^2+\frac{1}{4}g^2t^4)=u^2t^2 \\
    (\text{organize terms by }t\text{ factor})\implies&\frac{g^2}{4}t^4+S_zgt^2+V_x^2t^2+V_y^2t^2-u^2t^2-2S_xV_xt-2S_yV_yt+S_x^2+S_y^2+S_z^2=0 \\
    (\text{factor out each }t\text{ power})\implies&(\frac{g^2}{4})t^4+(S_zg+V_x^2+V_y^2-u^2)t^2-2(S_xV_x+S_yV_y)t+(S_x^2+S_y^2+S_z^2)=0 \\
\end{align*}
```
Finally, simplify with vector operations:
```math
\text{By vector properties:} \\
V_x^2+V_y^2=\lVert\vec{V}\rVert^2 \\
S_x^2+S_y^2+S_z^2=\lVert\vec{S}\rVert^2 \\
S_xV_x+S_yV_y=S_xV_x+S_yV_y+(S_z\cdot0)=\vec{S}\cdot\vec{V} \\
```
```math
(\text{substitute})\implies\boxed{(\frac{g^2}{4})t^4+(S_zg+\lVert\vec{V}\rVert^2-u^2)t^2-2(\vec{S}\cdot\vec{V})t+\lVert\vec{S}\rVert^2=0}
```

#### Analysis
We now have a quartic polynomial in $t$ containing no unknowns. \
In standard form, the coefficients are as follows:
```math
At^4+Bt^3+Ct^2+Dt+E=0 \\
```
```math
\begin{align*}
    &A=\frac{1}{4}g^2 \\
    &B=0 \\
    &C=S_zg+\lVert\vec{V}\rVert^2-u^2 \\
    &D=-2(\vec{S}\cdot\vec{V}) \\
    &E=\lVert\vec{S}\rVert^2
\end{align*}
```
There may be 0, 2, or 4 real solutions for $t$, but there likely can be only either 0 or 2 real positive solutions for $t$, corresponding to a high and low trajectory.

A quartic always has a closed-form solution. Using Ferrari's method for the quartic along with either Cardano's method or the trigonometric method for the resolvent cubic can yield these real solutions without using complex numbers in any of the steps.

The larger values of $t$ correspond to steeper trajectories (lobs) with a larger $\theta$. These are ideal for the hub.

The smaller values of $t$ correspond to shallow, direct trajectories that may be used for other targets.

### 3. Find $\alpha$ and $\theta$

First, define $\vec{L}$ as the horizontal ($x$ and $y$ directions only) "effective displacement" vector. \
This comes from hypothetically changing the reference frame from being still to having velocity $\vec{V}$.

The situation where the robot moves while the target is still is **perfectly analogous** to the situation where the robot is still while the target moves (with opposite velocity). \
It is like 'subtracting $\vec{V}$ from both sides'.

Now, the situation is like we are attempting to shoot a moving target with velocity $-\vec{V}$, from a static shooter, so we must predict the target's future position. \
Define the useful vector of the effective horizontal displacement:
```math
\vec{L}_{xy}\coloneqq\vec{S}_{xy}-\vec{V}t=\langle S_x-V_xt,S_y-V_yt\rangle
```

#### Calculating $\alpha$ from $t$
Start with the definition of constant velocity:
```math
\hookrightarrow\vec{v}_{xy}=\frac{\vec{S}_{xy}}{t}
```
```math
\begin{align*}
    (\text{subtitute for }\vec{v}_{xy})\implies&\langle u\cos\theta\cos\alpha+V_x,u\cos\theta\sin\alpha+V_y \rangle=\frac{\vec{S}_{xy}}{t} \\
    (\text{subtract }\vec{V}\text{ from sides})\implies&\langle u\cos\theta\cos\alpha,u\cos\theta\sin\alpha \rangle=\frac{\vec{S}_{xy}}{t}-\vec{V} \\
    (\text{multiply sides by }t)\implies&\langle u\cos\theta\cos\alpha,u\cos\theta\sin\alpha \rangle=\vec{S}_{xy}-\vec{V}t \\
    (\text{subtitute }\vec{L}\text{ in})\implies&\langle u\cos\theta\cos\alpha,u\cos\theta\sin\alpha \rangle=\vec{L} \\
    (\text{take angle of sides})\implies&\angle\langle u\cos\theta\cos\alpha,u\cos\theta\sin\alpha \rangle=\angle\vec{L} \\
    (\text{use definition of }\angle)\implies&\arctan(\frac{u\cos\theta\sin\alpha}{u\cos\theta\cos\alpha})=\angle\vec{L} \\
    (\text{simplify fraction})\implies&\arctan(\tan\alpha)=\angle\vec{L} \\
    (\text{inverse functions cancel})\implies&\boxed{\alpha=\angle\vec{L}}
\end{align*}
```

#### Calculating $\theta$ from $t$
For the $z$ (vertical) component start with the kinematics equation and rearrange:
```math
\hookrightarrow S_z=v_zt-\frac{1}{2}gt^2
```
```math
\begin{align*}
    (\text{move around terms})\implies&v_zt=S_z+\frac{1}{2}gt^2 \\
    (\text{substitute for }v_z)\implies&ut\sin\theta=S_z+\frac{1}{2}gt^2 \\
\end{align*}
```
Save this last equation for later.

Next, for the $xy$ (horizontal) component, considered together, use the velocity definition. Note that $\lVert\vec{w}_{xy}\rVert=\cos\theta$.
```math
\hookrightarrow\vec{v}_{xy}=\frac{\vec{S}_{xy}}{t} \\
```
```math
\begin{align*}
    (\text{substitute for }\vec{v}_{xy})\implies&u\hat{w}_{xy}+\vec{V}=\frac{\vec{S}_{xy}}{t} \\
    (\text{multiply sides by }t)\implies&ut\hat{w}_{xy}+\vec{V}t=\vec{S}_{xy} \\
    (\text{subtract }\vec{V}t\text{ from sides})\implies&ut\hat{w}_{xy}=\vec{S}_{xy}-\vec{V}t \\
    (\text{substitute }\vec{L}\text{ in})\implies&ut\hat{w}_{xy}=\vec{L} \\
    (\text{take magnitude of sides})\implies&\lVert ut\hat{w}_{xy}\rVert=\lVert\vec{L}\rVert \\
    (\text{factor out scalars on left side})\implies&ut\lVert\hat{w}_{xy}\rVert=\lVert\vec{L}\rVert \\
    (\text{use }\,\lVert\vec{w}_{xy}\rVert=\cos\theta\,) \implies&ut\cos\theta=\lVert\vec{L}\rVert
\end{align*}
```
Take this last equation, and the one we saved earlier, and divide them by each other:
```math
ut\sin\theta=S_z+\frac{1}{2}gt^2,\quad ut\cos\theta=\lVert\vec{L}\rVert
```
```math
(\text{divide equations})\implies\frac{ut\sin\theta}{ut\cos\theta}=\frac{S_z+\frac{1}{2}gt^2}{\lVert\vec{L}\rVert}
```
```math
\begin{align*}
    (\text{simplify left})\implies&\tan\theta=\frac{S_z+\frac{1}{2}gt^2}{\lVert\vec{L}\rVert} \\
    (\text{take atan2 of sides})\implies&\boxed{\theta=\text{atan2}(S_z+\frac{1}{2}gt^2,\lVert\vec{L}\rVert)} \\
\end{align*}
```
(Since $\theta$ is physically always in the interval $[0,\frac{\pi}{4}]$, there is no need to find other valid angles.)

### 4. Convert to Robot Coordinate System
Since $\alpha$ and $\theta$ are in field coordinate system, they must first be converted to the robot coordinate system to be usable by the robot. \
Find unknowns $\alpha_r,\theta_r$ from $\alpha,\theta$.

#### Common Case
If $\text{pitch}(\boldsymbol{R}_R)=\text{roll}(\boldsymbol{R}_R)=0$ (the robot is flat on the ground), then the calculations are trivial. By intuition:
```math
\begin{align*}
    &\hookrightarrow\boxed{\alpha_r=\alpha-\text{yaw}(\boldsymbol{R}_R)} \\
    &\hookrightarrow\boxed{\theta_r=\theta} \\
\end{align*}
```

#### Generalization
To generalize to any arbitrary $\boldsymbol{R}_R$, some rotation algebra must be done.

Define or recall:
```math
\begin{align*}
    &\boldsymbol{R}_R\coloneqq(\text{robot 3D rotation, FCS}) \\
    &\hat{w}\coloneqq(\text{unit shoot direction})=\boldsymbol{R}(0,\theta,\alpha)\hat{i} \\
    &\boldsymbol{R}_S\coloneqq(\text{shooter rotation, RCS})=\boldsymbol{R}(0,\theta_r,\alpha_r) \\
\end{align*}
```

The direction of $\boldsymbol{R}_S$, after being converted to FCS by applying $\boldsymbol{R}_R$, is $\hat{w}$:
```math
\hookrightarrow\boldsymbol{R}_R\boldsymbol{R}_S\hat{i}=\hat{w}
```
```math
\begin{align*}
    (\text{apply }\boldsymbol{R}_R^{-1}\text{ to sides})\implies&\boldsymbol{R}_R^{-1}(\boldsymbol{R}_R\boldsymbol{R}_S\hat{i})=\boldsymbol{R}_R^{-1}(\hat{w}) \\
    (\text{associative property})\implies&(\boldsymbol{R}_R^{-1}\boldsymbol{R}_R)\boldsymbol{R}_S\hat{i}=\boldsymbol{R}_R^{-1}\hat{w} \\
    (\text{substitute }\boldsymbol{R}_R^{-1}\boldsymbol{R}_R=\boldsymbol{I})\implies&(\boldsymbol{I})\boldsymbol{R}_S\hat{i}=\boldsymbol{R}_R^{-1}\hat{w} \\
    (\text{applying }\boldsymbol{I}\text{ has no effect})\implies&\boldsymbol{R}_S\hat{i}=\boldsymbol{R}_R^{-1}\hat{w} \\
\end{align*}
```
We find that the direction of $\boldsymbol{R}_S$ is $\boldsymbol{R}_R^{-1}\hat{w}$.

Define $\hat{d}$ as this desired direction:
```math
\hat{d}\coloneqq\boldsymbol{R}_R^{-1}\hat{w}=(\text{forward vector of }\boldsymbol{R}_S\,)
```
Simply use the components of $\hat{d}$ to find $\theta_r$ and $\alpha_r$, keeping $\text{roll}(\boldsymbol{R}_S)=0$.
```math
\begin{align*}
    &\hookrightarrow\theta_r=\text{pitch}(\boldsymbol{R}_S)=\arcsin(\frac{-d_z}{\lVert\hat{d}\rVert}) \\
    &\hookrightarrow\alpha_r=\text{yaw}(\boldsymbol{R}_S)=\angle{\vec{d}_{xy}} \\
\end{align*}
```
```math
\begin{align*}
    &\boxed{\theta_r=\arcsin(-d_z)} \\
    &\boxed{\alpha_r=\text{atan2}(d_y,d_x)}
\end{align*}
```
(Due to things with the reference frame of 3D rotations, the $z$ component is the *negated* $\sin$ of the pitch angle.)

## Solution
Summary of findings, without intermediate calculations.

Given:
```math
\vec{P}_{xyz}, \vec{V}_{xy},\boldsymbol{R}_R,\vec{T}_{xyz},\vec{Q}_{xyz},u,g
```
To find:
```math
\alpha,\enspace\theta
```
Define:
```math
\vec{S}_{xyz}=\vec{T}-(\vec{P}+\boldsymbol{R}_R\vec{Q})
```
First, find $t$ using a (depressed) quartic formula. There may be multiple solutions, or no solution.
```math
(\frac{g^2}{4})t^4+(S_zg+\lVert\vec{V}\rVert^2-u^2)t^2-2(\vec{S}\cdot\vec{V})t+\lVert\vec{S}\rVert^2=0 \\
```
```math
t\in\mathbb{R}^+\enspace(t>0)
```
Choose a $t$ based on its size, with larger $t$ having a steeper trajectory. \
If there is no valid $t$, fall back to a default direction.

Calculate the unknowns:
```math
\vec{L}_{xy}\coloneqq\vec{S}_{xy}-\vec{V}t
```
```math
\boxed{\alpha=\angle\vec{L}}
```
```math
\boxed{\theta=\text{atan2}(S_z+\frac{1}{2}gt^2,\lVert\vec{L}\rVert)}
```
If robot is not tilted ($\text{pitch}(\boldsymbol{R}_R)=\text{roll}(\boldsymbol{R}_R)=0$):
```math
\begin{align*}
    &\boxed{\alpha_r=\alpha-\text{yaw}(\boldsymbol{R}_R)} \\
    &\boxed{\theta_r=\theta} \\
\end{align*}
```
But in general, for any robot rotation:
```math
\hat{w}=\langle\cos\theta\cos\alpha,\cos\theta\sin\alpha,\sin\theta\rangle \\
\hat{d}\coloneqq\boldsymbol{R}_R^{-1}\hat{w} \\
```
```math
\begin{align*}
    &\boxed{\theta_r=\arcsin(-d_z)} \\
    &\boxed{\alpha_r=\text{atan2}(d_y,d_x)}
\end{align*}
```

## Extra Case: If $\theta$ is a Parameter
If $\theta$ is known (chosen), then $\alpha$ can easily be calculated directly, because the magnitude of the horizontal shoot velocity is known.

First, notice that $\alpha$ is completely independent of the $z$ axis; heights and vertical velocities do not affect the $\alpha$ solution. Ignore all $z$ vector components and directions.

### Problem Description

Given:
```math
u,\,\theta,\,\vec{V},\,\vec{S}_{xy}
```
Find $\alpha$ such that $\vec{v}_{xy}$ and $\vec{S}_{xy}$ are parallel.

### Calculations
For conciseness, define:
```math
\phi\coloneqq\angle\vec{S} \\
\sigma\coloneqq u\cos\theta \\
```
Then:
```math
\vec{v}_{xy}=\langle \sigma\cos\alpha+V_x,\sigma\sin\alpha+V_y \rangle \\
```

One way to check if two vectors are parallel is to check that their slopes are the same:
```math
\begin{align*}
    &\frac{v_y}{v_x}=\frac{S_y}{S_x} \\
    (\text{substitute for all values})\implies&\frac{\sigma\sin\alpha+V_y}{\sigma\cos\alpha+V_x}=\frac{\lVert\vec{S}\rVert\sin\phi}{\lVert\vec{S}\rVert\cos\phi}=\frac{\sin\phi}{\cos\phi} \\
    (\text{cross multiply})\implies&(\sigma\sin\alpha+V_y)\cos\phi=(\sigma\cos\alpha+V_x)\sin\phi \\
    (\text{distribute})\implies&\sigma\sin\alpha\cos\phi+V_y\cos\phi=\sigma\cos\alpha\sin\phi+V_x\sin\phi \\
    (\text{rearrange terms})\implies&\sigma\sin\alpha\cos\phi-\sigma\cos\alpha\sin\phi=V_x\sin\phi-V_y\cos\phi \\
    (\text{divide by }\sigma)\implies&\sin\alpha\cos\phi-\cos\alpha\sin\phi=\frac{V_x\sin\phi-V_y\cos\phi}{\sigma}
\end{align*}
```
Use the sine difference identity:
```math
\sin(\alpha-\phi)=\sin\alpha\cos\phi-\cos\alpha\sin\phi
```
Substitute and solve for $\alpha$.
```math
\begin{align*}
    (\text{subsitute with identity})\implies&\sin(\alpha-\phi)=\frac{\vec{V}\times\vec{S}}{\sigma\lVert\vec{S}\rVert} \\
    (\text{take arcsin of sides})\implies&\alpha-\phi=\arcsin(\frac{\vec{V}\times\vec{S}}{\sigma\lVert\vec{S}\rVert}) \\
    (\text{move }\phi\text{ to other side})\implies&\alpha=\phi+\arcsin(\frac{\vec{V}\times\vec{S}}{\sigma\lVert\vec{S}\rVert}) \\
\end{align*} 
```
We will ignore the second branch of $\arcsin$ giving a second $\alpha_2$ (corresponding to a supplementary angle facing in the wrong direction).
```math
\boxed{\alpha=\angle\vec{S}+\arcsin(\frac{\vec{V}\times\vec{S}}{u\cos(\theta)\lVert\vec{S}\rVert})}
```
A real solution for $\alpha$ exists when the $\arcsin$ argument is in $\arcsin$'s domain (valid inputs) of $[-1, 1]$:
```math
\left\lvert\frac{\vec{V}\times\vec{S}}{u\cos(\theta)\lVert\vec{S}\rVert}\right\rvert\le1
```

## Physical Considerations

### Invalid Trajectories
Some trajectories are physically impossible because of obstacles.

Primarily, the hub walls require steep trajectories.

One way to validate the trajectory is to check that the end of the trajectory is at a steep enough downwards angle. First, take $\vec{v}_{xyz}$. Then calculate final velocity $\vec{v}_f=\vec{v}+at$

### Sources of Error
This is a list of possible sources of error, sorted by most significant to least significant.

The shoot speed of the projectile likely varies due to natural variance in the projectile and flywheel speed. Shooting many projectiles quickly may affect the flywheel speed, but this has to be tested. The variability of projectile speeds should be determined. The error likely increases with higher shoot speeds. This is a large source of error.

Projectile air resistance (drag) is likely a large source of error, causing undershooting. The error increases drastically with shoot speed. However, research needs to be done to examine its impact.

The magnus effect for backspin applies since the projectile is given backspin by the shooting mechanism. This generates lift, which causes overshooting. The error increases for greater backspin. It is unknown how significant this source of error is.

Error in data such as robot pose and robot velocity yield proportional shooting error. But robot odometry and vision should be able to keep the accuracy of these within suitable error bounds for shooting.

The muzzle point changes when shooter pitch changes, but this error is always only a couple of inches and so is negligible. The shooter position $\vec{Q}$ should be chosen well to reduce this error.

The velocity from the shooter rotating and aiming likely has negligible impact on the projectile's velocity, but this should be tested. The error increases with faster shooter rotation.

Most of the time, $V_z=0$. But in any case, it should be completely negligible. The tower climb is likely slow. The hub cannot be targeted from the bump.

### Reducing and Compensating for Error
Having a greater $\theta$ (steeper trajectory) makes the error have a lesser impact on whether the fuel makes it into the hub.

Reducing exit speed reduces many sources of error.

___

## Attribution
This file was written by Anthony Le, [@aatle](https://github.com/aatle) on GitHub.

Some techniques used in the calculations were guided with the help of an AI, GPT-5. (None of the calculations were written with AI.)

## License
REBUILT Trajectory Calculations  © 2026 by Anthony Le is licensed under Creative Commons Attribution 4.0 International. To view a copy of this license, visit https://creativecommons.org/licenses/by/4.0/
