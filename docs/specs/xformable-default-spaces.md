# Default spaces and winding

`RigExecJoint`, `RigExecControl`, and volume weight providers evaluate their
declared default-space channels through OpenExec. Editing these inputs dirties
their consumers without replacing the rig's binding epoch.

The formulas below use USD's row-vector matrix convention: the leftmost
transform applies first. `R` is the existing orthonormalized rest transform,
`Rp` the nearest namespace provider's rest transform, and `Dp` the resolved
`parent:defaultSpace`. A default offset `O` applies the six `default:*` scalar
channels: XYZ rotations in degrees followed by translation.

```
computed default:space = O * R * inverse(Rp) * Dp
avars:defaultSpace     = default:space            (fallback)
posed:defaultSpace     = avars:defaultSpace       (fallback)
parent:defaultSpace   = parent's computeDefaultFrame (fallback)
parent:space          = parent's computePointFrame   (fallback)
```

An absent namespace parent contributes identity. A provider's public
`computeDefaultFrame` returns the four landmarks of its effective
`default:space`. The matrix properties above can also be requested as ordinary
computed property taps or connected to another declared input. These values are
computed in memory; reading the USD attribute directly still reads its authored
value.

Each matrix property follows the same selection rule: its connected computed
matrix is authoritative, including an identity matrix. Otherwise a non-identity
authored matrix is authoritative. An identity authored matrix selects the
computed fallback. This preserves the existing `posed:space` convention. To
explicitly select world identity instead of an inherited space, connect a
matrix-valued attribute containing identity.

When `posed:space` has no authoritative connection or non-identity authored
value, posing uses:

```
posed = A * posed:defaultSpace * inverse(parent:defaultSpace) * parent:space
```

`A` applies per-axis avar scale, avar rotations in `avars:rotationOrder`,
`avars:rspin` about X, and translation. `avars:unitScaleFactor` multiplies all
three translation avars before composition, converting them to local distance
units. It does not change rotation, scale, or default/rest translation. Volume
weights retain rigid placement and use their `inputs:scale*` shape channels.

With all new channels at their schema defaults, the formula reduces to the
previous `A * R * inverse(Rp) * parentPosed` behavior. Editing a default pose
does not edit the rest frame. Solver-owned joint poses remain authoritative.

For a `RigExecTwistDistribution`, `inputs:twistTurns` adds signed revolutions to
the principal twist extracted from its endpoints. Sample weight `w` receives
an additional `360 * twistTurns * w` degrees around the aim axis. Integer turns
retain the endpoint orientation while describing winding between the endpoints;
fractional values also rotate the end and can be animated or connected.
The C++ builder exposes `SetTwistTurns`, and Python exposes `set_twist_turns`.
