# NoCAP Theorem Spring-Damper System

[](https://godotengine.org)
[](https://dotnet.microsoft.com/)
[](https://docs.microsoft.com/en-us/dotnet/csharp/)
[](https://www.google.com/search?q=LICENSE)
[](https://www.google.com/search?q=https://www.youtube.com/%40NoCapTheorem)

> **A high-fidelity harmonic oscillator simulation engineered in C# and Godot 4.**
> Designed with a strict separation of concerns, this system features dynamic MultiMesh hardware instancing, procedural parametric coil rendering, and a continuous physics-driven orchestrator utilizing damped harmonic oscillator constraints.

---

## Quick Start & Installation

### Prerequisites

* **Godot Engine v4.x** (specifically the **.NET edition**)
* **.NET 8.0 SDK** (or higher)

### Build & Run

Clone the repository and compile the C# solution to ensure the assembly is generated before launching the engine:

```bash
# Build C# solution and execute in Godot
dotnet build && godot

```

---

## License & Channel

Distributed under the **Apache2 License**. See `LICENSE` for more information.

* **No-Cap Theorem Platform:** For deep-dive architectural breakdowns, rigorous systems engineering principles, and video walkthroughs on simulation physics, check out **No-Cap Theorem**.

---

---

# Technical Specification & Documentation

## 1. Architectural Overview

To maintain rigorous systems engineering standards, the codebase strictly separates data, rendering, input, and orchestration into four distinct layers:

1. **Configuration (Data Layer):** A centralized `Resource` defining physical constants, visual scaling, and telemetry tuning.
2. **Procedural Spring (Rendering Layer):** GPU-instanced drawing of connecting coil segments without relying on heavy physics nodes.
3. **Free Camera (Controller Layer):** 6-DoF kinematic camera movement.
4. **Orchestrator (Main Class):** Bootstraps the sub-systems, dynamically updates the physics nodes, applies restorative forces, and broadcasts real-time UDP telemetry metrics.

---

## 2. Technical Breakdown

### Component 1: Harmonic Oscillator Generation & Constraints

The physical simulation consists of a dynamically generated, frozen kinematic anchor and a dynamic rigid body bob.

* **Damping & Collisions:** Both linear and angular damping are strictly set to zero (`DampMode.Replace`) to ensure the system loses energy only through explicitly programmed restorative forces.
* **Force Application:** The orchestrator evaluates the vector between the anchor and bob each physics frame, applying a continuous force according to a damped harmonic oscillator model (Hooke's Law with velocity damping):

$$\mathbf{F}_{\mathrm{total}} =
-k(\Vert{}\mathbf{p}_{\mathrm{bob}} - \mathbf{p}_{\mathrm{anchor}}\Vert{} -
L_{\mathrm{rest}}) \cdot \mathbf{\hat{d}} - c (\mathbf{v}_{\mathrm{bob}} \cdot \mathbf{\hat{d}}) \cdot \mathbf{\hat{d}}$$

Where $k$ is the `SpringStiffness` (default 300.0) and $c$ is the `SpringDamping` (default 1.0).

### Component 2: Hardware-Instanced Rendering & Transform Algebra

Instead of generating unique cylinder meshes for every coil segment, the `CoilRenderer` leverages a single `MultiMeshInstance3D` to draw the entire chain in a single draw call. Shadow casting is explicitly disabled on these thin objects to optimize rendering and prevent artifacting.

To dynamically position and rotate these segments along a parametric helix, the system executes a custom linear algebra routine to construct a 3D rotation matrix (Basis).

1. **Direction Vector (Y-Axis):**

$$\mathbf{y} = \frac{\mathbf{p}_{\mathrm{end}} -
\mathbf{p}_{\mathrm{start}}}{\Vert{}\mathbf{p}_{\mathrm{end}} - \mathbf{p}_{\mathrm{start}}\Vert{}}$$

2. **Singularity Handling (Gimbal Lock Prevention):**
If the computed direction aligns perfectly with the global `Vector3.Up` (evaluated via dot product $> 0.99$), cross-product operations will yield a zero-vector. The system catches this and dynamically pivots the reference axis to `Vector3.Right`:

$$\mathbf{v}_{\mathrm{ref}} =
\begin{cases} \mathrm{Right}, & \mathrm{if } \vert{}\mathbf{y} \cdot \mathrm{Up}\vert{} > 0.99 \\ \mathrm{Up}, & \mathrm{otherwise}
\end{cases}$$

3. **Orthogonal Construction:**

$$\mathbf{x} = \frac{\mathbf{y} \times \mathbf{v}_{\mathrm{ref}}}{\Vert{}\mathbf{y} \times \mathbf{v}_{\mathrm{ref}}\Vert{}}$$

$$\mathbf{z} = \frac{\mathbf{x} \times \mathbf{y}}{\Vert{}\mathbf{x} \times \mathbf{y}\Vert{}}$$

### Component 3: Telemetry & Dynamical Energy Metrics

The orchestrator bypasses standard UI nodes, opting to stream continuous JSON physics metrics via a non-blocking `UdpClient` to external telemetry consumers. The payload isolates specific energetic terms for a proper Lagrangian:

1. **Kinetic Energy:**

$$T = \frac{1}{2} m \Vert{}\mathbf{v}_{\mathrm{bob}}\Vert{}^2$$

2. **Total Potential Energy:** Derived from both gravitational potential (relative to the anchor) and elastic potential:

$$V = m g (\mathbf{p}_{\mathrm{bob}_y} -
\mathbf{p}_{\mathrm{anchor}_y}) + \frac{1}{2} k (\Vert{}\mathbf{p}_{\mathrm{bob}} - \mathbf{p}_{\mathrm{anchor}}\Vert{} -
L_{\mathrm{rest}})^2$$

This ensures the user can track total mechanical energy and the Lagrangian ($T - V$) across the simulation lifespan.

---

## 3. How to Control It

| Category | Input / Key | Action |
| --- | --- | --- |
| **Movement** | `W` / `S` | Move Forward / Backward |
|  | `A` / `D` | Strafe Left / Right |
|  | `SPACE` / `CTRL` | Ascend / Descend |
|  | `SHIFT` | Sprint (2.5x speed multiplier) |
| **System** | `ESC` | Toggle Mouse Capture Mode |
|          | `MOUSE MOTION` | Pitch/Yaw Camera Rotation |
