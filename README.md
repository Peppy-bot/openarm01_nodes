# OpenArm Isaac Sim 6.0.1 — Quick Start

This setup launches complete simulated OpenArm stack with:

- NVIDIA Isaac Sim 6.0.1
- Isaac WebRTC browser viewer
- OpenArm Commander
- OpenArm Scene Commander
- Runtime scene loading
- Runtime object spawning, moving and removal
- Static and dynamic physics
- Runtime force application to dynamic objects
- Simulated OpenArm arms and grippers


Check Peppy:

```bash
peppy info
```

If required, check the Peppy service:

```bash
systemctl --user status peppy.service
```

---

# 2. Register the required Git repositories

Make sure to bring development branches directly up with Peppy to avoid shadowing by main.

```bash
peppy repo add \
  https://github.com/Peppy-bot/openarm-nodes.git \
  --ref feature/isaacsim-6.0.1-clean \
  --top

peppy repo add \
  https://github.com/Peppy-bot/contracts-hub.git \
  --ref modern-jaguar \
  --top

peppy repo add \
  https://github.com/Peppy-bot/launchers-hub.git \
  --ref feature/openarm-isaac-browser-stack \
  --top

peppy repo refresh
```

Check repository configuration then:

```bash
peppy repo list
```

The important repositories should include:

```text
launchers-hub
  ref: feature/openarm-isaac-browser-stack

contracts-hub
  ref: modern-jaguar

openarm-nodes
  ref: feature/isaacsim-6.0.1-clean
```

The OpenArm repository should expose nodes including:

```text
openarm_arm
openarm_arm_sim
openarm_backbone
openarm_commander
openarm_gripper
openarm_gripper_sim
openarm_isaac_webviewer
openarm_scene_commander
openarm_sim_isaac
```

---

# Configure Isaac WebRTC

Find LAN IP address of machine running Isaac Sim:

```bash
hostname -I
```

For example:

```text
192.168.1.120
```

Configure address Isaac should advertise to browser:

```bash
systemctl --user set-environment \
  APPTAINERENV_PEPPY_ISAAC_PUBLIC_IP=192.168.1.120 \
  APPTAINERENV_PEPPY_ISAAC_SIGNAL_PORT=49100 \
  APPTAINERENV_PEPPY_ISAAC_STREAM_PORT=47998
```

Replace `192.168.1.120` with the actual IP of the Isaac machine.

Restart Peppy:

```bash
systemctl --user restart peppy.service
```

Verify:

```bash
systemctl --user show-environment \
| grep APPTAINERENV_PEPPY_ISAAC
```

Expected:

```text
APPTAINERENV_PEPPY_ISAAC_PUBLIC_IP=192.168.1.120
APPTAINERENV_PEPPY_ISAAC_SIGNAL_PORT=49100
APPTAINERENV_PEPPY_ISAAC_STREAM_PORT=47998
```

Do not hard-code the IP address in the repository. The address may change when using DHCP or Wi-Fi.

---

# 4. Launch the complete OpenArm Isaac stack

Launch using registered launcher:

```bash
peppy stack launch openarm_v2 --with=isaac_sim
```

Peppy will resolve, build and launch required nodes.


# Open the Isaac browser viewer

Open:

```text
http://<ISAAC_HOST_IP>:8210
```

For example:

```text
http://192.168.1.120:8210
```

The browser viewer displays live Isaac Sim stream.

WebRTC uses:

```text
8210/TCP     Web viewer
49100/TCP    WebRTC signalling
47998/UDP    WebRTC media stream
```

If viewer opens but the video does not connect so first verify that configured public IP is reachable from browser machine.

---

# Open Scene Commander

Open:

```text
http://<ISAAC_HOST_IP>:8766
```

When working directly on the Isaac machine:

```text
http://127.0.0.1:8766
```

Scene Commander is used to construct and modify Isaac environment while the simulator is running.

## Scene controls

The **Scene** section allows you to:

- Select an Isaac environment
- Set the scene scale
- Load the selected scene
- Clear the runtime scene

Scene assets are provided byIsaac-side asset catalogue so exact catalogue can evolve without changing web application.

Examples include warehouse and grid environments.

Runtime scenes are created under:

```text
/World/RuntimeScene
```

---

# Search and spawn objects

Scene Commander provides an asset search field.

Examples of searchable assets include:

```text
table
can
cracker
drill
block
bottle
```

For a selected object, configure:

```text
X position
Y position
Z position
Scale
Physics
Mass
```

Then press:

```text
Spawn Object
```

Isaac assigns each spawned object a unique ID similar to:

```text
obj_626b16769f97
```

Runtime objects are created under:

```text
/World/RuntimeObjects/<object_id>
```

---

# Object physics modes

Scene Commander supports three physics modes.

## None

```text
Physics = none
```

Use this for assets that should only be placed visually and should not participate in rigid-body physics.

This is useful for:

- initial layout
- visual props
- complex assets during placement testing

## Static

```text
Physics = static
```

Use this for fixed objects such as:

- tables
- shelves
- walls
- fixtures
- work surfaces

Static objects participate in collision but do not move dynamically.

## Dynamic

```text
Physics = dynamic
```

Use this for objects that should:

- fall under gravity
- collide
- be pushed
- be grasped
- be lifted
- receive external forces

Mass is specified in kilograms.

Example:

```text
Physics = dynamic
Mass = 0.5 kg
```

---

# Runtime Object controls

Every spawned object appears in the **Runtime Objects** section.

The object entry shows information such as:

```text
object_id
asset_id
physics mode
mass
position
```

From this section you can:

```text
Move
Remove
Apply Force        dynamic objects only
```

## Move an object

Change its:

```text
X
Y
Z
```

coordinates and press:

```text
Move
```

This changes the object's runtime position without restarting Isaac Sim.

## Remove an object

Press:

```text
Remove
```

to delete that runtime object.

---

# 10. Apply force to a dynamic object

Objects spawned with:

```text
Physics = dynamic
```

provide additional force controls.

The Runtime Object card contains:

```text
Force magnitude (N)
Duration (s)

+X    -X
+Y    -Y
+Z    -Z
```

For example:

```text
Mass       = 0.5 kg
Force      = 1.0 N
Duration   = 0.5 s
Direction  = +Y
```

Press:

```text
+Y
```

to apply:

```text
[0.0, 1.0, 0.0] N
```

in Isaac world frame.

The force is automatically disabled after selected duration.

For lightweight objects, start with small forces.

For example:

```text
0.1 kg object:
0.2 – 0.5 N

0.5 kg object:
0.5 – 2 N
```

Avoid starting very large forces such as `20 N` on lightweight objects.

Force control is currently world-frame XYZ force control.

Torque and mouse-based viewport force manipulators are not currently exposed.

---

# 11. Move the OpenArm robot root

Scene Commander also provides robot-root positioning.

Set:

```text
X
Y
Z
```

and use:

```text
Move Robot
```

to reposition the OpenArm base in the Isaac world.

This is useful when configuring new workcells or positioning the robot relative to tables, shelves and manipulation objects.

---

# Open OpenArm Commander

Open:

```text
http://<ISAAC_HOST_IP>:8765
```

or locally:

```text
http://127.0.0.1:8765
```

OpenArm Commander controls robot itself.



# Typical workflow

A normal interactive session is:

```text
1. Launch the OpenArm Isaac stack

2. Open Isaac Viewer
   http://<IP>:8210

3. Open Scene Commander
   http://<IP>:8766

4. Load an environment

5. Search for a table or fixture

6. Spawn it as Static

7. Search for manipulation objects

8. Spawn them as Dynamic

9. Set appropriate mass

10. Use Runtime Objects to adjust positions

11. Apply small external forces if required

12. Open OpenArm Commander
    http://<IP>:8765

13. Move the arms and grippers

14. Observe everything live through the Isaac Viewer
```


# 20. Ports summary

| Service | Port | Purpose |
|---|---:|---|
| Isaac Viewer | 8210/TCP | Browser interface |
| OpenArm Commander | 8765/TCP | Arm and gripper control |
| Scene Commander | 8766/TCP | Scene/object/physics control |
| WebRTC Signalling | 49100/TCP | Isaac WebRTC signalling |
| WebRTC Stream | 47998/UDP | Isaac WebRTC media |

---

## Current architecture

```text
                       Browser
                          |
         +----------------+----------------+
         |                |                |
         v                v                v
   Isaac Viewer     Scene Commander   OpenArm Commander
      :8210              :8766             :8765
         |                |                |
         |                v                v
         |        openarm_sim_isaac   openarm_backbone
         |                ^                |
         |                |                v
         |                |         simulation relays
         |                |                |
         +----------------+----------------+
                          |
                          v
                      Isaac Sim
```
