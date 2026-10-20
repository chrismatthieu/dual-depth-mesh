# Dual depth mesh

Shaded mesh from two Intel RealSense depth cameras. Each camera streams depth on its own. Frames inside a distance cutoff (2.5 m by default) are turned into triangles, and the second camera is placed with a rigid offset so both surfaces share one coordinate frame. The window turns slowly on a dark background.

Color is not opened. The mesh is depth only.

This project is the `examples/multicam-mesh` demo extracted from [librealsense](https://github.com/realsenseai/librealsense). It links against an installed librealsense 2.x. It does not contain the SDK.

The cameras do not ship with a pose relative to each other. The offset in the program is the one tuned for this mount (`tx = 0.47 m`, `tz = -0.03 m`, `yaw = 21°`, `pitch = -7°`, `roll = -8°`). A different mount needs **H** and the align keys.

## Build

librealsense 2.59 or newer, GLFW, and OpenGL must already be installed. On this machine librealsense is under `~/.local`, and CMake searches there automatically.

```bash
cmake -S . -B build
cmake --build build
./run-dual-depth.sh
```

Two depth cameras must already be enumerated. Close RealSense Viewer first so nothing else holds the depth nodes.

```bash
./run-dual-depth.sh --range 3
./run-dual-depth.sh --tx 0.47 --yaw 21 --pitch -7 --roll -8 --tz -0.03
```

## What you see

Stand inside the cutoff distance. Anything farther is dropped, so the room falls away and the mesh is whoever or whatever is in front of both cameras. Drag to orbit, scroll to zoom. The automatic turn pauses while the mouse button is down and while the align controls are on screen.

Neighboring depth pixels become a triangle only when they are close in 3D, so an arm does not stretch a sheet across to the background.

## Controls

| Key | Effect |
|---|---|
| Drag | Orbit. Pauses the automatic turn. |
| Scroll | Zoom |
| Space | Reset the view |
| H | Show or hide the align controls |
| − / = | Cutoff distance, 0.1 m per press |
| A / D | Second camera left / right, 1 cm |
| W / S | Second camera in / out, 1 cm |
| I / K | Second camera down / up, 1 cm |
| Q / E | Yaw, 1° |
| T / G | Pitch, 1° |
| Z / X | Roll, 1° |
| [ / ] | Mesh detail (depth decimation 1–8) |
| 0 | Restore the default offset |

Align keys other than **−** / **=** work only while the controls are visible. **−** / **=** always change the range.

## How the two cameras become one mesh

1. Each device starts a depth-only pipeline at 640×480 Z16, 30 fps when that profile exists.
2. Decimation, a spatial smooth, and a hole fill run on the depth frame. Decimation 2 is the default, so the mesh is built from 320×240.
3. The SDK projects that depth image to a grid of 3D points in the camera frame.
4. Camera 0 stays at the origin. Camera 1 is placed with `p0 = Ry(yaw) Rx(pitch) Rz(roll) p1 + t`.
5. Points closer than 0.25 m or farther than the cutoff are discarded. Adjacent pixels that survive, and whose 3D edges are shorter than about 12 cm, become two triangles.
6. Vertex normals are averaged from those triangles and lit from above and in front.

When the same physical surface still appears twice, the offset does not yet match the mount. Press **H** and nudge translation and yaw until the copies land on each other, then press **H** again to hide the controls.
