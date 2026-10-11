# Running the field robot simulation in Docker

Everything the simulation needs — ROS 2 Jazzy, Gazebo Harmonic, MoveIt, the
UR5e description — is inside the container. Nothing has to be installed on the
host except Docker itself.

Tested on Ubuntu 24.04 with integrated AMD graphics. Any Linux with Docker and
an X11 session should work.

## Once, on a new machine

Install Docker from its own repository (Ubuntu's `docker.io` package does not
include Compose v2):

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
                        docker-buildx-plugin docker-compose-plugin

sudo usermod -aG docker $USER
```

**Log out and back in** for the group change to take effect, then check:

```bash
docker run --rm hello-world
docker compose version
```

## Starting the simulation

```bash
git clone <repository-url>
cd field_robot_project
chmod +x docker/*.sh
./docker/run.sh
```

The first run downloads and builds the image: roughly 5 GB and 20–40 minutes,
almost all of it waiting on downloads. Every run after that starts in seconds.

Then open a terminal inside the container and build the workspace:

```bash
./docker/shell.sh
cb                      # alias for: colcon build --symlink-install && source install/setup.bash
```

`cb` takes under a minute and is only needed after changing `CMakeLists.txt`
or adding files. Edits to `.py`, `.xacro`, `.yaml` and `.sdf` take effect
without rebuilding.

## The four terminals

Run `./docker/shell.sh` once per terminal — each call opens a new shell in the
same running container.

```bash
# terminal 1 - simulation
ros2 launch field_robot simulation.launch.py world:=tomato_field

# terminal 2 - motion planning (wait for Gazebo to finish opening)
ros2 launch field_robot moveit.launch.py

# terminal 3 - the two tomato detectors
ros2 launch field_robot perception.launch.py

# terminal 4 - drive the robot up to a row of plants
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

With a tomato 0.35–0.75 m from the arm's shoulder, in a fifth terminal:

```bash
ros2 run field_robot pick_tomato.py
```

## Stopping

```bash
./docker/stop.sh        # stop; the compiled workspace is kept
./docker/stop.sh -v     # stop and throw the build away too
```

## How it is put together

| Piece | What it does |
| --- | --- |
| `Dockerfile` | ROS 2 Jazzy base plus every package this project uses |
| `docker-compose.yml` | Mounts, X11, GPU, networking |
| `docker/run.sh` | Grants X access, passes the host's ids, starts the container |
| `docker/shell.sh` | One terminal inside it |
| `docker/stop.sh` | Stops it |
| `docker/entrypoint.sh` | Sources ROS for the container's main process |

Three choices worth knowing about:

**The source is mounted, the build output is not.** The repository appears
inside the container at `/ws/src/field_robot_project`, live, so editing a file
on the host changes it in the container at once. `build/`, `install/` and
`log/` live in Docker named volumes instead: they are specific to the
container's architecture and paths, and mixing them with a build made on the
host causes failures that are hard to read.

**The container user shares the host user's numeric id.** `docker/run.sh`
passes it in at build time. Without this, every file the container writes into
the mounted workspace comes back owned by root and cannot be edited on the
host — the single most common annoyance with Docker and ROS.

**Rendering goes through `/dev/dri`.** That covers AMD and Intel integrated
graphics. An NVIDIA card needs the NVIDIA Container Toolkit and a different
runtime setting; this setup does not do that.

## When something does not work

**No window appears.** X access was not granted. `./docker/run.sh` does it,
but if the container was started another way:

```bash
xhost +local:docker
```

**`cannot open display`.** `DISPLAY` was empty when the container started.
Check `echo $DISPLAY` on the host — it should be something like `:0` or `:1`.
This also happens over plain SSH, which carries no display.

**Gazebo opens but is very slow.** Hardware rendering is not reaching the
container. Check inside it:

```bash
glxinfo -B | grep "OpenGL renderer"
```

A renderer named `llvmpipe` or `softpipe` means software rendering. Usually
the `render` group id differs on this host; `docker/run.sh` reads it, so start
through the script. If the host simply has no usable GPU, accept the software
path:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./docker/run.sh
```

**Files owned by root after building.** The container was built with the wrong
user id. Rebuild through the script:

```bash
./docker/stop.sh
./docker/run.sh
```

**`permission denied` running the scripts.** Git and downloads often drop the
executable bit:

```bash
chmod +x docker/*.sh
```

**A leftover simulation from a previous run.** Inside the container:

```bash
pkill -f "gz sim"; pkill -f ros_gz
```

## Changing what is installed

Add the package to the `apt-get install` list in the `Dockerfile`, then:

```bash
./docker/run.sh        # rebuilds; the cached layers make it quick
```

Installing something inside a running container works for a quick try but is
lost when the container is recreated. Anything worth keeping belongs in the
`Dockerfile`.
