# UoS-ros-setup 🤖
ROS setup for University of Surrey's CVRML programme.
![ros_setup](./assets/turtlesim.png)

## Docker Setup

### Windows 🪟
Make sure you have the GlobalProtect VPN client installed and connected to the University of Surrey's network. See [here](uos_vpn_setup.pdf) for more information.

If you want the vanilla University of Surrey's docker image, run 
```bash
make windows-setup
```
This pulls the image from the University of Surrey's container registry and retag it as `uos-ros-setup:latest`. Alternatively, if you don't want to setup the VPN, you can pull from OSRF's official image:
```bash
docker pull osrf/ros:melodic-desktop-full
```

**Be mindful that both of these images are amd64 images.** 
If you're using M1 Macs, Docker uses QEMU to emulate the amd64 architecture on your machine. This can cause some unintended side effects so I highly advise the custom built image provided in this repository.

### Apple Sillicon 🍎
The `Dockerfile` in this repository uses `arm64v8/ros:melodic-ros-base-bionic` as the base image. You can build and tag this image by running:
```bash
make mac-setup
```

## X11 Forwarding
There's a pretty straightforward guide [here](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088).
Before you run, make sure you have XQuartz active and running. I explicitly set the `~/.zshrc` to export the `DISPLAY` variable to `:0`.
```bash
export DISPLAY=":0"
```

## Getting RViz to Work
https://gist.github.com/vfdev-5/b7685371071036cb739f23b3794b5b83
