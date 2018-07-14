# Docker image for easier firmware build

Advantage: easy installation, defined toolchain version for reproducible build
Drawback: slower compilation

## Dockerfile

You can create the docker image from the Dockerfile in this directory:
```bash
cd Firmware/docker
docker build -t px4-dev-hydrocontest .
```

## Compile the firmware using Docker

```bash
cd Firmware
docker run --rm -v (pwd):/work/Firmware px4-dev-hydrocontest bash -c 'cd Firmware && make clean && make px4fmu-v4_default'
```
