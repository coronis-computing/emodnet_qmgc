# Installation

You can build the tools from source, or run them from a Docker image that bundles everything they need.

## Option 1: Docker

The repository includes a `Dockerfile` that builds the tools on Ubuntu 24.04 and produces a small runtime image. The
image contains:

- `qm_tiler` and `dem2tin`;
- the helper scripts `create_layer_file.py` and `simple_tiles_server.py`;
- the GDAL command line tools (`gdalinfo`, `gdalwarp`, `gdalbuildvrt`, `gdaladdo`...), to prepare the inputs.

Build the image from the root of the repository:

```bash
docker build -t emodnet_qmgc .
```

The working directory in the container is `/data`. Mount your data folder there and run any of the tools:

```bash
docker run --rm -v "$PWD":/data emodnet_qmgc \
    qm_tiler -i my_dem.tif -o tiles -b
```

To serve the tiles from the container, publish the server port:

```bash
docker run --rm -p 8000:8000 -v "$PWD":/data emodnet_qmgc \
    simple_tiles_server.py -p 8000 tiles
```

!!! tip "Prebuilt images"
    A GitHub Actions workflow (`.github/workflows/docker.yml`) builds the image on every push. It publishes it to
    Docker Hub as `<user>/emodnet_qmgc:master` for the `master` branch, and as `:<tag>` and `:latest` for version tags
    (`v*`).

!!! note "File ownership"
    Files written in the container belong to `root` by default. Add `--user "$(id -u):$(id -g)"` to the
    `docker run` command to create them with your own user.

## Option 2: Build from source

### Requirements

The project is written in C++14, and builds with CMake (3.10 or later). It depends on:

| Library | Version | Notes |
|---------|---------|-------|
| [Boost](https://www.boost.org/) | | `program_options`, `filesystem` and `system` |
| [ZLib](https://www.zlib.net/) | | |
| [GDAL](https://gdal.org/) | 3.0 or later | |
| [GeographicLib](https://geographiclib.sourceforge.io/) | 1.x or 2.x | ECEF conversions |
| [CGAL](https://www.cgal.org/) | 5.x (tested with 5.6) | CGAL 4.x is no longer supported |
| [meshoptimizer](https://github.com/zeux/meshoptimizer) | bundled | A slightly modified copy is included in `3rdParty` |
| [JSON for Modern C++](https://github.com/nlohmann/json) | bundled | Included in `3rdParty` |

Python 3 is only needed for the optional [helper scripts](../user-guide/scripts.md).

### Ubuntu 24.04

All the dependencies are in the official repositories, which provide GDAL 3.8 and CGAL 5.6:

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    zlib1g-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libcgal-dev \
    libgdal-dev \
    libgeographiclib-dev \
    libproj-dev \
    gdal-bin \
    python3
```

Optionally, install `doxygen` to build the [API reference](../developer/api-reference.md) with `make doc`.

### Compilation

The build follows the usual CMake steps:

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
```

To install the tools system-wide (to `/usr/local` by default), run this from the `build` folder:

```bash
sudo make install
```

The helper scripts are not installed. Run them from the `scripts` folder, or copy them to a folder in your `PATH`.

### CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `CMAKE_BUILD_TYPE` | `Release` | Building without optimizations makes the CGAL code many times slower, so keep `Release` unless you are debugging. |
| `QMGC_BUILD_DEV_TESTS` | `ON` | Builds the [developer tools](../developer/dev-tools.md) in `src/dev-tests`. Set it to `OFF` to shorten the build. |
| `CMAKE_INSTALL_PREFIX` | `/usr/local` | Install location used by `make install`. |

For example:

```bash
cmake .. -DQMGC_BUILD_DEV_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$HOME/.local
```

## Check the installation

```bash
qm_tiler --help
dem2tin --help
```

Both commands print the list of their options. You are now ready for the [quick start](quickstart.md).
