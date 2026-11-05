# -----------------------------------------------------------------------------
# Build stage: compilers and development packages
# -----------------------------------------------------------------------------
FROM ubuntu:24.04 AS builder

# Note: GDAL >= 3.0 and CGAL >= 5.0 are required, both provided by Ubuntu 24.04
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    cmake \
    build-essential \
    pkg-config \
    zlib1g-dev \
    libboost-filesystem-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libcgal-dev \
    libgdal-dev \
    libgeographiclib-dev \
    libproj-dev \
    libnetcdf-dev \
    && rm -rf /var/lib/apt/lists/*

COPY . /usr/local/src/emodnet_qmgc
WORKDIR /usr/local/src/emodnet_qmgc/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DQMGC_BUILD_DEV_TESTS=OFF \
    && make -j$(nproc) \
    && make install DESTDIR=/staging

# -----------------------------------------------------------------------------
# Runtime stage: only the shared libraries the tools link against
# -----------------------------------------------------------------------------
FROM ubuntu:24.04

LABEL org.opencontainers.image.title="emodnet_qmgc" \
      org.opencontainers.image.description="EMODnet Quantized Mesh Generator for Cesium" \
      org.opencontainers.image.source="https://github.com/coronis-computing/emodnet_qmgc"

# gdal-bin is kept for preparing the inputs (gdaladdo, gdalbuildvrt...), and python3 for the scripts
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    libgdal34t64 \
    libboost-filesystem1.83.0 \
    libboost-program-options1.83.0 \
    libgeographiclib26 \
    libgmp10 \
    libmpfr6 \
    zlib1g \
    gdal-bin \
    python3 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /staging/usr/local/ /usr/local/
COPY scripts/*.py /usr/local/bin/
RUN chmod +x /usr/local/bin/*.py && ldconfig

WORKDIR /data
