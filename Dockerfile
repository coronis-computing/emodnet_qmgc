FROM ubuntu:24.04

# Install dependencies available through apt-get
# Note: GDAL >= 3.0 and CGAL >= 5.0 are required, both provided by Ubuntu 24.04
RUN apt-get update -y && apt-get install -y \
    git \
    cmake \
    wget \
    build-essential \
    pkg-config \
    bash-completion \
    zlib1g-dev \
    libboost-all-dev \
    libcgal-dev \
    libgdal-dev \
    gdal-bin \
    libproj-dev \
    python3 \
    libnetcdf-dev

# Copy the required files inside the image
ADD . /usr/local/src/emodnet_qmgc
RUN mkdir /usr/local/src/emodnet_qmgc/build
WORKDIR /usr/local/src/emodnet_qmgc/build
RUN cmake ..
RUN make -j$(nproc)
RUN make install
RUN ldconfig

# Move also the python scripts to be part of the command line tools
RUN cp /usr/local/src/emodnet_qmgc/scripts/*.py /usr/local/bin
