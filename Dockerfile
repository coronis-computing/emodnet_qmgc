FROM ubuntu:18.04

# Install dependencies available through apt-get
RUN apt-get update -y && apt-get install -y \
    git \
    cmake \
    wget \
    python-dev \
    build-essential \
    pkg-config \
    bash-completion \
    zlib1g-dev \
    libboost-all-dev \
    libcgal-dev \
    libproj-dev \
    python3

# Download and install the gdal version that works with Cesium Terrain Builder (avoids the "undefined reference to `GDALCreateOverviewDataset(GDALDataset*" error)
WORKDIR /opt/ 
RUN wget http://download.osgeo.org/gdal/2.1.0/gdal-2.1.0.tar.gz
RUN tar -xvzf gdal-2.1.0.tar.gz
WORKDIR gdal-2.1.0
RUN ./configure --with-python
RUN make -j$(nproc)
RUN make install

# Clone and install Cesium Terrain Builder
# Note: need to patch the src/GDALTiler.cpp file (cannot compile otherwise because of the same "undefined reference to `GDALCreateOverviewDataset(GDALDataset*" error mentioned above)
WORKDIR /opt/ 
RUN git clone https://github.com/geo-data/cesium-terrain-builder.git
ADD ./docker/cesium-terrain-builder/patches/GDALTiler.cpp.patch /tmp/GDALTiler.cpp.patch
RUN patch /opt/cesium-terrain-builder/src/GDALTiler.cpp /tmp/GDALTiler.cpp.patch
RUN mkdir /opt/cesium-terrain-builder/build
WORKDIR /opt/cesium-terrain-builder/build
RUN cmake ..
RUN make -j$(nproc)
RUN make install
RUN ldconfig

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
