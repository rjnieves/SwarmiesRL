FROM nvidia/cudagl:9.0-base-ubuntu16.04

# install packages
RUN apt-get update && apt-get install -q -y \
  build-essential \
  cuda-command-line-tools-9-0 \
  cuda-cublas-9-0 \
  cuda-cufft-9-0 \
  cuda-curand-9-0 \
  cuda-cusolver-9-0 \
  cuda-cusparse-9-0 \
  curl \
  libcudnn7=7.2.1.38-1+cuda9.0 \
  libnccl2=2.2.13-1+cuda9.0 \
  libfreetype6-dev \
  libhdf5-serial-dev \
  libpng12-dev \
  libzmq3-dev \
  pkg-config \
  python \
  python-dev \
  python-pip \
  rsync \
  software-properties-common \
  unzip \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
  apt-get install nvinfer-runtime-trt-repo-ubuntu1604-4.0.1-ga-cuda9.0 && \
  apt-get update && \
  apt-get install libnvinfer4=4.1.2-1+cuda9.0

RUN pip install --upgrade pip setuptools

RUN pip install tensorflow-gpu==1.12.3 keras
