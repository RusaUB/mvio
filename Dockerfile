FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and install system dependencies
# We use apt-get for consistency and include packages for:
# - Build tools (build-essential, cmake)
# - Download/extraction (wget, unzip, curl, git)
# - Python environment (python3, python3-venv, python3-pip) for the dataset downloader
# - Library dependencies (OpenCV, Eigen3)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    wget \
    unzip \
    curl \
    git \
    python3 \
    python3-venv \
    python3-pip \
    python3-dev \
    libopencv-dev \
    libeigen3-dev \
    file \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY . .

RUN mkdir -p build

WORKDIR /app/build
RUN cmake .. && make -j$(nproc)

CMD ["/bin/bash"]
