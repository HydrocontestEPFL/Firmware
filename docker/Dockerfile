FROM ubuntu:16.04
MAINTAINER Michael Spieler (nuft) <michael.spieler@epfl.ch>
LABEL Description="Image for building the modified Pixracer firmware for Hydrocontest"
WORKDIR /work

ADD . /work

# Install dependencies
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        git \
        wget \
        autoconf \
        automake \
        bison \
        build-essential \
        bzip2 \
        zip \
        file \
        flex \
        genromfs \
        gperf \
        libncurses-dev \
        libtool \
        pkg-config \
        cmake \
        python \
        python-empy \
        lib32ncurses5 lib32tinfo5 lib32z1 libc6-i386 \
    && apt clean

# ARM embedded gcc
RUN mkdir -p /opt/arm-none-eabi-gcc && \
    wget -qO- "https://launchpad.net/gcc-arm-embedded/4.9/4.9-2014-q4-major/+download/gcc-arm-none-eabi-4_9-2014q4-20141203-linux.tar.bz2" | tar jx --strip 1 -C /opt/arm-none-eabi-gcc && \
    rm -rf /opt/arm-none-eabi-gcc/share/doc

ENV PATH="/opt/arm-none-eabi-gcc/bin:$PATH"
