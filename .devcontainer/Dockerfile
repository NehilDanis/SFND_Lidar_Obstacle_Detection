
# add the start img 
FROM ubuntu:22.04

# Set environment variable to suppress interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# install cmake, gcc-12 and vim 
# with ubuntu the default coming gcc version is 9.5
# to use gcc 12 when calling gcc from command line
# the alternatives are updated
RUN set -ex                                                            &&\
    apt-get update                                                     &&\
    apt-get install -y cmake gcc-12 g++-12 vim                         &&\
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100 &&\
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100

# Install pcl
RUN apt-get update \
    && apt-get install -y \
        libpcl-dev \
        cmake \
        build-essential \
        valgrind \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Reset DEBIAN_FRONTEND variable to its default value
ENV DEBIAN_FRONTEND=
