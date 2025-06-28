
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
        valgrind

# install X-11 apps
RUN apt-get update \
    && apt-get install -y x11-apps

# install git
RUN apt-get update \
    && apt-get install -y git

# Reset DEBIAN_FRONTEND variable to its default value
ENV DEBIAN_FRONTEND=

# set the display environment variable
ENV DISPLAY=127.0.0.1:0
