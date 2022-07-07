FROM nvidia/cuda:11.4.0-devel-ubuntu20.04
ARG USER_ID
ARG USER_NAME

# Install deps
RUN apt-get update && \
  DEBIAN_FRONTEND=noninteractive apt-get install -y  \
    wget unzip git less curl unzip build-essential cmake cmake-curses-gui \
    libopenexr-dev \
    libopenimageio-dev \
    libopenexr-dev \
    zlib1g-dev \
    libassimp-dev \
    libceres-dev \
    libboost-all-dev \
    libeigen3-dev

RUN mkdir /deps

# Build and install IMath
# WORKDIR  /deps
# RUN git clone https://github.com/AcademySoftwareFoundation/Imath.git
# WORKDIR /deps/Imath
# RUN mkdir /deps/Imath/build
# WORKDIR /deps/Imath/build
# RUN cmake ../
# RUN make -j && make install

# Build lib for Alembic files
WORKDIR /deps
RUN git clone https://github.com/alembic/alembic
WORKDIR /deps/alembic
#RUN git checkout 1.8.1
RUN git checkout 1.7.16
# Patch the cmake config to also support cpack - https://stackoverflow.com/questions/3557037/appending-a-line-to-a-file-only-if-it-does-not-already-exist
RUN grep -qxF 'include(CPack)' CMakeLists.txt || echo 'include(CPack)' >> CMakeLists.txt
RUN mkdir /deps/alembic/build
WORKDIR /deps/alembic/build
#RUN cmake -DCMAKE_INSTALL_PREFIX=/deps/alembic/build/install -DALEMBIC_SHARED_LIBS=OFF ../
RUN cmake ../
RUN make -j && make install && make package

# # Build and install the Geogram lib
# WORKDIR /deps
# RUN git clone https://drbroun@bitbucket.org/drbroun/geogram.git
# WORKDIR /deps/geogram
# RUN git checkout beholder
# RUN mkdir /deps/geogram/build
# WORKDIR /deps/geogram/build
# RUN cmake -DCMAKE_BUILD_TYPE=Release -DGEOGRAM_LIB_ONLY=ON \
#      -DGEOGRAM_USE_SYSTEM_GLFW3=OFF -DGEOGRAM_WITH_EXPLORAGRAM=OFF \
#      -DGEOGRAM_WITH_FPG=OFF -DGEOGRAM_WITH_GRAPHICS=OFF \
#      -DGEOGRAM_WITH_HLBFGS=OFF -DGEOGRAM_WITH_LEGACY_NUMERICS=OFF \
#      -DGEOGRAM_WITH_LUA=OFF -DGEOGRAM_WITH_TETGEN=OFF \
#      -DGEOGRAM_WITH_TRIANGLE=OFF -DVORPALINE_PLATFORM=Linux64-gcc ../
# RUN make -j && make install

RUN apt-get update && apt-get install -y gdb

# Gui deps
RUN apt-get update && apt-get install -y xterm

WORKDIR /src

# Setup username and id so that it matches host machine (otherwise files will be written as root)
RUN useradd -u ${USER_ID} ${USER_NAME}
ENV USER=${USER_NAME}
ENV USER=${USER_NAME}
ENV HOME=/home/${USER_NAME}
RUN mkdir -p /home/${USER_NAME}
RUN chown ${USER_NAME}:${USER_NAME} /home/${USER_NAME}
USER ${USER_NAME}