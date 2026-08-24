FROM utiasasrl/vtr3:latest

ARG GROUPID=0
ARG USERID=0
ARG USERNAME=root
ARG HOMEDIR=/root

RUN if [ ${GROUPID} -ne 0 ]; then addgroup --gid ${GROUPID} ${USERNAME}; fi \
  && if [ ${USERID} -ne 0 ]; then adduser --disabled-password --gecos '' --uid ${USERID} --gid ${GROUPID} ${USERNAME}; fi

ENV DEBIAN_FRONTEND=noninteractive

## Switch to root to install dependencies
USER 0:0

# Install aws dependencies for boreas dataset installation
RUN apt update && apt upgrade -q -y zip unzip
RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip" && \
  unzip awscliv2.zip && \
  ./aws/install

# LaTeX support for plotting
RUN apt update && apt install -q -y texlive-latex-extra

## Install Torch. The base image already has a standalone LibTorch C++ build at
## /opt/torch/libtorch (cxx11-ABI) on LD_LIBRARY_PATH/CMAKE_PREFIX_PATH. pip's torch
## wheel bundles its own (pre-cxx11-ABI) libtorch, and having both resolve in the same
## process causes undefined-symbol crashes. So make pip's copy the ONLY one used, by
## pointing TORCH_LIB/LD_LIBRARY_PATH/CMAKE_PREFIX_PATH at it instead, ahead of the
## base image's paths -- this also makes any C++ code (e.g. vtr_torch) built later
## pick up pip's torch via find_package(Torch), so there is only ever one libtorch.
RUN pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
RUN pip install --upgrade packaging
ENV TORCH_LIB=/usr/local/lib/python3.10/dist-packages/torch
ENV LD_LIBRARY_PATH=$TORCH_LIB/lib:${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV CMAKE_PREFIX_PATH=$TORCH_LIB:$CMAKE_PREFIX_PATH

# Set up entrypoint
COPY ./entrypoint.sh ./entrypoint.sh
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

USER ${USERID}:${GROUPID}

# Set up env variables
ENV ROOTDIR=$(pwd)
