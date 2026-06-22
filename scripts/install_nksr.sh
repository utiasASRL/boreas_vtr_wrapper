#!/bin/bash
set -e

if [ -z "${ROOTDIR:-}" ]; then
  SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
  ROOTDIR=$(dirname "$SCRIPT_DIR")
fi

if [ ! -d "$ROOTDIR/external/NKSR/package" ]; then
  echo "NKSR submodule is missing. Run:"
  echo "  git submodule update --init --recursive external/NKSR"
  exit 1
fi

if [ ! -f "$ROOTDIR/venv/bin/activate" ]; then
  echo "Project virtual environment not found at $ROOTDIR/venv."
  echo "Run scripts/create_venv.sh first."
  exit 1
fi

source "$ROOTDIR/venv/bin/activate"

# Keep the Python CUDA stack aligned with the CUDA 11.8 toolkit provided by
# the Docker image. Do not use external/NKSR/requirements.txt: upstream
# currently pins CUDA 12.8 packages.
pip install \
  torch==2.7.1+cu118 \
  torchvision==0.22.1+cu118 \
  torchaudio==2.7.1+cu118 \
  --index-url https://download.pytorch.org/whl/cu118

pip install \
  torch-scatter==2.1.2 \
  -f https://data.pyg.org/whl/torch-2.7.0+cu118.html

pip install \
  "python-pycg>=1.0.1" \
  pykdtree \
  omegaconf \
  ninja \
  GitPython \
  pyntcloud \
  plyfile

# The RTX A4500 Laptop GPU has compute capability 8.6. NKSR builds a custom
# C++/CUDA extension and therefore needs nvcc from the CUDA devel image.
export CUDA_HOME=/usr/local/cuda-11.8
export TORCH_CUDA_ARCH_LIST="8.6"
# NKSR compiles many large C++/CUDA translation units. Default to one compiler
# process to avoid exhausting system memory. Override these variables manually
# when more parallelism is safe.
export MAX_JOBS="${MAX_JOBS:-1}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
export MAKEFLAGS="${MAKEFLAGS:--j1}"

if [ ! -x "$CUDA_HOME/bin/nvcc" ]; then
  echo "nvcc was not found at $CUDA_HOME/bin/nvcc."
  echo "Run this script inside the project's CUDA devel container."
  exit 1
fi

pip install --no-build-isolation "$ROOTDIR/external/NKSR/package"

python -c "import nksr, torch; print(f'NKSR {nksr.__version__}; PyTorch {torch.__version__}; CUDA build {torch.version.cuda}')"
