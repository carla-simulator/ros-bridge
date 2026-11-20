apt-get install -y --no-install-recommends \
    libpng16-16 \
    libtiff6 \
    libjpeg8 \
    build-essential \
    wget \
    git \
    libxerces-c-dev \
    python3-pip 

export CARLA_ARTIFACTS_URL="https://github.com/cgeller/carla/releases/download/test/PythonAPI.tar.gz"
export CARLA_API_PATH="/opt/carla/PythonAPI"
export CARLA_CACHE_DIR="/tmp/carlaCache"
export CARLA_SETUP_SCRIPT="/opt/carla/setup.bash"

# Download PythonAPI as artifact from CARLA CI pipeline
mkdir -p /opt/carla
curl --location --output artifacts.tar.gz "https://github.com/openads-project/carla-simulator/releases/download/v1.0.0-1.0.0/PythonAPI.tar.gz"
tar -xzf "artifacts.tar.gz" -C .
mv PythonAPI "$CARLA_API_PATH"

# Install the CARLA wheel that matches the current Python minor version.
pyver=$(python3 -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')")
shopt -s nullglob
wheels=("$CARLA_API_PATH"/carla/dist/*"$pyver"*.whl)
shopt -u nullglob
if [[ ${#wheels[@]} -eq 0 ]]; then
    echo "No CARLA wheel found for Python $pyver in $CARLA_API_PATH/carla/dist" >&2
    exit 1
fi
python3 -m pip install --no-cache-dir "${wheels[0]}"

mkdir -p "$(dirname "$CARLA_SETUP_SCRIPT")" "$CARLA_CACHE_DIR"
chmod 1777 "$CARLA_CACHE_DIR"

# Create a script to append necessary paths to PYTHONPATH.
{
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla/agents"
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla"
    echo "export CARLA_CACHE_DIR=$CARLA_CACHE_DIR"
} >> "$CARLA_SETUP_SCRIPT"

# .bashrc sources the setup script
echo "source /opt/carla/setup.bash" >> /root/.bashrc
