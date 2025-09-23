# Guide: Integrating ROS 2 with a Conda Environment

This guide provides a stable and reliable procedure for setting up and running this ROS 2 project within a Conda virtual environment. Using Conda is highly recommended for managing complex Python dependencies like `airsim`, `pytorch`, etc., but requires a specific workflow to avoid conflicts with the ROS 2 system environment.

### The Golden Rule

The core principle for all operations, including building and running, is:
**Activate the Conda environment first, then source the ROS 2 setup files.**

---

### Step 1: Environment Creation and Preparation

A correctly configured Conda environment is the foundation for success.

**1. Create the Conda Environment**

We recommend using Python 3.10 to match the ROS 2 Humble distribution.

```bash
# Create a new environment named 'airsim'
conda create -n airsim python=3.10
# it would be better to use a same python version with /usr/bin/env python3, for example in my ubuntu, it is python==3.10.12
```

**2. Activate and Install Core Tools**

Within the new environment, install the project dependencies and essential ROS 2 build tools.

```bash
# Activate the environment
conda activate airsim

# Install colcon, the ROS 2 build tool, from the conda-forge channel
conda install -c conda-forge colcon-common-extensions

# Install an updated C++ runtime to prevent GLIBCXX conflicts with ROS 2
conda install -c conda-forge libstdcxx-ng
```

**3. Install Project-Specific Python Dependencies**

Install all Python packages required by this project into the Conda environment.

```bash
# Make sure the 'airsim' environment is active
conda activate airsim

# Install dependencies using pip or conda
pip install msgpack-rpc-python
pip install airsim
conda install numpy scipy pandas matplotlib
```


---

### Step 2: Building the ROS 2 Workspace

This is the most critical phase. An incorrect build environment will result in the wrong Python interpreter being used at runtime.

**1. Start from a fresh, clean terminal session.**

**2. Clean up old build artifacts** (skip if this is your first time building):
```bash
cd /path/to/your/ros2_ws
rm -rf build/ install/ log/
```

**3. Activate the Conda environment:**
```bash
conda activate airsim
```

**4. (Optional but Highly Recommended) Forcefully specify the Python interpreter.**
This explicitly tells the ROS 2 build system which Python executable to use, guaranteeing correctness.
```bash
export AMENT_PYTHON_EXECUTABLE=$(which python)
```

**5. Source the system's ROS 2 setup file:**
```bash
source /opt/ros/humble/setup.bash
```
at this time, which python should be virtual env

**6. Build the workspace:**
```bash
# From your workspace root (/path/to/your/ros2_ws)
colcon build
```

After the build completes, you can verify its success by checking the shebang line of your node's executable script. It **must** point to your Conda environment's Python.
```bash
head -n 1 install/<your_pkg_name>/lib/<your_pkg_name>/<your_node_name>
# Expected output: #!/path/to/your/miniconda3/envs/airsim/bin/python
```

---

### Step 3: Running ROS 2 Nodes

**1. Start from a fresh, clean terminal session.**

**2. Activate the Conda environment:**
```bash
conda activate airsim
```

**3. Source your workspace's `install` setup file:**
```bash
cd /path/to/your/ros2_ws
source install/setup.bash
```

**4. Launch the application:**
```bash
ros2 launch <your_pkg_name> <your_launch_file>.launch.py
```

---

### Troubleshooting Common Issues

* **Problem: `ModuleNotFoundError: No module named 'some_package'`**
    * **Cause A:** The package is not installed in your Conda environment.
        * **Solution:** Run `conda activate <your_env>` followed by `conda install some_package` or `pip install some_package`.
    * **Cause B:** The workspace was built in the wrong environment, creating an executable that uses the system Python.
        * **Solution:** Carefully follow all instructions in **Step 2: Building the ROS 2 Workspace**, especially the cleanup (`rm -rf ...`) and activation steps.

* **Problem: `ImportError: ... version 'GLIBCXX_...' not found`**
    * **Cause:** A C++ standard library conflict between the Conda environment and the system's ROS 2 installation.
    * **Solution:** Ensure you have installed the updated runtime as described in Step 1: `conda install -c conda-forge libstdcxx-ng`.

* **Problem: The wrong version of a package (e.g., NumPy) is being used.**
    * **Cause:** A package installed in your user-site directory (`~/.local/lib/pythonX.X/site-packages`) is "shadowing" the version in your Conda environment.
    * **Solution (Recommended):** Uninstall the conflicting package from your user-site. Run `pip uninstall <package_name>` and verify that the path it's removing from is `~/.local/lib/...`.
    * **Solution (Workaround):** Tell Python to ignore the user-site directory by setting an environment variable before launching: `export PYTHONNOUSERSITE=1`.