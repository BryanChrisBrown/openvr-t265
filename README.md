# OpenVR T265

This is a community driver to support the Intel T265 sensor on OpenVR for 6DOF tracking.  It is unafiliated with Intel.

## Build Instructions

# Dependencies
- Visual Studio 2019 CE (https://visualstudio.microsoft.com/vs/). Under workloads make sure to select:
  - Desktop development with C++
  - .Net desktop development
- cmake (https://cmake.org/download/)
  - making sure to add it at least to the current user's system path
- Git (https://git-scm.com/download/win/)

# Installation
- Launch the windows command prompt `cmd`
- Run the following
```
git clone --recurse-submodules https://github.com/BryanChrisBrown/openvr-t265
cd openvr-t265
build.bat
```

