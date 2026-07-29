# Object detection models

This empty folder is a placeholder for object detection models loaded manually
or downloaded through the CMake build.

## Manual installation

All `*.names` and `*.torchscript` files in this folder will be automatically
deployed to the installed packages and made available to nodes using object
detection models by loading through their filename, without specifying an
absolute or relative path to the model.

## Automatic download via CMake build

To automatically download all available models via CMake at build time, call
CMake with the additional argument `-DGET_AVT341_DETECTION_MODELS=ON`. If you
are using `colcon` to build this package, this can be dones by passing
`--cmake-args` to the `colcon build` command as follows:

```shell
colcon build --cmake-args -DGET_AVT341_DETECTION_MODELS=ON
```
