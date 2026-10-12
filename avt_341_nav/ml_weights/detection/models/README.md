# Object detection models

This folder holds the object detection models, either loaded manually
or downloaded through the CMake build.

## Manual installation

All `*.names` and `*.torchscript` files in this folder will be automatically
deployed to the installed packages and made available to nodes using object
detection models by loading through their filename, without specifying an
absolute or relative path to the model.

## Automatic download via CMake build

All available models are downloaded automatically at build time into this
folder. A model file is only downloaded when it is missing or its hash does not
match the expected one, and a failed download (ex: no internet) only produces a
warning, so the build still succeeds and the next build retries.

To skip the download, call CMake with the additional argument
`-DGET_AVT341_DETECTION_MODELS=OFF`. If you are using `colcon` to build this
package, this can be done by passing `--cmake-args` to the `colcon build`
command as follows:

```shell
colcon build --cmake-args -DGET_AVT341_DETECTION_MODELS=OFF
```
