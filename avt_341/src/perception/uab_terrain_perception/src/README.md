# Building Linux shared library
1. Build docker image `docker build -t matlab_with_perception_add_ons:r2023a .`
2. Create/run the container `docker run -it --rm --name uab_terrain_perception_build -p 5901:5901 -p 6080:6080 matlab_with_perception_add_ons:r2023a -vnc`
3. Then use either:
   1. Another terminal
      1. `docker exec -it uab_terrain_perception_build /bin/bash`
   2. or VNC Viewer
      1. Host: `localhost:1`
      2. Pw: `matlab`
4. Grant execute permission to build script with `sudo chmod +x build_uab_perception.sh`
5. Run build script `./build_uab_perception.sh`
6. Build contents are at `/home/matlab/lib_uab_perception_wrapper_out` by default
   - ex. Copy to host machine
      1. `docker cp uab_terrain_perception_build:/home/matlab/lib_uab_perception_wrapper_out/lib_uab_perception_wrapper.so .`
      2. `docker cp uab_terrain_perception_build:/home/matlab/lib_uab_perception_wrapper_out/lib_uab_perception_wrapper.h .`