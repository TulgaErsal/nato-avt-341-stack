# avt_341_depth_estimation_node

## Install Dependencies
The python dependendencies are managed with pipenv. To install the dependencies run the following command inside the 'nato-avt-341-stack/avt_341/src/ia/depth_estimation' path:
```bash
pipenv install
```
Once the dependencies are installed, you can set the 'shebang' of the **avt_341_depth_estimation_node** with the following command:
```bash
python3 shebang_util.py
```

## Run the **avt_341_depth_estimation_node**
You can run the node using the provided launch file:
```bash
roslaunch avt_341 ai_depth_estimation_node_krc.launch
```

## Parameters
In the roslaunch file you can set the following parameters:
- **depth_estimation_model**: Is the path to the *.h5 file of the model. Default value: **$(find avt_341)/src/ia/depth_estimation/Models/densedepth_krc_v2_29082023_225814/model.h5**
- **input_image_topic**: The topic where the RGB images are published. Default value: **/airsim_node/tracer/front_center/Scene**
- **depth_map_topic**: The topic where the depth map is published. Default value: **/depth_map**

