# avt_341_segmentation_node

## Install Dependencies
The python dependendencies are managed with pipenv. To install the dependencies run the following command inside the 'nato-avt-341-stack/avt_341/src/ia/segmentation' path:
```bash
pipenv install
```
Once the dependencies are installed, you can set the 'shebang' of the **avt_341_depth_estimation_node** with the following command:
```bash
python3 shebang_util.py
```

## Run the **avt_341_segmentation_node**
You can run the node using the provided launch file:
```bash
roslaunch avt_341 ai_segmentation_node_krc.launch
```

## Parameters
In the roslaunch file you can set the following parameters:
- **model_selection**: Is the model architecture selection. Default value: **$(find avt_341)/src/ia/segmentation/configs/bisenet_customer.py**
- **weight_path**: Is the path to the model pre-trained weights. Default value: **$(find avt_341)/src/ia/segmentation/Models/BiSeNet_Tracer/model_final_krc.pth**
- **input_image_topic**: The topic where the RGB images are published. Default value: **/airsim_node/tracer/front_center/Scene**
- **depth_map_topic**: The topic where the depth map is published. Default value: **/segmented_image_topic**