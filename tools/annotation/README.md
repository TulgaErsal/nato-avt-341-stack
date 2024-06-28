# label_tool

Tool for annotating videos, rosbags or image sequences for YoloV5 or YoloV8 training. 
It uses [EfficientViT](https://github.com/mit-han-lab/efficientvit) for placing bounding boxes based on mouse clicks.

## Installation
First, install [conda](https://conda.io/projects/conda/en/latest/user-guide/getting-started.html) for creating virtual python environments.

Then, clone [EfficientViT](https://github.com/mit-han-lab/efficientvit) and follow its "Getting started" instructions:
```bash
conda create -n efficientvit python=3.10
conda activate efficientvit
conda install -c conda-forge mpi4py openmpi
pip install -r requirements.txt
```

In addition, install `rosbags`:
```bash
pip install rosbags
```

If you encounter problems with NumPy versions, try to downgrade to NumPy 1.X
```bash
pip install --force-reinstall -v "numpy<2.0.0"
```

Download one (or more) of the pretrained [Segment Anything Models (SAM)](https://github.com/mit-han-lab/efficientvit).
Larger models can be more accurate in the predictions, but will be slower to run. 

Create a new directory under `efficientvit/assets/`:
```shell
mkdir -p efficientvit/assets/checkpoints/sam
```
Place the model(s) here.

Then, download **label_data.py** and place it in `efficientvit/`

## Running
Activate the newly created conda environment and run the script.
`-h` will list all the options. 
```shell
conda activate efficientvit
python label_data.py -h
```

```
usage: label_data.py [-h] [-o OUTPUT_DIR] [-f FORCE_REANNOTATE] [-s STEP]
                     [-t TOPIC] [-r IMG_FRAMERATE] [--model MODEL]
                     [--weight_url WEIGHT_URL] [--multimask]
                     [--image_path IMAGE_PATH] [--mode {point,box,all}]
                     [--point POINT] [--box BOX]
                     [--pred_iou_thresh PRED_IOU_THRESH]
                     [--stability_score_thresh STABILITY_SCORE_THRESH]
                     [--min_mask_region_area MIN_MASK_REGION_AREA]
                     input_dir

Annotation tool for YoloV5/V8. Can label from videos, images or rosbags

positional arguments:
  input_dir             Input directory containing either .mp4 videos,
                        .png/.jpg images or rosbags (ROS 1 or 2)

options:
  -h, --help            show this help message and exit
  -o OUTPUT_DIR, --output_dir OUTPUT_DIR
                        Output directory for pairs of images and annotations
                        (default: annotations)
  -f FORCE_REANNOTATE, --force_reannotate FORCE_REANNOTATE
                        Overwrite existing annotations (default: False)
  -s STEP, --step STEP  Skip a time step (in seconds) between each frame to be
                        labeled (default: 5.0)
  -t TOPIC, --topic TOPIC
  -r IMG_FRAMERATE, --img_framerate IMG_FRAMERATE
                        Images sequences assumed recorded at this framerate
                        (default: 30)
  --model MODEL
  --weight_url WEIGHT_URL
  --multimask
  --image_path IMAGE_PATH
  --mode {point,box,all}
  --point POINT
  --box BOX
  --pred_iou_thresh PRED_IOU_THRESH
  --stability_score_thresh STABILITY_SCORE_THRESH
  --min_mask_region_area MIN_MASK_REGION_AREA
```

All the options from MODEL and downwards can be ignored, the default values are fine.

## Annotating
When the tool starts to run, it spawns a window for selecting object class.
Currently only 'MRZR' is available. More classes can be added by modifying the source code directly.

If the tool is able to find valid files from the given inputs, it will try to extract frames for labeling.

- **SPACE:** accept label
- **Q:** skip frame
- **R:** reset SAM generated bounding box
- **G:** draw bounding box
- **C:** skip frame (manual draw)

Works with .mp4 video files, .png/.jpg image sequences and ros1bags.
In theory, it should work with ros2bags as well, but there is some error when reading those types of bags. Will try to fix it soon.
