# EfficientViT: Multi-Scale Linear Attention for High-Resolution Dense Prediction
# Han Cai, Junyan Li, Muyan Hu, Chuang Gan, Song Han
# International Conference on Computer Vision (ICCV), 2023

import argparse
import datetime
import os
import sys

import cv2
import numpy as np
from copy import deepcopy

import tkinter as tk
import threading

from efficientvit.apps.utils import parse_unknown_args
from efficientvit.models.efficientvit.sam import EfficientViTSamAutomaticMaskGenerator, EfficientViTSamPredictor
from efficientvit.models.utils import build_kwargs_from_config
from efficientvit.sam_model_zoo import create_sam_model

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path

# Define your classes
classes = ["MRZR"]
class_id = 0  # Global variable to hold the selected class ID
buttons = []  # List to store button references

clicked_point = None
got_click = False


def parse_arguments():
    parser = argparse.ArgumentParser(prog="label_data.py",
                                     description="Annotation tool for YoloV5/V8. "
                                                 "Can label from videos, images or rosbags",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("input_dir", type=str,
                        help="Input directory containing either .mp4 videos, .png/.jpg images or rosbags (ROS 1 or 2")
    parser.add_argument("-o", "--output_dir", type=str, default="annotations",
                        help="Output directory for pairs of images and annotations")
    parser.add_argument("-f", "--force_reannotate", type=bool, default=False, help="Overwrite existing annotations")
    # TODO: skip timestep
    parser.add_argument("-s", "--step", type=float, default=5.0,
                        help="Skip a time step (in seconds) between each frame to be labeled")
    parser.add_argument("-t", "--topic", type=str, default="/camera")
    parser.add_argument("-r", '--img_framerate', type=int, default=30,
                        help="Images sequences assumed recorded at this framerate")
    # SAM arguments
    parser.add_argument("--model", type=str, default="l1")
    parser.add_argument("--weight_url", type=str, default=None)
    parser.add_argument("--multimask", action="store_true")
    parser.add_argument("--image_path", type=str, default="assets/fig/cat.jpg")
    parser.add_argument("--mode", type=str, default="all", choices=["point", "box", "all"])
    parser.add_argument("--point", type=str, default=None)
    parser.add_argument("--box", type=str, default=None)
    # EfficientViTSamAutomaticMaskGenerator args
    parser.add_argument("--pred_iou_thresh", type=float, default=0.8)
    parser.add_argument("--stability_score_thresh", type=float, default=0.85)
    parser.add_argument("--min_mask_region_area", type=float, default=100)
    args, opt = parser.parse_known_args()
    opt = parse_unknown_args(opt)
    return args, opt


def create_gui():
    # Start the Tkinter GUI in a separate thread
    gui_thread = threading.Thread(target=run_tkinter_gui)
    gui_thread.daemon = True  # Set daemon so it exits when the main program does
    gui_thread.start()


def run_tkinter_gui():
    root = tk.Tk()
    root.title("Class Selector")

    # Set window size (width x height)
    root.geometry('500x500')  # Adjust as needed for your display

    # Font configuration for buttons
    button_font = ('Helvetica', 25)  # Change font and size as desired

    # Create a button for each class
    for idx, cls in enumerate(classes):
        btn = tk.Button(root, text=cls, font=button_font, command=lambda idx=idx: select_class(idx))
        btn.pack(pady=10, padx=10, fill=tk.X)
        buttons.append(btn)  # Store button reference

    root.mainloop()


def create_efficientvit_sam_model(args, opt):
    # build model
    efficientvit_sam = create_sam_model(args.model, True, args.weight_url).cuda().eval()
    efficientvit_sam_predictor = EfficientViTSamPredictor(efficientvit_sam)
    efficientvit_mask_generator = EfficientViTSamAutomaticMaskGenerator(
        efficientvit_sam,
        pred_iou_thresh=args.pred_iou_thresh,
        stability_score_thresh=args.stability_score_thresh,
        min_mask_region_area=args.min_mask_region_area,
        **build_kwargs_from_config(opt, EfficientViTSamAutomaticMaskGenerator),
    )
    return efficientvit_sam_predictor


def main():
    global got_click, clicked_point
    args, opt = parse_arguments()

    create_gui()
    efficientvit_sam_predictor = create_efficientvit_sam_model(args, opt)

    if os.path.isabs(args.output_dir):
        save_dir = args.output_dir
    else:
        save_dir = os.path.abspath(os.path.join(args.input_dir, "..", args.output_dir))

    cv2.namedWindow("Select object", cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Image Display', 1920, 1080)  # Set your desired window sizee
    for root, _, files in os.walk(args.input_dir):
        count = 0
        for file in files:
            file_path = os.path.join(root, file)
            if is_image(file):
                if not image_is_processed(file, save_dir) or args.force_reannotate:
                    process_image(count, efficientvit_sam_predictor, file, file_path, save_dir, args.step,
                                  args.img_framerate)
                else:
                    print(f"{file} already labeled")
                count += 1
            elif is_video(file):
                print(f"Labeling video {file}")
                process_video(efficientvit_sam_predictor, file, file_path, save_dir, args.force_reannotate, args.step)
                print(f"Labeled {file}")
            elif is_rosbag1(file) or is_rosbag2(file):
                print(f"Rosbag not fully implemented. Would have labeled {file} with {file_path}")
                process_bag(efficientvit_sam_predictor, file, file_path, save_dir, args.force_reannotate,
                            args.step, args.topic)
    print("Processed all valid files")

    cv2.destroyAllWindows()
    sys.exit()


def process_image(count, efficientvit_sam_predictor, file, file_path, save_directory, time_step, framerate):
    frames_per_timestep = framerate * time_step
    if count % frames_per_timestep == 0:
        image = cv2.imread(file_path)
        labeled_filename = f"{file.replace('.', '_')}"
        label_and_save_frame(labeled_filename, efficientvit_sam_predictor, image, save_directory)
    # else:
    #     print(f"Skipping frame {file} because timestep not reached")


def process_video(efficientvit_sam_predictor, file, file_path, save_directory, force_reannotate, time_step):
    video_capture = cv2.VideoCapture(file_path)
    if not video_capture.isOpened():
        print("Error opening video stream or file")

    count = 0
    while True:
        frame_time_ms = video_capture.get(cv2.CAP_PROP_POS_MSEC)
        ret, frame = video_capture.read()
        if not ret:
            break
        if frame_time_ms / 1000 >= count * time_step:
            frame_time = datetime.timedelta(milliseconds=frame_time_ms)
            h_m_s = str(frame_time).split('.')[0].replace(':', '_')
            file_basename = "{}_{}_{:03d}".format(file.replace('.', '_'), h_m_s, frame_time.microseconds // 1000)
            if not video_frame_is_processed(file_basename, save_directory) or force_reannotate:
                label_and_save_frame(file_basename, efficientvit_sam_predictor, frame, save_directory)
            else:
                print(file_basename, "already processed")
            count += 1
    video_capture.release()
    return count


def process_bag(efficientvit_sam_predictor, file, file_path, save_directory, force_reannotate, time_step, topic):
    if is_rosbag1(file):
        typestore = get_typestore(Stores.ROS1_NOETIC)
    else:
        typestore = get_typestore(Stores.ROS2_HUMBLE)

    count = 0
    # Create reader instance and open for reading.
    with AnyReader([Path(file_path)], default_typestore=typestore) as reader:
        connections = [conn for conn in reader.connections if conn.topic == topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            if count == 0:
                first_frame_time = timestamp * 1e-9
            frame_time = timestamp * 1e-9 - first_frame_time
            if frame_time >= count * time_step:
                msg = reader.deserialize(rawdata, connection.msgtype)
                if count == 0:
                    first_msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    first_time_seconds = msg.header.stamp.sec  # message time is most likely smaller than rosbag time
                msg_time_diff = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - first_msg_time
                seconds = int(msg_time_diff)
                hours, hours_remainder = divmod(seconds, 3600)
                minutes, _ = divmod(hours_remainder, 60)
                unix_ms = int((msg_time_diff * 1000) % 1000)
                file_basename = "{}_{}_{:02d}_{:02d}_{:03d}".format(file.replace('.', '_'), hours, minutes, seconds,
                                                                    unix_ms)

                if not video_frame_is_processed(file_basename, save_directory) or force_reannotate:
                    if msg.encoding == 'bayer_rggb8':
                        raw_image = np.reshape(msg.data, (msg.height, msg.width))
                        color_image = cv2.cvtColor(raw_image, cv2.COLOR_BAYER_RG2RGB)
                        label_and_save_frame(file_basename, efficientvit_sam_predictor, color_image, save_directory)
                    else:
                        print("Processing of RBG images not implemented yet.")
                else:
                    print(file_basename, "already processed")
                count += 1
    print("Processed images in bag")


def label_and_save_frame(filename, efficientvit_sam_predictor, frame, save_directory):
    global got_click
    # todo handle colors gnerically (rggb etc)
    inverted_im = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb_image = np.array(inverted_im)
    height, width, _ = frame.shape
    cv2.setMouseCallback("Select object", get_mouse_click)
    img = deepcopy(frame)
    bboxes = []
    print(f"Using class {classes[class_id]}")
    frame_not_processed = True
    while frame_not_processed:
        save = False
        skip_img = False
        got_click = False
        manually_draw = False
        while not got_click:
            cv2.imshow("Select object", img)
            key = cv2.waitKey(1)
            if key == ord('q'):
                got_click = True
                skip_img = True
            elif key == ord('g'):
                got_click = True
                manually_draw = True

        if skip_img:
            print("Skipping", filename)
            # count += 1
            break
        elif manually_draw:
            print("Manually draw ...")
            # count += 1
            bbox = cv2.selectROI('Select object', frame, showCrosshair=True, fromCenter=False)
            bboxes = [bbox]
            break

        point_coords = [(x, y) for x, y in clicked_point]
        point_labels = [l for _, l in clicked_point]

        efficientvit_sam_predictor.set_image(rgb_image)
        masks, scores, _ = efficientvit_sam_predictor.predict(
            point_coords=np.array(point_coords),
            point_labels=np.array(point_labels),
            multimask_output=True,
        )
        idx = np.argmax(scores)
        img = draw_binary_mask(img, masks[idx], (0, 0, 255))
        img = draw_scatter(img, point_coords)
        bbox = get_bbox_from_mask(masks[idx])
        bboxes.append(bbox)
        cv2.rectangle(img, bbox, (255, 0, 0), 2)

        cv2.imshow("Select object", img)
        key = cv2.waitKey(0)
        if key == ord('x'):
            continue
        elif key == ord('r'):
            img = deepcopy(frame)
            bboxes = []
        elif key == ord(" "):  #
            # count += 1
            save = True
            break
        elif key == ord('g'):  # manually select
            bbox = cv2.selectROI('Select object', frame, showCrosshair=True, fromCenter=False)
            bboxes = [bbox]
            save = True
            break
        elif key == ord('q'):  # skip
            # count += 1
            break
    if save:
        save_annotations(save_directory, filename, class_id, frame, bboxes)
        return True
    return False


def save_annotations(save_directory, filename, class_id, frame, bboxes):
    os.makedirs(save_directory, exist_ok=True)
    base_name = f"{filename}_label_{class_id}"
    full_base_name = os.path.join(save_directory, base_name)
    label_path = f"{full_base_name}.txt"
    h, w, _ = frame.shape
    with open(label_path, 'w') as file:
        for bbox in bboxes:
            x_center = (bbox[0] + bbox[2] / 2) / w
            y_center = (bbox[1] + bbox[3] / 2) / h
            width = bbox[2] / w
            height = bbox[3] / h
            file.write(f"{class_id} {x_center} {y_center} {width} {height}\n")

    image_path = f"{full_base_name}.jpg"
    cv2.imwrite(image_path, frame)
    print("Saved label", base_name)


def is_video(file):
    return file.endswith('.mp4') or file.endswith('.MP4')


def is_rosbag1(file):
    return file.endswith('.bag')


def is_rosbag2(file):
    return file.endswith('.db3') or file.endswith('.mcap')


def is_image(file):
    return file.endswith('.png') or file.endswith('.jpg')


def image_is_processed(file, save_dir):
    labeled_file = f"{file.replace('.', '_')}_label_{class_id}.txt"
    return os.path.exists(os.path.join(save_dir, labeled_file))


def video_frame_is_processed(file, save_dir):
    labeled_file = f"{file}_label_{class_id}.txt"
    return os.path.exists(os.path.join(save_dir, labeled_file))


def select_class(index):
    global class_id, buttons
    class_id = index
    print(f"Selected class: {classes[class_id]}")
    # Update button styles to reflect the current selection
    for idx, btn in enumerate(buttons):
        if idx == class_id:
            btn.config(bg='blue', fg='white')  # Highlight selected button
        else:
            btn.config(bg='white', fg='black')  # Reset other buttons


def get_bbox_from_mask(mask: np.ndarray):
    # Ensure the mask is a 2D array, since cv2.findContours expects a single-channel image
    if len(mask.shape) > 2:
        mask = mask[:, :, 0]  # Assuming mask is grayscale and extra dimensions are redundant

    # Convert mask to uint8 type
    mask = (mask * 255).astype(np.uint8)
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Handle the case where no contours are found
    if not contours:
        return None  # Or return (0,0,0,0) if you need a default bbox format

    # Sort contours by area and get the largest one
    largest_contour = max(contours, key=cv2.contourArea)

    # Compute the bounding box of the largest contour
    bbox = cv2.boundingRect(largest_contour)
    return bbox


def draw_binary_mask(raw_image: np.ndarray, binary_mask: np.ndarray, mask_color=(0, 0, 255)) -> np.ndarray:
    # Create a mask where the binary mask is True
    color_mask = np.zeros_like(raw_image)
    color_mask[binary_mask == 1] = mask_color

    # Blend the color mask with the raw image
    canvas = cv2.addWeighted(raw_image, 0.9, color_mask, 0.5, 0)
    return canvas


def draw_scatter(image: np.ndarray, points: list[list[int]], color=(0, 255, 0), radius=5, thickness=-1) -> np.ndarray:
    # Check if a single color is provided or a list of colors for each point
    if isinstance(color, tuple):
        for x, y in points:
            cv2.circle(image, (x, y), radius, color, thickness)
    else:
        for point, clr in zip(points, color):
            x, y = point
            cv2.circle(image, (x, y), radius, clr, thickness)

    return image


# Mouse callback function
def get_mouse_click(event, x, y, flags, param):
    global clicked_point, got_click
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = np.array([[x, y]])
        got_click = True


if __name__ == "__main__":
    main()
