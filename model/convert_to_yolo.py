import pandas as pd
import os
import shutil
from pathlib import Path
import cv2
import random

# Configuration
DATASET_PATH = "archive"  # Your dataset path
OUTPUT_PATH = "lunar_dataset"
TRAIN_SPLIT = 0.8  # 80% train, 20% validation

# Choose which image type to use for training
# Options: 'render', 'clean', 'ground'
IMAGE_TYPE = 'render'  # Use 'render' for realistic renders

print(f"Dataset path: {DATASET_PATH}")
print(f"Using {IMAGE_TYPE} images for training")

# Load bounding boxes CSV
csv_path = os.path.join(DATASET_PATH, "bounding_boxes.csv")
print(f"\nLoading bounding boxes from: {csv_path}")
bbox_df = pd.read_csv(csv_path)

print(f"CSV loaded with {len(bbox_df)} bounding boxes")
print(f"Columns: {bbox_df.columns.tolist()}")

# CSV column mapping for your dataset
FRAME_COL = 'Frame'  # Image ID (1-9766)
X_COL = 'TopLeftCornerX'  # X coordinate
Y_COL = 'TopLeftCornerY'  # Y coordinate
W_COL = 'Length'  # Width
H_COL = 'Height'  # Height

# Load anomaly ID lists
anomaly_ids = set()
anomaly_files = {
    'mismatch_IDs.txt': 'Scene mismatch between mask and render',
    'cam_anomaly_IDs.txt': 'Camera artifacts',
    'shadow_IDs.txt': 'Overwhelming shadows',
    'ground_facing_IDs.txt': 'Camera pointed at ground',
    'top200_largerocks_IDs.txt': 'Large rock misalignment'
}

print("\n" + "=" * 60)
print("Loading anomaly ID lists...")
print("=" * 60)

for anomaly_file, description in anomaly_files.items():
    file_path = os.path.join(DATASET_PATH, anomaly_file)
    if os.path.exists(file_path):
        with open(file_path, 'r') as f:
            ids = [line.strip() for line in f if line.strip()]
            anomaly_ids.update(ids)
        print(f"  {anomaly_file}: {len(ids)} IDs ({description})")
    else:
        print(f"  {anomaly_file}: Not found (skipping)")

print(f"\nTotal anomalous IDs to exclude: {len(anomaly_ids)}")

# Create output directories
os.makedirs(os.path.join(OUTPUT_PATH, "images", "train"), exist_ok=True)
os.makedirs(os.path.join(OUTPUT_PATH, "images", "val"), exist_ok=True)
os.makedirs(os.path.join(OUTPUT_PATH, "labels", "train"), exist_ok=True)
os.makedirs(os.path.join(OUTPUT_PATH, "labels", "val"), exist_ok=True)


def format_id(frame_num):
    """Convert frame number to 4-digit ID string"""
    return f"{int(frame_num):04d}"


def get_image_path(frame_num, image_type='render'):
    """Get the full path to an image given its frame number"""
    img_id = format_id(frame_num)
    filename = f"{image_type}{img_id}.png"
    img_path = os.path.join(DATASET_PATH, "images", image_type, filename)
    return img_path if os.path.exists(img_path) else None


def convert_bbox_to_yolo(x, y, w, h, img_width, img_height):
    """
    Convert bounding box to YOLO format
    YOLO format: <class> <x_center> <y_center> <width> <height> (normalized 0-1)

    Input format: x, y (top-left corner), w, h (width, height)
    """
    # Calculate center point
    x_center = x + w / 2.0
    y_center = y + h / 2.0

    # Normalize by image dimensions
    x_center_norm = x_center / img_width
    y_center_norm = y_center / img_height
    width_norm = w / img_width
    height_norm = h / img_height

    # Clamp values to [0, 1] range
    x_center_norm = max(0, min(1, x_center_norm))
    y_center_norm = max(0, min(1, y_center_norm))
    width_norm = max(0, min(1, width_norm))
    height_norm = max(0, min(1, height_norm))

    return x_center_norm, y_center_norm, width_norm, height_norm


# Get all unique frame numbers (image IDs)
all_frames = bbox_df[FRAME_COL].unique()
print(f"\nFound {len(all_frames)} unique images with bounding boxes")

# Filter out anomalous samples
print("\n" + "=" * 60)
print("Filtering anomalous samples...")
print("=" * 60)

valid_frames = []
for frame_num in all_frames:
    frame_id = format_id(frame_num)

    if frame_id not in anomaly_ids:
        # Verify image exists
        img_path = get_image_path(frame_num, IMAGE_TYPE)
        if img_path:
            valid_frames.append(frame_num)

print(f"Valid images after filtering: {len(valid_frames)}")
print(f"Removed {len(all_frames) - len(valid_frames)} images (anomalies or missing files)")

# Split into train/val
random.seed(42)
random.shuffle(valid_frames)
split_idx = int(len(valid_frames) * TRAIN_SPLIT)
train_frames = valid_frames[:split_idx]
val_frames = valid_frames[split_idx:]

print(f"\nTrain set: {len(train_frames)} images")
print(f"Val set: {len(val_frames)} images")


def process_images(frame_list, split_name):
    """Process images and create YOLO label files"""
    processed = 0
    skipped = 0
    total_boxes = 0

    for frame_num in frame_list:
        # Get image path
        img_path = get_image_path(frame_num, IMAGE_TYPE)

        if not img_path:
            skipped += 1
            continue

        # Read image to get dimensions
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Could not read image {img_path}")
            skipped += 1
            continue

        img_height, img_width = img.shape[:2]

        # Copy image to output directory
        frame_id = format_id(frame_num)
        img_filename = f"{IMAGE_TYPE}{frame_id}.png"
        output_img_path = os.path.join(OUTPUT_PATH, "images", split_name, img_filename)
        shutil.copy(img_path, output_img_path)

        # Get all bounding boxes for this frame
        img_bboxes = bbox_df[bbox_df[FRAME_COL] == frame_num]

        # Create YOLO label file
        label_filename = f"{IMAGE_TYPE}{frame_id}.txt"
        label_path = os.path.join(OUTPUT_PATH, "labels", split_name, label_filename)

        num_boxes = 0
        with open(label_path, 'w') as f:
            for _, bbox in img_bboxes.iterrows():
                # Class ID: 0 for large rocks (single class detection)
                class_id = 0

                # Get bounding box coordinates
                x = float(bbox[X_COL])
                y = float(bbox[Y_COL])
                w = float(bbox[W_COL])
                h = float(bbox[H_COL])

                # Convert to YOLO format
                x_c, y_c, w_norm, h_norm = convert_bbox_to_yolo(x, y, w, h, img_width, img_height)

                # Skip invalid boxes
                if w_norm <= 0 or h_norm <= 0:
                    continue

                # Write to file
                f.write(f"{class_id} {x_c:.6f} {y_c:.6f} {w_norm:.6f} {h_norm:.6f}\n")
                num_boxes += 1

        total_boxes += num_boxes
        processed += 1

        if processed % 500 == 0:
            print(f"  Processed {processed}/{len(frame_list)} images...")

    avg_boxes = total_boxes / processed if processed > 0 else 0
    print(
        f"  {split_name}: Processed {processed} images, {total_boxes} boxes (avg {avg_boxes:.1f} boxes/image), skipped {skipped}")
    return processed, total_boxes


# Process train and validation sets
print("\n" + "=" * 60)
print("Processing training set...")
print("=" * 60)
train_count, train_boxes = process_images(train_frames, "train")

print("\n" + "=" * 60)
print("Processing validation set...")
print("=" * 60)
val_count, val_boxes = process_images(val_frames, "val")

# Create dataset.yaml file
yaml_content = f"""# Lunar Landscape Dataset
path: {os.path.abspath(OUTPUT_PATH)}
train: images/train
val: images/val

# Classes
nc: 1  # number of classes (large rocks only)
names: ['large_rock']
"""

yaml_path = os.path.join(OUTPUT_PATH, "dataset.yaml")
with open(yaml_path, 'w') as f:
    f.write(yaml_content)

# Print summary
print("\n" + "=" * 60)
print("DATASET CONVERSION COMPLETE")
print("=" * 60)
print(f"Output directory: {os.path.abspath(OUTPUT_PATH)}")
print(f"Configuration file: {yaml_path}")
print(f"\nDataset Summary:")
print(f"  Image type: {IMAGE_TYPE}")
print(f"  Training images: {train_count}")
print(f"  Training boxes: {train_boxes}")
print(f"  Validation images: {val_count}")
print(f"  Validation boxes: {val_boxes}")
print(f"  Total images: {train_count + val_count}")
print(f"  Total boxes: {train_boxes + val_boxes}")
print(f"  Avg boxes/image: {(train_boxes + val_boxes) / (train_count + val_count):.2f}")
print(f"\nFiltered out:")
print(f"  Anomalous samples: {len(anomaly_ids)}")
print(f"  Images without boxes: {9766 - len(all_frames)}")

print("\n" + "=" * 60)
print("Next Steps:")
print("=" * 60)
print("1. Verify a few images and labels:")
print(f"   Images: {OUTPUT_PATH}/images/train/")
print(f"   Labels: {OUTPUT_PATH}/labels/train/")
print("2. Run training:")
print("   python train_yolo.py")
print("=" * 60)