"""
Script to inspect your lunar dataset structure and CSV format
Run this BEFORE the conversion script to verify everything
"""

import pandas as pd
import os
from pathlib import Path

# Configuration
DATASET_PATH = "archive"  # Change this!

print("=" * 70)
print("LUNAR DATASET STRUCTURE INSPECTOR")
print("=" * 70)

# Check if dataset path exists
if not os.path.exists(DATASET_PATH):
    print(f"\nERROR: Dataset path not found: {DATASET_PATH}")
    print("Please update DATASET_PATH in the script!")
    exit(1)

print(f"\nDataset path found: {DATASET_PATH}")

# List all files in root directory
print("\n" + "=" * 70)
print("FILES IN DATASET ROOT:")
print("=" * 70)
root_files = os.listdir(DATASET_PATH)
for f in sorted(root_files):
    full_path = os.path.join(DATASET_PATH, f)
    if os.path.isfile(full_path):
        size = os.path.getsize(full_path)
        print(f" {f} ({size:,} bytes)")
    else:
        print(f" {f}/")

# Check for required files
print("\n" + "=" * 70)
print("CHECKING REQUIRED FILES:")
print("=" * 70)

required_files = ['bounding_boxes.csv']
optional_files = [
    'mismatch_IDs.txt',
    'cam_anomaly_IDs.txt',
    'shadow_IDs.txt',
    'ground_facing_IDs.txt',
    'top200_largerocks_IDs.txt'
]

for f in required_files:
    path = os.path.join(DATASET_PATH, f)
    if os.path.exists(path):
        print(f"  {f}")
    else:
        print(f"  {f} - MISSING!")

print("\nOptional anomaly ID files:")
for f in optional_files:
    path = os.path.join(DATASET_PATH, f)
    if os.path.exists(path):
        with open(path, 'r') as file:
            count = len([line for line in file if line.strip()])
        print(f"  {f} ({count} IDs)")
    else:
        print(f"  {f} - Not found")

# Check images directory structure
print("\n" + "=" * 70)
print("CHECKING IMAGE DIRECTORIES:")
print("=" * 70)

images_path = os.path.join(DATASET_PATH, "images")
if os.path.exists(images_path):
    print(f"images/ directory found")

    subdirs = ['render', 'clean', 'ground']
    for subdir in subdirs:
        subdir_path = os.path.join(images_path, subdir)
        if os.path.exists(subdir_path):
            count = len([f for f in os.listdir(subdir_path) if f.endswith('.png')])
            print(f"  images/{subdir}/ - {count} images")
        else:
            print(f"  images/{subdir}/ - Not found")
else:
    print(f"images/ directory not found!")

# Analyze bounding_boxes.csv
print("\n" + "=" * 70)
print("ANALYZING BOUNDING_BOXES.CSV:")
print("=" * 70)

csv_path = os.path.join(DATASET_PATH, "bounding_boxes.csv")
if os.path.exists(csv_path):
    df = pd.read_csv(csv_path)

    print(f"\nCSV loaded successfully!")
    print(f"  Total rows: {len(df)}")
    print(f"  Columns ({len(df.columns)}): {df.columns.tolist()}")

    print("\nColumn Data Types:")
    for col in df.columns:
        print(f"  {col}: {df[col].dtype}")

    print("\nFirst 5 rows:")
    print(df.head().to_string())

    print("\nBasic Statistics:")
    print(df.describe())

    # Try to identify coordinate columns
    print("\nDetecting coordinate columns:")
    possible_coords = ['x', 'y', 'w', 'h', 'xmin', 'ymin', 'width', 'height',
                       'x_min', 'y_min', 'left', 'top', 'right', 'bottom']
    found_coords = [col for col in df.columns if col.lower() in possible_coords]

    if found_coords:
        print(f"  Potential coordinate columns: {found_coords}")
        print("\n  Sample values from these columns:")
        print(df[found_coords].head().to_string())
    else:
        print("  Could not automatically identify coordinate columns")

    # Check for ID column
    print("\nDetecting image ID column:")
    possible_id_cols = ['id', 'image_id', 'filename', 'image', 'image_name']
    id_col = next((col for col in df.columns if col.lower() in possible_id_cols), None)

    if id_col:
        print(f"  Likely ID column: '{id_col}'")
        print(f"  Unique images: {df[id_col].nunique()}")
        print(f"  Sample IDs: {df[id_col].head(10).tolist()}")
    else:
        print("  Could not automatically identify ID column")
        print(f"  Available columns: {df.columns.tolist()}")

    # Check for missing values
    print("\nMissing values check:")
    missing = df.isnull().sum()
    if missing.any():
        print("  Found missing values:")
        for col, count in missing[missing > 0].items():
            print(f"    {col}: {count} missing")
    else:
        print("  No missing values found")

else:
    print("bounding_boxes.csv not found!")

# Check real_moon_images if it exists
print("\n" + "=" * 70)
print("CHECKING REAL MOON IMAGES:")
print("=" * 70)

real_moon_path = os.path.join(DATASET_PATH, "real_moon_images")
if os.path.exists(real_moon_path):
    count = len([f for f in os.listdir(real_moon_path) if f.endswith(('.png', '.jpg', '.jpeg'))])
    print(f"real_moon_images/ found - {count} images")
    print("  (These can be used for testing the trained model)")
else:
    print("real_moon_images/ not found")

# Summary and recommendations
print("\n" + "=" * 70)
print("SUMMARY & RECOMMENDATIONS:")
print("=" * 70)

print("\nNext steps:")
print("1. Update DATASET_PATH in convert_to_yolo.py")
print("2. Based on the CSV analysis above, verify the column names in convert_to_yolo.py")
print("3. Choose which image type to use (render, clean, or ground)")
print("4. Run: python convert_to_yolo.py")
print("5. Run: python train_yolo.py")

print("\nTips:")
print("- 'render' images are realistic and recommended for training")
print("- 'clean' images are cleaned-up ground truth")
print("- 'ground' images are segmentation masks")
print("- The anomaly ID files will automatically filter out problematic images")

print("\n" + "=" * 70)
