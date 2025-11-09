from ultralytics import YOLO
import torch

# Check if CUDA is available
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {device}")

# Initialize YOLO model
# Options: yolov8n.pt (nano), yolov8s.pt (small), yolov8m.pt (medium),
#          yolov8l.pt (large), yolov8x.pt (extra large)
# For lunar landscape with ~7800 training images, yolov8s is recommended

model = YOLO('yolov8s.pt')  # Load pretrained model (small - good balance)

# Train the model
results = model.train(
    data='lunar_dataset/dataset.yaml',  # Path to dataset config
    epochs=100,  # Number of epochs
    imgsz=640,  # Image size (640 is standard)
    batch=16,  # Batch size (adjust based on GPU memory)
    name='lunar_rocks_detection',  # Experiment name
    patience=50,  # Early stopping patience
    save=True,  # Save checkpoints
    device=device,  # Use GPU if available

    # Optional hyperparameters for better performance
    optimizer='AdamW',  # Optimizer (SGD, Adam, AdamW)
    lr0=0.01,  # Initial learning rate
    lrf=0.01,  # Final learning rate factor
    momentum=0.937,  # SGD momentum
    weight_decay=0.0005,  # Weight decay
    warmup_epochs=3.0,  # Warmup epochs

    # Augmentation parameters (can help with generalization)
    hsv_h=0.015,  # HSV hue augmentation
    hsv_s=0.7,  # HSV saturation augmentation
    hsv_v=0.4,  # HSV value augmentation
    degrees=0.0,  # Rotation (+/- degrees)
    translate=0.1,  # Translation (+/- fraction)
    scale=0.5,  # Scale (+/- gain)
    shear=0.0,  # Shear (+/- degrees)
    perspective=0.0,  # Perspective (+/- fraction)
    flipud=0.0,  # Flip up-down probability
    fliplr=0.5,  # Flip left-right probability
    mosaic=1.0,  # Mosaic augmentation probability
    mixup=0.0,  # Mixup augmentation probability

    # Validation settings
    val=True,  # Validate during training
    plots=True,  # Save training plots
    save_period=10,  # Save checkpoint every N epochs
)

print("\n" + "=" * 50)
print("Training Complete!")
print("=" * 50)
print(f"Best model saved at: {results.save_dir}/weights/best.pt")
print(f"Last model saved at: {results.save_dir}/weights/last.pt")
print(f"\nTraining results saved in: {results.save_dir}")