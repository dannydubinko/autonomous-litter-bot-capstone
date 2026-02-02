from ultralytics import YOLO
import os

# Fix the path for Linux (forward slashes)
pt_path = "./segmentation_nano.pt"

if os.path.exists(pt_path):
    model = YOLO(pt_path)
    # This creates the 'best_openvino_model/' folder
    model.export(format="openvino", imgsz=640) 
    print("Export complete!")
else:
    print(f"Error: Could not find the .pt file at {pt_path}")