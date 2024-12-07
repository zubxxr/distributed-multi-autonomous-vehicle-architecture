from flask import Flask, request, jsonify
import torch
from io import BytesIO
from PIL import Image
import math

app = Flask(__name__)

model_path = 'test.pt'

# Load YOLO model
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)


@app.route('/detect', methods=['POST'])
def detect():
    try:
        # Read image from request
        image = Image.open(BytesIO(request.data))
        # Perform detection
        results = model(image)

        # Apply confidence threshold explicitly
        confidence_threshold = 0.5
        detections = results.pandas().xyxy[0].to_dict(orient='records')
        filtered_detections = [
            d for d in detections if d['confidence'] >= confidence_threshold
        ]

        print(f"Filtered Detections (Confidence >= {confidence_threshold}): {filtered_detections}")  # Debugging

        car_count = 0
        processed_boxes = []

        def calculate_iou(box1, box2):
            """Calculate the Intersection over Union (IoU) between two bounding boxes."""
            x1 = max(box1['xmin'], box2['xmin'])
            y1 = max(box1['ymin'], box2['ymin'])
            x2 = min(box1['xmax'], box2['xmax'])
            y2 = min(box1['ymax'], box2['ymax'])

            inter_area = max(0, x2 - x1) * max(0, y2 - y1)
            box1_area = (box1['xmax'] - box1['xmin']) * (box1['ymax'] - box1['ymin'])
            box2_area = (box2['xmax'] - box2['xmin']) * (box2['ymax'] - box2['ymin'])

            union_area = box1_area + box2_area - inter_area
            return inter_area / union_area if union_area > 0 else 0

        def is_duplicate(box, existing_boxes, iou_threshold=0.3):
            """Check if a bounding box is a duplicate using IoU."""
            for existing_box in existing_boxes:
                if calculate_iou(box, existing_box) > iou_threshold:
                    return True
            return False

        for detection in filtered_detections:
            if detection['name'] == 'car':
                if not is_duplicate(detection, processed_boxes):
                    car_count += 1
                    processed_boxes.append(detection)

        print(f"Number of Cars: {car_count}")

        return jsonify(filtered_detections)

    except Exception as e:
        return jsonify({"error": str(e)})

if __name__ == '__main__':
    app.run(debug=True)

