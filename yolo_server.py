from flask import Flask, request, jsonify
import torch
from io import BytesIO
from PIL import Image

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
        
        # Check if the results contain detections
        detections = results.pandas().xyxy[0].to_dict(orient='records')
        print(f"Detected Objects: {detections}")  # Debugging line
    
        formatted_detections = []
        car_count = 0

        for detection in detections:
            if detection['name'] == 'car':
                car_count += 1

            formatted_detections.append({
                "xmin": detection['xmin'],
                "ymin": detection['ymin'],
                "xmax": detection['xmax'],
                "ymax": detection['ymax'],
                "name": detection['name'],
                "confidence": detection['confidence']
            })
            
        print(f"Number of Cars: {car_count}")

        # return jsonify({"detections": formatted_detections})
        return jsonify(formatted_detections)

    except Exception as e:
        return jsonify({"error": str(e)})

if __name__ == '__main__':
    app.run(debug=True)

