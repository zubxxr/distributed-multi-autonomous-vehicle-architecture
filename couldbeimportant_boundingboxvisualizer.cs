using UnityEngine;

public class BoundingBoxVisualizer : MonoBehaviour
{
    public Color boxColor = Color.red; // Color of the bounding box
    public Detection[] detections;    // Array of detections for debugging
    public int imageWidth = 1920;     // Width of the input image (YOLO model input)
    public int imageHeight = 1080;    // Height of the input image (YOLO model input)

    private void OnDrawGizmos()
    {
        if (detections == null || detections.Length == 0)
            return;

        Gizmos.color = boxColor;

        foreach (var detection in detections)
        {
            // Normalize YOLO bounding box coordinates to screen space
            float xMin = detection.xmin / imageWidth * Screen.width;
            float yMin = (1 - detection.ymax / imageHeight) * Screen.height; // Invert Y-axis
            float width = (detection.xmax - detection.xmin) / imageWidth * Screen.width;
            float height = (detection.ymax - detection.ymin) / imageHeight * Screen.height;

            // Convert to a Rect for Gizmos.DrawWireCube
            Vector3 center = new Vector3(xMin + width / 2, yMin + height / 2, 0);
            Vector3 size = new Vector3(width, height, 0);

            // Draw the bounding box
            Gizmos.DrawWireCube(center, size);
        }
    }
}
