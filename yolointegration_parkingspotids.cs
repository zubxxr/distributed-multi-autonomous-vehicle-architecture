using UnityEngine;
using System.Collections.Generic;
using System.Net.Http;
using System.Threading.Tasks;
using UnityEngine.UI;

public class ParkingSpot
{
    public int id;                     // Unique ID for the parking spot
    public Rect boundary;              // The boundaries of the parking spot in overhead view
    public bool isOccupied;            // Status of the parking spot
}

public class YoloIntegration : MonoBehaviour
{   
    private List<ParkingSpot> parkingSpots = new List<ParkingSpot>();

    void Start()
    {
        InitializeParkingSpots();
    }

    void InitializeParkingSpots()
    {
        parkingSpots.Add(new ParkingSpot { id = 1, boundary = new Rect(5, 5, 5, 5), isOccupied = false });
        parkingSpots.Add(new ParkingSpot { id = 2, boundary = new Rect(5, 5, 5, 5), isOccupied = false });
        parkingSpots.Add(new ParkingSpot { id = 3, boundary = new Rect(5, 5, 5, 5), isOccupied = false });
    }

    List<int> GetEmptyParkingSpots()
    {
        List<int> emptySpots = new List<int>();
        foreach (var spot in parkingSpots)
        {
            if (!spot.isOccupied)
            {
                emptySpots.Add(spot.id);
            }
        }
        return emptySpots;
    }

    public Camera cameraToCapture; // The camera capturing the view
    public RawImage debugImage;    // Optional: For showing captured image in UI
    public GameObject boundingBoxPrefab; // Prefab for bounding boxes
    public Transform canvasTransform;   // The parent canvas for the bounding box prefabs
    [SerializeField] private RectTransform overheadCameraView;

    private string serverUrl = "http://127.0.0.1:5000/detect"; // YOLO server URL
    private List<GameObject> boundingBoxes = new List<GameObject>();

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Y)) // Press 'Y' to capture and send image
        {
            CaptureAndSend();
        }

        if (Input.GetKeyDown(KeyCode.E)) // Press 'E' to check for empty spots
        {
            List<int> emptySpots = GetEmptyParkingSpots();
            Debug.Log($"Empty Spots: {string.Join(", ", emptySpots)}");
        }
    }

    Texture2D ResizeTexture(Texture2D source, int newWidth, int newHeight)
    {
        RenderTexture rt = RenderTexture.GetTemporary(newWidth, newHeight);
        Graphics.Blit(source, rt);
        RenderTexture.active = rt;

        Texture2D newTex = new Texture2D(newWidth, newHeight, TextureFormat.RGB24, false);
        newTex.ReadPixels(new Rect(0, 0, newWidth, newHeight), 0, 0);
        newTex.Apply();

        RenderTexture.ReleaseTemporary(rt);
        return newTex;
    }

    async void CaptureAndSend()
    {
        // Capture the camera view to a texture
        RenderTexture renderTexture = cameraToCapture.targetTexture;
        Texture2D tex = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
        RenderTexture.active = renderTexture;
        tex.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        tex.Apply();

        // Display captured image in debug UI (optional)
        if (debugImage != null)
        {
            debugImage.texture = tex;
        }

        Texture2D resizedTex = ResizeTexture(tex, 416, 416);

        // Convert to PNG and send to YOLO server
        byte[] imageBytes = tex.EncodeToPNG();
        string yoloResponse = await SendToYOLO(imageBytes);

        // Parse YOLO response and draw bounding boxes
        if (!string.IsNullOrEmpty(yoloResponse))
        {
            ParseAndDrawBoundingBoxes(yoloResponse, tex.width, tex.height);
        }

        // Cleanup
        Destroy(tex);
        RenderTexture.active = null;
    }

    async Task<string> SendToYOLO(byte[] imageBytes)
    {
        using (HttpClient client = new HttpClient())
        {
            try
            {
                ByteArrayContent content = new ByteArrayContent(imageBytes);
                HttpResponseMessage response = await client.PostAsync(serverUrl, content);
                response.EnsureSuccessStatusCode();
                return await response.Content.ReadAsStringAsync();
            }
            catch (HttpRequestException e)
            {
                Debug.LogError($"YOLO Server Error: {e.Message}");
                return null;
            }
        }
    }

    void ParseAndDrawBoundingBoxes(string json, int imageWidth, int imageHeight)
    {
        // Clear previous bounding boxes
        foreach (GameObject box in boundingBoxes)
        {
            Destroy(box);
        }
        boundingBoxes.Clear();

        // Get the RectTransform of OverheadCameraView
        RectTransform cameraViewRect = overheadCameraView.GetComponent<RectTransform>();
        Vector2 viewSize = cameraViewRect.rect.size; // Width and height of the camera view in canvas space

        // Parse JSON response
        Detection[] detections = JsonHelper.FromJson<Detection>(json);

        // Reset parking spot statuses
        foreach (var spot in parkingSpots)
        {
            spot.isOccupied = false;
        }

        foreach (Detection detection in detections)
        {
            // Convert YOLO coordinates to normalized space (0 to 1)
            float xMin = detection.xmin / imageWidth;
            float yMin = detection.ymin / imageHeight;
            float xMax = detection.xmax / imageWidth;
            float yMax = detection.ymax / imageHeight;


            float flippedYMin = 1 - yMax;
            float flippedYMax = 1 - yMin;

            // Scale normalized coordinates to local space within OverheadCameraView

            float localXMin = xMin * viewSize.x;
            float localYMin = flippedYMin * viewSize.y;
            float localXMax = xMax * viewSize.x;
            float localYMax = flippedYMax * viewSize.y;

            // Calculate the center of the bounding box in local space
            float centerX = (localXMin + localXMax) / 2;
            float centerY = (localYMin + localYMax) / 2;

            // Check which parking spot the detection overlaps
            foreach (var spot in parkingSpots)
            {
                if (spot.boundary.Contains(new Vector2(centerX, centerY)))
                {
                    spot.isOccupied = true;
                    Debug.Log($"Detection '{detection.name}' overlaps with Parking Spot {spot.id}");
                    break;
                }
                
            }

            // Apply a manual Y-offset adjustment to align the boxes to the camera view
            float xOffset = 10; // Adjust this value to fine-tune alignment
            float yOffset = -305; // Adjust this value to fine-tune alignment

            localXMin += xOffset;
            localXMax += xOffset;
            localYMin += yOffset;
            localYMax += yOffset;

            // Create a bounding box prefab
            GameObject box = Instantiate(boundingBoxPrefab, overheadCameraView);
            RectTransform rt = box.GetComponent<RectTransform>();

            // Set the bounding box position relative to OverheadCameraView
            rt.anchoredPosition = new Vector2(localXMin, localYMin);

            // Set the size of the bounding box
            rt.sizeDelta = new Vector2(
                localXMax - localXMin, // Width
                localYMax - localYMin  // Height
            );

            // Debug.Log($"Bounding Box Position: {rt.anchoredPosition}, Size: {rt.sizeDelta}");

            Text label = box.GetComponentInChildren<Text>();
            if (label != null)
            {
                // Set the label text
                // label.text = $"{detection.name} ({detection.confidence:F2})";
                label.text = $"{detection.name}";

                // Adjust the label position relative to the bounding box
                RectTransform labelRect = label.GetComponent<RectTransform>();

                if (labelRect != null)
                {
                    labelRect.anchorMin = new Vector2(0, 1);
                    labelRect.anchorMax = new Vector2(0, 1);
                    labelRect.pivot = new Vector2(0.5f, 1);

                    labelRect.anchoredPosition = new Vector2(75, 22); // Centered at the top
                }
            }
            boundingBoxes.Add(box);
        }
        
        foreach (var spot in parkingSpots) 
        {
            Debug.Log($"Parking Spot {spot.id} is {(spot.isOccupied ? "occupied" : "empty")}");
            Debug.Log($"Spot {spot.id}: Boundary {spot.boundary}");
        }
    }
}

// Helper classes
[System.Serializable]
public class Detection
{
    public float xmin, ymin, xmax, ymax;
    public string name;
    public float confidence;
}

// Helper for JSON array parsing
public static class JsonHelper
{
    public static T[] FromJson<T>(string json)
    {
        string newJson = "{\"Items\":" + json + "}";
        Wrapper<T> wrapper = JsonUtility.FromJson<Wrapper<T>>(newJson);
        return wrapper.Items;
    }

    [System.Serializable]
    private class Wrapper<T>
    {
        public T[] Items;
    }
}
