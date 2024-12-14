// YoloIntegration.cs
using UnityEngine;
using System.Collections.Generic;
using System.Net.Http;
using System.Threading.Tasks;
using UnityEngine.UI;
using System.Linq; // For LINQ methods like OrderBy

public class YoloIntegration : MonoBehaviour
{   
    public Camera cameraToCapture; // The camera capturing the view
    public RawImage debugImage;    // Optional: For showing captured image in UI
    public GameObject boundingBoxPrefab; // Prefab for bounding boxes
    public Transform canvasTransform;   // The parent canvas for the bounding box prefabs
    [SerializeField] private RectTransform overheadCameraView;

    private string serverUrl = "http://127.0.0.1:5000/detect"; // YOLO server URL
    private List<GameObject> boundingBoxes = new List<GameObject>();

    [SerializeField]
    public List<ParkingSpot> parkingSpots = new List<ParkingSpot>(); // List of parking spots


    public List<string> emptyParkingSpots = new List<string>(); // List of empty parking spot IDs

    void Start()
    {
        Debug.Log($"Parking Spots Found: {parkingSpots.Count}");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Y)) // Press 'Y' to capture and send image
        {
            CaptureAndSend();
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
        RenderTexture renderTexture = cameraToCapture.targetTexture;
        Texture2D tex = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
        RenderTexture.active = renderTexture;
        tex.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        tex.Apply();

        if (debugImage != null)
        {
            debugImage.texture = tex;
        }

        Texture2D resizedTex = ResizeTexture(tex, 416, 416);
        byte[] imageBytes = tex.EncodeToPNG();
        string yoloResponse = await SendToYOLO(imageBytes);

        if (!string.IsNullOrEmpty(yoloResponse))
        {
            ParseAndDrawBoundingBoxes(yoloResponse, tex.width, tex.height);
        }

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
        foreach (GameObject box in boundingBoxes)
        {
            Destroy(box);
        }
        boundingBoxes.Clear();

        RectTransform cameraViewRect = overheadCameraView.GetComponent<RectTransform>();
        Vector2 viewSize = cameraViewRect.rect.size;

        Detection[] detections = JsonHelper.FromJson<Detection>(json);

        // Clear the empty parking spots list before recalculating
        emptyParkingSpots.Clear();

        Debug.Log($"Parking Spots 1: {string.Join(", ", parkingSpots)}");
        
        // Check detections for each parking spot
        foreach (ParkingSpot spot in parkingSpots)
        {
            spot.IsOccupiedByYOLO(detections.ToList(), imageWidth, imageHeight);

            bool isOccupied = spot.IsOccupied;

            if (!isOccupied)
            {
                emptyParkingSpots.Add(spot.id);
            }
        }

        // Update parking spot colors based on occupancy status
        foreach (ParkingSpot spot in parkingSpots)
        {
            spot.UpdateColor();  // Call a method in ParkingSpot to update the color
        }

        Debug.Log($"Empty Parking Spots: {string.Join(", ", emptyParkingSpots)}");

        foreach (Detection detection in detections)
        {
            float xMin = detection.xmin / imageWidth;
            float yMin = detection.ymin / imageHeight;
            float xMax = detection.xmax / imageWidth;
            float yMax = detection.ymax / imageHeight;

            float flippedYMin = 1 - yMax;
            float flippedYMax = 1 - yMin;

            float localXMin = xMin * viewSize.x;
            float localYMin = flippedYMin * viewSize.y;
            float localXMax = xMax * viewSize.x;
            float localYMax = flippedYMax * viewSize.y;

            float xOffset = 10;
            float yOffset = -305;

            localXMin += xOffset;
            localXMax += xOffset;
            localYMin += yOffset;
            localYMax += yOffset;

            GameObject box = Instantiate(boundingBoxPrefab, overheadCameraView);
            RectTransform rt = box.GetComponent<RectTransform>();
            rt.anchoredPosition = new Vector2(localXMin, localYMin);
            rt.sizeDelta = new Vector2(
                localXMax - localXMin,
                localYMax - localYMin
            );

            Text label = box.GetComponentInChildren<Text>();
            if (label != null)
            {
                label.text = $"{detection.name}";
                RectTransform labelRect = label.GetComponent<RectTransform>();
                if (labelRect != null)
                {
                    labelRect.anchorMin = new Vector2(0, 1);
                    labelRect.anchorMax = new Vector2(0, 1);
                    labelRect.pivot = new Vector2(0.5f, 1);
                    labelRect.anchoredPosition = new Vector2(75, 28);
                }
            }

            boundingBoxes.Add(box);
        }
    }
}


