using UnityEngine;
using System.Linq; // For LINQ methods like OrderBy
using System.Collections.Generic; 
public class ParkingSpot : MonoBehaviour
{
    public string id; // Spot ID (e.g., "Spot_1")
    public GameObject[] parkingLines; // References to the GameObjects representing the lines
    public Rect bounds; // Combined bounds of the parking space
    public bool IsOccupied = false; // Whether the spot is occupied

    private Renderer[] renderers;
    private GameObject fillPlane; // Plane to fill the spot with color

    void Start()
    {
        
        renderers = new Renderer[parkingLines.Length];
        for (int i = 0; i < parkingLines.Length; i++)
        {
            renderers[i] = parkingLines[i].GetComponent<Renderer>();
        }

        // Calculate combined bounds from parking lines
        bounds = CalculateBounds();
        CreateFillPlane();
    }

    void Update()
    {
        // Debug visualization: change color of lines based on occupancy
        Color color = IsOccupied ? Color.red : Color.green;
        foreach (Renderer renderer in renderers)
        {
            renderer.material.color = color;
        }
    }
    private Rect CalculateBounds()
    {
        if (parkingLines.Length < 2)
        {
            Debug.LogError($"Not enough parking lines assigned to {id}");
            return new Rect();
        }

        // To store the final combined bounds
        Bounds combinedBounds = new Bounds(parkingLines[0].transform.position, Vector3.zero);

        // Iterate through all consecutive pairs of parking lines
        for (int i = 0; i < parkingLines.Length - 1; i++)
        {
            Vector3 line1Pos = parkingLines[i].transform.position;
            Vector3 line2Pos = parkingLines[i + 1].transform.position;

            // Assuming these values are correct, but we'll manually adjust the Y scale for this calculation
            Vector3 line1Scale = new Vector3(-2.42f, 1f, 5.28f);  // Correct X and Z scale, Y scale manually adjusted
            Vector3 line2Scale = new Vector3(-2.42f, 1f, 5.28f);  // Same scale for both lines

            // Calculate the distance between the two parking lines (depth of the spot)
            float distance = Vector3.Distance(line1Pos, line2Pos);

            // Ensure that the Y-scale remains correctly adjusted
            float width = Mathf.Abs(line1Scale.x);  // Width based on the x-scale
            float depth = Mathf.Abs(line1Scale.z);  // Depth based on the z-scale
            float height = Mathf.Abs(line1Scale.y); // Manually adjust if needed, but this is typically irrelevant for bounds

            // Find the center of the parking spot (midpoint between the two parking lines)
            Vector3 center = (line1Pos + line2Pos) / 2;

            // Debugging the positions, scales, and distances
            Debug.Log($"Parking Line {i+1} position: {line1Pos}, scale: {line1Scale}");
            Debug.Log($"Parking Line {i+2} position: {line2Pos}, scale: {line2Scale}");
            Debug.Log($"Distance between parking lines: {distance}");
            Debug.Log($"Calculated width: {width}, depth: {depth}, height: {height}");

            // Update the combined bounds by encapsulating the bounds of this parking spot
            combinedBounds.Encapsulate(new Bounds(center, new Vector3(width, height, depth)));
        }

        // Convert the final combined bounds to a Rect
        Rect finalBounds = new Rect(
            combinedBounds.min.x,
            combinedBounds.min.z,  // Use Z-axis for depth in 3D
            combinedBounds.size.x,
            combinedBounds.size.z
        );

        if (id == "5")  // For debugging purposes
        {
            Debug.Log($"Final Combined Bounds for {id}: Min={finalBounds.min}, Size={finalBounds.size}");
        }

        return finalBounds;
    }

    public void IsOccupiedByYOLO(List<Detection> detections, int imageWidth, int imageHeight)
    {
        bool isOccupied = false;

        foreach (var detection in detections)
        {
            Rect detectionRect = new Rect(
                detection.xmin / imageWidth,
                detection.ymin / imageHeight,
                (detection.xmax - detection.xmin) / imageWidth,
                (detection.ymax - detection.ymin) / imageHeight
            );

            // Assuming the bounds of the parking spot is a Rect
            if (this.bounds.Overlaps(detectionRect))
            {
                isOccupied = true;
                break;
            }
        }

        // Update IsOccupied status
        this.IsOccupied = isOccupied;
    }


    public void UpdateColor()
    {
        Renderer renderer = GetComponent<Renderer>(); // Assuming you have a Renderer attached to your ParkingSpot GameObject
        if (renderer != null)
        {
            if (IsOccupied)
                renderer.material.color = Color.red; // Change color to red if occupied
            else
                renderer.material.color = Color.green; // Change color to green if empty
        }
    }

    private void CreateFillPlane()
    {
        if (bounds.width <= 0 || bounds.height <= 0)
        {
            Debug.LogError($"Invalid bounds for {id}: {bounds}");
            return;
        }

        // Create the plane
        fillPlane = GameObject.CreatePrimitive(PrimitiveType.Quad);
        fillPlane.name = $"FillPlane_{id}";
        fillPlane.transform.SetParent(transform, false);

        // Position and scale
        Vector3 center = new Vector3(bounds.x + bounds.width / 2, 0.01f, bounds.y + bounds.height / 2);
        fillPlane.transform.position = center;

        fillPlane.transform.localScale = new Vector3(bounds.width, bounds.height, 1);

        // Determine the average rotation of parking lines
        Quaternion averageRotation = CalculateAverageRotation();
        fillPlane.transform.rotation = averageRotation * Quaternion.Euler(90, 0, 0);
        
        // Apply a material
        Material fillMaterial = new Material(Shader.Find("Unlit/Color"));
        fillMaterial.color = Color.clear; // Initially transparent
        fillPlane.GetComponent<Renderer>().material = fillMaterial;


        // Disable the collider
        Destroy(fillPlane.GetComponent<Collider>());
        Debug.Log($"Created fill plane for {id} at {fillPlane.transform.position} with scale {fillPlane.transform.localScale}");

    }

    private Quaternion CalculateAverageRotation()
    {
        if (parkingLines.Length == 0)
        {
            Debug.LogError($"No parking lines assigned to {id}");
            return Quaternion.identity;
        }

        Vector3 averageForward = Vector3.zero;
        Vector3 averageUp = Vector3.zero;

        foreach (GameObject line in parkingLines)
        {
            Transform lineTransform = line.transform;
            averageForward += lineTransform.forward;
            averageUp += lineTransform.up;
        }

        averageForward.Normalize();
        averageUp.Normalize();

        // Construct the average rotation
        Quaternion averageRotation = Quaternion.LookRotation(averageForward, averageUp);
        Debug.Log($"Average rotation for {id}: {averageRotation.eulerAngles}");
        return averageRotation;
    }

    public void SetFillColor(Color color)
    {
        if (fillPlane != null)
        {
            fillPlane.GetComponent<Renderer>().material.color = color;
            Debug.Log($"Set color for {id} to {color}");
        }
        else
        {
            Debug.LogError($"Fill plane not created for {id}");
        }
    }


    public void ClearFillColor()
    {
        if (fillPlane != null)
        {
            Debug.Log($"ClearFillColor Called");
            fillPlane.GetComponent<Renderer>().material.color = Color.clear; // Reset to transparent
        }
    }

    public void TestFillColors()
    {
        if (fillPlane != null)
        {   
            Debug.Log($"TestFillColors Called");
            SetFillColor(Color.blue); // Change this to any color for testing

        }
        else
        {
            Debug.LogError($"Fill plane is not created for {id}");
        }
    }


}
