using UnityEngine;

public class TopDownCameraFollow : MonoBehaviour
{
    public Transform player;  // Reference to the player
    public Vector3 offset;    // Offset from the player


    void LateUpdate()
    {
        if (player != null)
        {
            // Follow player's X and Z positions, keep camera's Y fixed
            Vector3 newPosition = new Vector3(player.position.x + offset.x, transform.position.y, player.position.z + offset.z);
            transform.position = newPosition;
        }
    }
}
