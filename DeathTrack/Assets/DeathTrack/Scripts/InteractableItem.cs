using UnityEngine;

public class InteractableItem : MonoBehaviour
{
    [SerializeField] private Rigidbody rb;
    [SerializeField] private AudioClip audioSource;
    [SerializeField] private float forceFactor;
    [SerializeField] private float minVelocity;

    private AudioSource lastAudioSource = null;
    private void Start()
    {
        rb = GetComponent<Rigidbody>(); 
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "Car" || collision.gameObject.tag == "DominoBrick")
        {
            Vector3 velocityDirection = collision.relativeVelocity;
            rb.AddForceAtPosition(velocityDirection * forceFactor, collision.GetContact(0).point);
     
        }

        if (collision.relativeVelocity.magnitude >= minVelocity && (lastAudioSource ==null || !lastAudioSource.isPlaying ))
        {
            lastAudioSource = AudioManager.Instance.PlaySound(audioSource,transform.position);
        }
    }
}
