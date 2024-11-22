using UnityEngine;
using System.Collections.Generic;
using System;

public class AudioManager : MonoBehaviour
{
    public static AudioManager Instance;

    [Header("Audio Settings")]
    public AudioClip[] audioClips; // Array of sound clips
    public int poolSize = 10; // Initial number of audio sources in the pool
    public AudioSource carEngineAudioSource;
    public AudioSource carAccelerationAudioSource;
    private Queue<AudioSource> audioSourcePool; // Pool of reusable audio sources

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
        }
        else
        {
            Destroy(gameObject); // Ensure only one AudioManager exists
        }

        // Create the audio source pool
        audioSourcePool = new Queue<AudioSource>();
        for (int i = 0; i < poolSize; i++)
        {
            AudioSource newAudioSource = gameObject.AddComponent<AudioSource>();
            newAudioSource.playOnAwake = false; // Prevent automatic playback
            audioSourcePool.Enqueue(newAudioSource);
        }
    }

    // Play a sound from the pool or create a new one if none are available
    public AudioSource PlaySound(AudioClip clip, Vector3 position)
    {
        AudioSource audioSource = null;
        if (audioSourcePool.Count > 0)
        {
            audioSource = audioSourcePool.Dequeue();
            audioSource.transform.position = position;
            audioSource.clip = clip;
            audioSource.Play();
            StartCoroutine(ReturnAudioSourceToPool(audioSource, clip.length)); // Return to pool after the clip finishes
        }
        return audioSource;
    }


    // Find an audio clip by name
    private AudioClip GetAudioClipByName(string name)
    {
        foreach (AudioClip clip in audioClips)
        {
            if (clip.name == name)
            {
                return clip;
            }
        }
        return null;
    }

    // Return the audio source to the pool after the clip is finished
    private System.Collections.IEnumerator ReturnAudioSourceToPool(AudioSource audioSource, float delay)
    {
        yield return new WaitForSeconds(delay); // Wait until the sound has finished
        audioSource.Stop();
        audioSourcePool.Enqueue(audioSource); // Return the AudioSource to the pool
    }

    public void HandleMovingCarSound(float blendSpeed)
    {
        carAccelerationAudioSource.volume = Mathf.Lerp(carAccelerationAudioSource.volume, 1f, blendSpeed * Time.deltaTime);
        carEngineAudioSource.volume = Mathf.Lerp(carEngineAudioSource.volume, 0f, blendSpeed * Time.deltaTime);
    }

    public void HandleIdleCarSound(float blendSpeed)
    {
        carAccelerationAudioSource.volume = Mathf.Lerp(carAccelerationAudioSource.volume, 0f, blendSpeed * Time.deltaTime);
        carEngineAudioSource.volume = Mathf.Lerp(carEngineAudioSource.volume, 1f, blendSpeed * Time.deltaTime);
    }
}
