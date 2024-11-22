using NUnit.Framework;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;
using static UnityEditor.Experimental.GraphView.GraphView;

[RequireComponent(typeof(Rigidbody))]   
public class CarController : MonoBehaviour
{
    #region SerializedVariables
    [Header("Core Components")]

    [Tooltip("Reference to the car's Rigidbody component, used for physics calculations.")]
    [SerializeField] private Rigidbody carRB;

    [Tooltip("List of transforms representing the positions of the car's wheels.")]
    [SerializeField] private List<Transform> wheelTransforms = new List<Transform>();

    [Space(10)]
    [Header("Car  variables")]
    [SerializeField] private float groundedDrag;
    [SerializeField] private float airDrag;
    [Space(10)]
    [Header("Suspension Variables")]

    [Tooltip("Strength of the suspension springs; higher values result in stiffer suspension.")]
    [SerializeField] private float suspensionStrength;

    [Tooltip("Damping force to reduce oscillations in the suspension system.")]
    [SerializeField] private float dampingForce;
    [SerializeField] private float springTravel;
    [SerializeField] private float wheelRadius;
    [SerializeField] private float restingDistance;
    [SerializeField] private LayerMask drivableLayerMask;


    [Space(10)]
    [Header("Steering Variables")]

    [Tooltip("Factor affecting tire grip; higher values improve traction.")]
    [SerializeField] private float tireGripFactor;
    [SerializeField] private float tireMass;

    [Space(10)]
    [Header("Acceleration And Braking Variables")]

    [Tooltip("Transform representing the point where acceleration forces are applied.")]
    [SerializeField] private Transform accelerationPoint;
    [SerializeField] private AnimationCurve powerCurve;
    [SerializeField] private float acceleration = 25f;
    [SerializeField] private float maxSpeed = 100f;
    [SerializeField] private float deceleration = 10f;

    [Space(10)]
    [Header("BouncingVariables")]
    [SerializeField] private float bounceForce;


    [Space(10)]
    [Header("Model References")]
    [SerializeField] private List<Transform> wheelModels = new List<Transform>();
    [SerializeField] private float wheelModelRotationSpeed;

    [Space(10)]
    [Header("Sound Settings")]
    [SerializeField] private float blendSpeed = 2f;
    [SerializeField] private float accelerationThreshold = 2f;


    #endregion

    #region PrivateVariables

    private float moveInput = 0;
    private int[] wheelsIsGrounded = new int[4]; //1 represents true and 0 represents false
    private bool isGrounded = false;
    public float steerAngle = 30f;

    #endregion

    #region Unity Lifecycle Methods
    private void FixedUpdate()
    {
        HandleBounceCar();
        ProcessWheelGrounding();
        isGrounded = GroundCheck();
        //Movement();
    }

    private void Update()
    {
        GetPlayerInput();
        AdjustDampingForGroundedState();
        SteeringInputAndWheelRotation();
        HandleSounds();
    }

    #endregion

    #region Car Mechanics

    /// <summary>
    /// Handle Bounce of the car
    /// </summary>
    private void HandleBounceCar()
    {
        if (Input.GetKeyDown(KeyCode.B))
        {
            BounceCar();
        }
    }

    /// <summary>
    /// This method randomly selects a wheel and apply upward force at that wheel's position
    /// </summary>
    private void BounceCar()
    {
        int randomIndex = Random.Range(0, wheelTransforms.Count);
        Transform randomTransform = wheelTransforms[randomIndex];
        carRB.AddForceAtPosition((Vector3.up) * bounceForce, randomTransform.position, ForceMode.Impulse);
    }

    /// <summary>
    /// Casts a ray down from each wheel and apply suspension and steering
    /// </summary>
    private void ProcessWheelGrounding()
    {
        for (int i = 0; i < wheelTransforms.Count; i++)
        {
            if (Physics.Raycast(wheelTransforms[i].position, -wheelTransforms[i].up, out RaycastHit hit, restingDistance, drivableLayerMask))
            {
                wheelsIsGrounded[i] = 1;
                ApplySuspension(i, hit);
                ApplySteering(i);
                
                Vector3 accelerationDirection = wheelTransforms[i].forward;

                if(moveInput!=0)
                {
                    float carSpeed = Vector3.Dot(carRB.transform.forward, carRB.linearVelocity);

                    float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / maxSpeed);


                    float availableTorque = powerCurve.Evaluate(normalizedSpeed) * moveInput;

                    carRB.AddForceAtPosition(accelerationDirection * availableTorque, wheelTransforms[i].position);
                }

            }
            else
            {
                wheelsIsGrounded[i] = 0;
               
            }
        }
    }

    /// <summary>
    /// Applies suspension force to the specified wheel based on its grounded status.
    /// This method calculates the suspension force using the formula:
    /// <c>Force = (Offset * SpringStrength) - (Velocity * Damping)</c>,
    /// where:
    /// <list type="bullet">
    /// <item><description><c>Offset</c> is the normalized difference between the resting distance and the actual distance from the ground.</description></item>
    /// <item><description><c>SpringStrength</c> determines the stiffness of the suspension.</description></item>
    /// <item><description><c>Velocity</c> is the current velocity of the wheel along the suspension direction.</description></item>
    /// <item><description><c>Damping</c> is the force that reduces oscillations in the suspension system.</description></item>
    /// </list>
    /// The calculated suspension force is then applied at the wheel's position.
    /// </summary>
    /// <param name="i">The index of the wheel in the <c>wheelTransforms</c> list.</param>
    /// <param name="hit">The raycast hit information that contains details about the ground contact.</param>
    private void ApplySuspension(int i, RaycastHit hit)
    {
        Vector3 suspensionDir = wheelTransforms[i].up;

        Vector3 tireSuspensionWorldVel = carRB.GetPointVelocity(wheelTransforms[i].position);

        float velocity = Vector3.Dot(suspensionDir, tireSuspensionWorldVel);

        float currentSpringLength = hit.distance - wheelRadius;

        float offset = (restingDistance - currentSpringLength) / springTravel;

        float suspensionForce = (offset * suspensionStrength) - (velocity * dampingForce);

        carRB.AddForceAtPosition(suspensionForce * suspensionDir, wheelTransforms[i].position);
    }

    /// <summary>
    /// Applies steering force to the specified wheel based on the current steering input.
    /// This method calculates the desired steering velocity and applies a force to the wheel 
    /// to simulate the steering effect. The force is determined using the formula:
    /// <c>Force = Mass * (DesiredVelocity / Time.fixedDeltaTime) * SteeringDirection</c>.
    /// 
    /// <list type="bullet">
    /// <item><description><c>SteeringDirection</c> is the right direction of the wheel, used to apply lateral forces.</description></item>
    /// <item><description><c>TireSteeringWorldVelocity</c> represents the current velocity of the wheel in world space.</description></item>
    /// <item><description><c>Velocity</c> is calculated as the dot product of the steering direction and the tire's world velocity.</description></item>
    /// <item><description><c>DesiredVelocity</c> is the target velocity for the wheel based on the current steering input, scaled by the tire grip factor.</description></item>
    /// </list>
    /// The resulting force is applied at the wheel's position to achieve the desired steering effect.
    /// </summary>
    /// <param name="i">The index of the wheel in the <c>wheelTransforms</c> list.</param>
    private void ApplySteering(int i)
    {
        Vector3 steeringDirection = wheelTransforms[i].right;

        Vector3 tireSteeringWorldVel = carRB.GetPointVelocity(wheelTransforms[i].position);

        float vel = Vector3.Dot(steeringDirection, tireSteeringWorldVel);

        float desiredVel = -vel * tireGripFactor;

        float acceleration = desiredVel / Time.fixedDeltaTime;

        carRB.AddForceAtPosition(tireMass * acceleration * steeringDirection, wheelTransforms[i].position);
    }

    /// <summary>
    /// returns true if two or more wheel are grounded
    /// </summary>
    /// <returns></returns>
    private bool GroundCheck()
    {
        int wheelGroundedCount = 0;
        for (int i = 0; i < wheelsIsGrounded.Length; i++)
        {
            if (wheelsIsGrounded[i] == 1)
            {
                wheelGroundedCount++;
            }
        }
        return wheelGroundedCount > 1;
    }

    /// <summary>
    /// Check if grounded and adjusts damping of rigidbody accordingly
    /// </summary>
    private void AdjustDampingForGroundedState()
    {
        if (!isGrounded)
        {
            carRB.linearDamping = airDrag;
        }
        else
        {
            carRB.linearDamping = groundedDrag;
        }
    }

    #endregion



    #region PlayerInput

    /// <summary>
    /// Retrieves the player's input for vertical movement (acceleration and braking).
    /// This method uses the Unity Input system to get the value from the "Vertical" axis,
    /// which typically corresponds to the W/S or Up/Down arrow keys or joystick vertical input.
    /// The value is stored in <c>moveInput</c>, which is later used to apply acceleration
    /// and deceleration forces to the vehicle.
    /// </summary>
    private void GetPlayerInput()
    {
        moveInput = Input.GetAxis("Vertical");
    }


    private float[] wheelRollAngles = new float[4];

    /// <summary>
    /// Retrieves the player's input for steering and applies rotation to the front wheels.
    /// This method uses the Unity Input system to get the value from the "Horizontal" axis,
    /// which typically corresponds to the A/D or Left/Right arrow keys or joystick horizontal input.
    /// 
    /// The target rotation for the front wheels is calculated by multiplying the steering input
    /// by the maximum steering angle (<c>steerAngle</c>).
    /// The calculated rotation is then applied to the local rotation of the front tires
    /// to simulate steering behavior.
    /// </summary>
    // Track each wheel's cumulative rolling rotation on the X-axis
    private void SteeringInputAndWheelRotation()
    {
        float steerInput = Input.GetAxis("Horizontal");
        float targetAngle = steerAngle * steerInput;
        float rotationAmount = moveInput * wheelModelRotationSpeed * Time.deltaTime;
        wheelTransforms[0].localRotation = Quaternion.Euler(0, targetAngle, 0);
        wheelTransforms[1].localRotation = Quaternion.Euler(0, targetAngle, 0);
        for (int i = 0; i < wheelModels.Count; i++)
        {
            if (i < 2) // Front wheels: Apply both steering and rolling
            {
                // Apply the steering rotation based on target angle
                Quaternion steerRotation = Quaternion.Euler(0, targetAngle, 0);

                // Update and apply the rolling rotation independently
                wheelRollAngles[i] += rotationAmount;
                Quaternion rollRotation = Quaternion.Euler(wheelRollAngles[i], 0, 0);

                // Combine the steering and rolling rotations
                wheelModels[i].localRotation = steerRotation * rollRotation;
            }
            else // Back wheels: Apply only rolling
            {
                // Update and apply the rolling rotation only
                wheelRollAngles[i] += rotationAmount;
                wheelModels[i].localRotation = Quaternion.Euler(wheelRollAngles[i], 0, 0);
            }
        }
    }


    #endregion

    #region Sound
    private void HandleSounds()
    {
        if (moveInput >= accelerationThreshold || moveInput <= -accelerationThreshold)
        {
            AudioManager.Instance.HandleMovingCarSound(blendSpeed);

        }
        else
        {
            AudioManager.Instance.HandleIdleCarSound(blendSpeed);

        }
    }
    #endregion

}