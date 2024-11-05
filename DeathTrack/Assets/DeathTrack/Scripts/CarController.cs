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
    [Header("Car Drag Variables")]
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


    [Space(10)]
    [Header("Acceleration And Braking Variables")]

    [Tooltip("Transform representing the point where acceleration forces are applied.")]
    [SerializeField] private Transform accelerationPoint;
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
        Movement();
    }

    private void Update()
    {
        GetPlayerInput();
        AdjustDampingForGroundedState();
        SteeringInputAndWheelRotation();
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

        carRB.AddForceAtPosition(carRB.mass * acceleration * steeringDirection, wheelTransforms[i].position);
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

    #region Movement Methods

    /// <summary>
    /// Handles Acceleration and Deceleration
    /// </summary>
    private void Movement()
    {
        if (isGrounded)
        {
            Acceleration();
            Deceleration();
        }
    }

    /// <summary>
    /// Applies acceleration force to the car based on player input.
    /// This method uses the <c>moveInput</c> value to determine the acceleration force 
    /// applied in the forward direction of the car. The force is calculated by multiplying
    /// the acceleration scalar with the player�s input and the car's forward direction.
    /// 
    /// The acceleration force is applied at the specified <c>accelerationPoint</c>
    /// to simulate realistic vehicle movement.
    private void Acceleration()
    {
        carRB.AddForceAtPosition(acceleration * moveInput * transform.forward, accelerationPoint.position, ForceMode.Acceleration);
    }

    /// <summary>
    /// Applies deceleration force to the car based on player input.
    /// This method uses the <c>moveInput</c> value to determine the deceleration force 
    /// applied in the backward direction of the car. The force is calculated by multiplying
    /// the deceleration scalar with the player�s input and the opposite direction of the car's forward direction.
    /// 
    /// The deceleration force is applied at the specified <c>accelerationPoint</c>
    /// to simulate braking effects on the vehicle.
    private void Deceleration()
    {
        carRB.AddForceAtPosition(deceleration * moveInput * -transform.forward, accelerationPoint.position, ForceMode.Acceleration);
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
    private void SteeringInputAndWheelRotation()
    {
        float steerInput = Input.GetAxis("Horizontal");

        // Calculate target steering angle based on input
        float targetAngle = steerAngle * steerInput;

        // Apply steering rotation to the front wheel parents
        wheelTransforms[0].localRotation = Quaternion.Euler(0, targetAngle, 0);
        wheelTransforms[1].localRotation = Quaternion.Euler(0, targetAngle, 0);

        // Calculate the rolling rotation based on move input
        float rotationAmount = moveInput * wheelModelRotationSpeed * Time.deltaTime;

        for (int i = 0; i < 2; i++)
        {
            Quaternion steerRotation = Quaternion.Euler(0, targetAngle, 0);
            Quaternion rollRotation = Quaternion.Euler(rotationAmount, 0, 0);

            // Combine steering and rolling rotations
            wheelModels[i].localRotation = steerRotation * rollRotation;
        }

        // Apply only the rolling rotation to the back wheels (wheelModels[2] and wheelModels[3])
        for (int i = 2; i < wheelModels.Count; i++)
        {
            wheelModels[i].Rotate(Vector3.right * rotationAmount, Space.Self);
        }
    }


    #endregion


}