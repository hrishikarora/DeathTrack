using UnityEngine;

public class WheelController : MonoBehaviour
{
    [SerializeField] private Rigidbody carRB;
    [SerializeField] private LayerMask layer;
    [SerializeField] private AnimationCurve powerCurve;

    [SerializeField] private float carTopSpeed;
    [SerializeField] private float restingDistance;
    [SerializeField] private float springStrength;
    [SerializeField] private float dampingForce;
    [SerializeField] private float bounceForce;
    [SerializeField] private float tireGripFactor;
    [SerializeField] private float gravityForce = 1000f;
    [SerializeField] private float groundedDrag;
    [SerializeField] private float airDrag;
    [SerializeField] private float airborneTorqueFactor = 0.1f;  // Reduced torque for airborne state
    [SerializeField] private float angularDamping = 0.1f;  // Reduced torque for airborne state

    private Transform wheelTransform;
    private bool isGrounded = true;
    private void Start()
    {
        wheelTransform = this.transform;
    }

    private void Update()
    {
        if (Physics.Raycast(wheelTransform.position, -wheelTransform.up, out RaycastHit hit, restingDistance, layer))
        {
            isGrounded = true;

            Vector3 suspensionDir = wheelTransform.up;
            /// Force = (offset * strenght) - (Velocity * Damping)

            Vector3 tireSuspensionWorldVel = carRB.GetPointVelocity(wheelTransform.position);

            float velocity = Vector3.Dot(suspensionDir, tireSuspensionWorldVel);

            float offset = restingDistance - hit.distance;

            float suspensionForce = (offset * springStrength) - (velocity * dampingForce);


            carRB.AddForceAtPosition(suspensionForce * suspensionDir, wheelTransform.position);



            //For steering drifiting force is F = M A

            Vector3 steeringDirection = wheelTransform.right;

            Vector3 tireSteeringWorldVel = carRB.GetPointVelocity(wheelTransform.position);

            float vel = Vector3.Dot(steeringDirection, tireSteeringWorldVel);

            float desiredVel = -vel * tireGripFactor;

            float acceleration = desiredVel / Time.fixedDeltaTime;

            carRB.AddForceAtPosition(carRB.mass * acceleration * steeringDirection, wheelTransform.position);





        }
        else
        {
            carRB.angularVelocity *= (1 - angularDamping * Time.fixedDeltaTime);

            if (carRB.transform.position.y > transform.position.y)
            {
                carRB.AddForceAtPosition(transform.up * gravityForce, transform.position);
            }
            else
            {
                carRB.AddForceAtPosition(transform.up * -gravityForce, transform.position);
            }
        }
        //Acceleration , Deceleration
        Vector3 accelerationDirection = wheelTransform.forward;
        float carSpeed = Vector3.Dot(accelerationDirection, carRB.linearVelocity);
        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);

        // Apply reduced torque if airborne
        float inputFactor = isGrounded ? Input.GetAxis("Vertical") : 0;

       

        float torque = isGrounded ? 1.0f : airborneTorqueFactor;

        float availableTorque = powerCurve.Evaluate(normalizedSpeed) * inputFactor * torque;

        carRB.AddForceAtPosition(accelerationDirection * availableTorque, wheelTransform.position);
 
    }
}