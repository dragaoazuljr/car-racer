using System.Collections.Generic;
using UnityEngine;

public class CarMoviment : MonoBehaviour
{
    [SerializeField]
    public float speed = 10f;
    [SerializeField]
    public float springStrength = 1f;
    [SerializeField]
    public float springDamper= 1f;
    [SerializeField]
    [Range(0f, 2f)]
    public float tireGripFactor = 0.5f;
    [SerializeField]
    public bool ForwardWheelDrive = true;
    [SerializeField]
    public float tireMass = 1f;
    [SerializeField]
    public GameObject centerOfMass;
    [SerializeField]
    [Range(0, 1f)]
    public float springTravelDistance = 1f;
    [SerializeField]
    [Range(0.75f, 1f)]
    public float wheelTurningDistance = 0.05f;
    [SerializeField]
    [Range(0f, 10f)]
    public float wheelTurningVelocity;

    [SerializeField]
    List<GameObject> carWheels;

    Rigidbody rb;
    float throttleInput;
    float steerInput;

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        Transform centerOfMassTransform = centerOfMass.GetComponent<Transform>();
        rb.centerOfMass = centerOfMassTransform.position;
    }

    void OnDrawGizmos() {
        foreach (GameObject wheel in carWheels) {
          Transform wheelT = wheel.GetComponent<Transform>();

          // Gizmos.color = Color.red;
          // Gizmos.DrawLine(wheelT.position, wheelT.position + transform.forward);

          // Gizmos.color = Color.green;
          // Gizmos.DrawLine(wheelT.position, wheelT.position + wheelT.forward);

         // Gizmos.color = Color.white;
         // Gizmos.DrawLine(wheelT.position, wheelT.position + (Vector3.down * multiplier));
        }
    }

    // Update is called once per frame
    void Update()
    { 
        //position camera behind car
        Vector3 offset = new Vector3(0f, 5f, -7f);

        throttleInput = Input.GetAxisRaw("Acceleration");
        steerInput = Input.GetAxisRaw("Steer");
        bool spaceBar = Input.GetKey(KeyCode.Space);

        if (spaceBar) {
          transform.Rotate(transform.rotation.x, transform.rotation.y, transform.rotation.z * Time.deltaTime);
        }
    }

    void FixedUpdate() {
      RotateFrontWheels(steerInput);

      // Raycast suspention for each wheel;
      foreach (GameObject whellGameObj in carWheels) {
        Transform wheelTransform = whellGameObj.GetComponent<Transform>();
        
        Ray ray = new Ray(wheelTransform.position, wheelTransform.TransformDirection(Vector3.down));
        RaycastHit hit;
        Debug.DrawRay(ray.origin, ray.direction * 0.3f);

        if (Physics.Raycast(ray, out hit)) {
          if (hit.distance <= springTravelDistance) {
            Vector3 tireWorldVelocity = rb.GetPointVelocity(wheelTransform.position);
            CalculateSuspentionForce(wheelTransform, tireWorldVelocity, hit.distance);
            CalculateSteeringForce(wheelTransform, tireWorldVelocity);

            if (throttleInput != 0) {
              ApplyThrottleAcceleration(throttleInput);
            }
          }
        }
      }
    }

    void CalculateSuspentionForce(Transform wheelTransform, Vector3 tireWorldVelocity, float hitDistance) {
      Vector3 springDir = wheelTransform.up;

      float offset = springTravelDistance - hitDistance;
      float vel = Vector3.Dot(springDir, tireWorldVelocity);

      float force = (offset * springStrength) - (vel * springDamper);

      Ray suspentionForceRay = new Ray(wheelTransform.position, wheelTransform.TransformDirection(springDir));
      Debug.DrawRay(wheelTransform.position, wheelTransform.TransformDirection(springDir) * force);

      rb.AddForceAtPosition(springDir * force, wheelTransform.position);
    }

    void CalculateSteeringForce(Transform wheelTransform, Vector3 tireWorldVelocity) {
      Vector3 steeringDir = wheelTransform.right;

      float steeringVel = Vector3.Dot(steeringDir, tireWorldVelocity);

      float desiredVelChange = -steeringVel * tireGripFactor;

      float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

      Vector3 force = steeringDir * tireMass * desiredAccel;

      Ray steeringDirectionForceRay = new Ray(wheelTransform.position, wheelTransform.TransformDirection(force));
      Debug.DrawRay(wheelTransform.position, wheelTransform.TransformDirection(force), Color.red);

      rb.AddForceAtPosition(force, wheelTransform.position);
    }

    void ApplyThrottleAcceleration (float throttleInput) {
      int index = 0;
      foreach (GameObject wheel in carWheels) {
        if (ForwardWheelDrive && index <= 1) {
          addForceToProperWheel(wheel, throttleInput);
        } else if (!ForwardWheelDrive && index >= 2) {
          addForceToProperWheel(wheel, throttleInput);
        }

        index++;
      }
    }

    void addForceToProperWheel(GameObject wheel, float throttleInput) {
      Transform wheelTransform = wheel.GetComponent<Transform>();
      Vector3 wheelPointedDir = wheelTransform.forward;

      Vector3 force = wheelPointedDir * throttleInput * speed;

      Ray throttleForceRay = new Ray(wheelTransform.position, wheelTransform.TransformDirection(wheelTransform.forward));
      Debug.DrawRay(wheelTransform.position, force, Color.blue);

      rb.AddForceAtPosition(force, wheelTransform.position);
    }

    void RotateFrontWheels (float input) {
      int index = 0;
      foreach (GameObject wheelGameObj in carWheels) {
        if (index <= 1) {
          Transform wheel = wheelGameObj.GetComponent<Transform>();
          Vector3 wheelCarCross = Vector3.Cross(wheel.forward, transform.forward);
          float wheelCarFowardDot = Vector3.Dot(wheel.forward, transform.forward);

          Ray wheelFowardRay = new Ray(wheel.position, wheel.forward);
          Ray carFowardRay = new Ray(wheel.position, transform.forward);
          Ray crossFowardRay = new Ray(wheel.position, wheelCarCross);

          Debug.DrawRay(wheelFowardRay.origin, wheelFowardRay.direction, Color.yellow);
          Debug.DrawRay(carFowardRay.origin, carFowardRay.direction, Color.magenta);
          Debug.DrawRay(crossFowardRay.origin, crossFowardRay.direction, Color.white);


          if (input != 0 && wheelCarFowardDot >= wheelTurningDistance) {
            wheel.Rotate(new Vector3(0, input * wheelTurningVelocity, 0));
          } else if (wheelCarCross.y > 0  && wheelCarFowardDot != 1 && input == 0) {
            if (wheelCarCross.y >= 0.01f) {
              wheel.Rotate(0, (1 - wheelCarCross.y) * wheelTurningVelocity, 0);
            }
          } else if (wheelCarCross.y < 0 && wheelCarFowardDot != 1 && input == 0) {
            if (wheelCarCross.y <= -0.01f) {
              wheel.Rotate(0, wheelCarCross.y * wheelTurningVelocity, 0);
            }
          }
        };

        index++;
      }
    }
}
