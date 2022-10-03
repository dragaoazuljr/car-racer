using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMoviment : MonoBehaviour
{
    [SerializeField]
    public GameObject car;

    [SerializeField]
    [Range(0f, 1f)]
    public float cameraSensibility = 0.9f;

    private Transform carTransform;
    private Vector3 cameraOffset;

    // Start is called before the first frame update
    void Start()
    {
      carTransform = car.GetComponent<Transform>();

      cameraOffset = new Vector3(0, 6, -8);
    }

    // Update is called once per frame
    void Update()
    {
      Vector3 cameraRotateAxis = -carTransform.forward;
      float rotateValue = Vector3.Dot(transform.forward, carTransform.forward);

      if (rotateValue < cameraSensibility ) {
        transform.position = carTransform.position + cameraOffset + carTransform.forward;
        transform.RotateAround(carTransform.position, transform.up, -rotateValue);
      } else {
        transform.position = carTransform.position + cameraOffset;
      }
    }
}
