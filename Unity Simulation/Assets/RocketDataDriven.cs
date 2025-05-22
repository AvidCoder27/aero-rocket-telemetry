using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;

public class RocketDataDriven : MonoBehaviour
{

    struct Datum {
        public float timestamp;
        public float dt;
        public float accelX;
        public float accelY;
        public float accelZ;
        public float gyroX;
        public float gyroY;
        public float gyroZ;
    }

    private Rigidbody rb;
    private List<Datum> data = new List<Datum>();

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("Rigidbody not found on the Rocket GameObject.");
        }
    }

    private void Start()
    {
        using(var reader = new StreamReader("./unity_data.csv"))
        {
            while (!reader.EndOfStream)
            {
                var line = reader.ReadLine();
                var values = line.Split(',');
                if (values.Length == 8)
                {
                    Datum datum = new Datum();
                    datum.timestamp = float.Parse(values[0]);
                    datum.dt = float.Parse(values[1]);
                    datum.accelX = float.Parse(values[2]);
                    datum.accelY = float.Parse(values[3]);
                    datum.accelZ = float.Parse(values[4]);
                    datum.gyroX = float.Parse(values[5]);
                    datum.gyroY = float.Parse(values[6]);
                    datum.gyroZ = float.Parse(values[7]);
                    data.Add(datum);
                }
                else
                {
                    Debug.LogWarning("Invalid data line: " + line);
                }
            }
        }

        StartCoroutine(ProcessData());
    }

    private IEnumerator ProcessData()
    {
        foreach (var datum in data)
        {
            float dt = datum.dt;
            Vector3 accel = new Vector3(datum.accelY, -datum.accelX, datum.accelZ);
            Vector3 gyro = new Vector3(datum.gyroY, datum.gyroX, datum.gyroZ);

            rb.MoveRotation(rb.rotation * Quaternion.Euler(gyro));
            rb.AddRelativeForce(accel, ForceMode.Acceleration);

            yield return new WaitForSeconds(dt);
        }
        Debug.Log("Data processing complete.");
        rb.linearVelocity = Vector3.zero;
        yield return null;
    }
}
