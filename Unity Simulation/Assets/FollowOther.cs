using UnityEngine;

public class FollowOther : MonoBehaviour
{
    [SerializeField] private Transform target; // The target to follow

    private void Awake()
    {
        // Ensure the target is assigned
        if (target == null)
        {
            Debug.LogError("Target not assigned in the Follower GameObject.");
        }
    }

    private void Update()
    {
        Vector3 relativePos = target.position - transform.position;
        Quaternion rotation = Quaternion.LookRotation(relativePos, Vector3.up);
        transform.rotation = rotation;
    }
}
