using UnityEngine;

public class FollowOther : MonoBehaviour
{
    [SerializeField] private Transform target; // The target to follow
    [SerializeField] private Vector3 offset; // Optional offset to apply to the target's position
    private Quaternion rotation; // The rotation to apply to the follower

    private void Awake()
    {
        // Ensure the target is assigned
        if (target == null)
        {
            Debug.LogError("Target not assigned in the Follower GameObject.");
        }
        rotation = Quaternion.LookRotation(-offset, Vector3.up);
    }

    private void Update()
    {
        transform.position = target.position + offset;
        transform.rotation = rotation;
    }
}
