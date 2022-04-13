using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SPHCollision : MonoBehaviour
{
    [HideInInspector]
    public Vector3 position;
    [HideInInspector]
    public Vector3 right;
    [HideInInspector]
    public Vector3 up;
    [HideInInspector]
    public Vector2 scale;
    [HideInInspector]
    public Vector3 penetrationNormal;

    private void Awake()
    {
        position = transform.position;
        right = transform.right;
        up = transform.up;
        scale = new Vector2(transform.lossyScale.x / 2f, transform.lossyScale.y / 2f); // scale should be smaller than its real scale
        penetrationNormal = Vector3.Cross(right, up);
    }
}
