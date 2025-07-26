using System;
using System.Collections;
using Unity.Transforms;
using UnityEngine;

// ReSharper disable once CheckNamespace
public class DebugGameObject : MonoBehaviour
{
    public int id;
    [NonSerialized]
    public Vector2 velocity;
    [NonSerialized]
    public Vector2 prefVelocity;
    [NonSerialized]
    public LocalTransform localTransform = new LocalTransform()
    {
        Position = Vector3.zero,
        Rotation = Quaternion.identity,
        Scale = 1.0f
    };
    [NonSerialized]
    public TMPro.TMP_Text text;

    private void Awake()
    {
        text = GetComponentInChildren<TMPro.TMP_Text>();
        text.text = id.ToString();
    }

    private void Update()
    {
        transform.position = localTransform.Position;
        transform.rotation = localTransform.Rotation;
    }

    private void OnDrawGizmos()
    {
        Color c = Gizmos.color;

        Vector3 pos = transform.position;
        
        Gizmos.color = Color.red;
        Gizmos.DrawLine(pos, pos + new Vector3(velocity.x, 0.5f, velocity.y));
        
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(pos, pos + new Vector3(prefVelocity.x, 0.5f, prefVelocity.y));
    }
}