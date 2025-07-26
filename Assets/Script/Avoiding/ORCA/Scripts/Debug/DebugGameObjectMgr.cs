using UnityEngine;
using System.Collections.Generic;
using System;
using UnityEngine.Serialization;

// ReSharper disable once CheckNamespace
public class DebugGameObjectMgr : MonoBehaviour
{
    public DebugGameObject templateObj;

    public bool enable = true;
    
    public Dictionary<int, DebugGameObject> debugGameObjectMap = new Dictionary<int, DebugGameObject>();

    private static DebugGameObjectMgr instance = null;
    public static DebugGameObjectMgr Instance => instance;

    private void Awake()
    {
        instance = this;

        if (new FloatPair(5, 4) > new FloatPair(4, 5))
        {
            Debug.LogError("YES");
        }
        else
        {
            Debug.LogError("NO");
        }
    }

    private void OnDestroy()
    {
        instance = null;
    }
}