using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class Profiler : MonoBehaviour
{
    private static Profiler instance = null;
    
    public int collisionCount = 0;
    
    [Header("是否启用避障")]
    public bool avoiding = true;

    public TextMeshProUGUI text;

    public static Profiler Instance
    {
        get
        {
            if (instance == null)
            {
                instance = FindObjectOfType<Profiler>();
            }
            return instance;
        }
    }
    
    private void Update()
    {
        text.text = "Collision Count: " + collisionCount.ToString();    
    }
}
