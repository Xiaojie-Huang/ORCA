using System;
using UnityEngine;

// ReSharper disable once CheckNamespace
public class FPSMgr : MonoBehaviour
{
    public TMPro.TMP_Text text;

    private float ramainTime = 1.0f;
    private int frameCount = 0;

    private void Awake()
    {
        Application.targetFrameRate = 60;
    }

    private void Update()
    {
        ramainTime -= Time.deltaTime;
        if (ramainTime <= 0.0f)
        {
            ramainTime = 1.0f;
            text.text = $"{frameCount} FPS";
            frameCount = 0;
        }
        else
        {
            ++frameCount;
        }
    }
}