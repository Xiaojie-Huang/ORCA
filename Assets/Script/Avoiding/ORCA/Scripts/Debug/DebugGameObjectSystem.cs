using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using Object = UnityEngine.Object;

// ReSharper disable once CheckNamespace
public partial class DebugGameObjectSystem : SystemBase
{
    protected override void OnUpdate()
    {
        if (DebugGameObjectMgr.Instance == null)
        {
            return;
        }
        if (!DebugGameObjectMgr.Instance.enable)
        {
            return;
        }
        foreach (var (agent, transform) in SystemAPI.Query<RefRO<Agent>, RefRO<LocalTransform>>())
        {
            if (!DebugGameObjectMgr.Instance.debugGameObjectMap.TryGetValue(agent.ValueRO.id, out DebugGameObject debugGameObject))
            {
                GameObject inst = Object.Instantiate(DebugGameObjectMgr.Instance.templateObj.gameObject, DebugGameObjectMgr.Instance.transform, false);
                inst.SetActive(true);
                debugGameObject = inst.GetComponent<DebugGameObject>();
                debugGameObject.id = agent.ValueRO.id;
                debugGameObject.text.text = agent.ValueRO.id.ToString();
                DebugGameObjectMgr.Instance.debugGameObjectMap.Add(debugGameObject.id, debugGameObject);
            }
            debugGameObject.velocity = agent.ValueRO.velocity;
            debugGameObject.prefVelocity = agent.ValueRO.prefVelocity;
            debugGameObject.localTransform = transform.ValueRO;
        }
    }
}