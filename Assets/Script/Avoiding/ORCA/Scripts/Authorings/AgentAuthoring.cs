using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

public class AgentAuthoring : MonoBehaviour
{
    [Header("避障半径")]
    public float radius;
    [Header("避障责任权重")]
    public float weight;
    [Header("KDTree查找的最大邻接Agent数量")]
    public int maxNeighbors;
    [Header("KDTree查找邻接Agent时候的最大检测距离")]
    public float neighborDist;
    [Header("提前避障的时间（Agent之间）")]
    public float timeHorizon;
    [Header("提前避障的时间（静态障碍物）")]
    public float timeHorizonObst;
    [Header("预期移动速率")]
    public float speed;
        
    private class AgentAuthoringBaker : Baker<AgentAuthoring>
    {
        private static int agentID = 1;
        
        public override void Bake(AgentAuthoring authoring)
        {
            Transform t = authoring.transform;
            float3 position = t.position;
                
            Entity entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new LocalTransform()
            {
                Position = position,
                Rotation = t.rotation,
                Scale = 1.0f
            });
            float2 targetPos = -position.xz;
            float2 v = math.normalizesafe(targetPos - position.xz) * authoring.speed;
            AddComponent(entity, new Agent()
            {
                id = ++agentID,
                
                position = position.xz,
                radius = authoring.radius,
                weight = authoring.weight,
                targetPos = targetPos,
                maxSpeed = authoring.speed,
                
                maxNeighbors = authoring.maxNeighbors,
                neighborDist = authoring.neighborDist,
                
                timeHorizon = authoring.timeHorizon,
                timeHorizonObst = authoring.timeHorizonObst,
                
                velocity = v,
                prefVelocity = v
            });
        }
    }
}