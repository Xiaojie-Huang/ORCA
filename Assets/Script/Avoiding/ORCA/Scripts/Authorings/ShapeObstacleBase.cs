using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public abstract class ShapeObstacleBase : MonoBehaviour
{
    [SerializeField][HideInInspector]
    public List<Vector2> mCacheVertexList = new List<Vector2>();
    
#if UNITY_EDITOR
    protected List<Vector3> mEditorDebugVertexList = new List<Vector3>();
    
    private void OnDrawGizmos()
    {
        if (mCacheVertexList != null && mEditorDebugVertexList.Count > 2)
        {
            Gizmos.color = Color.red;
            for (int i = 0; i < mCacheVertexList.Count - 1; ++i)
            {
                Gizmos.DrawLine(mEditorDebugVertexList[i], mEditorDebugVertexList[i + 1]);
            }
            int vCount = mEditorDebugVertexList.Count;
            Gizmos.DrawLine(mEditorDebugVertexList[vCount - 1], mEditorDebugVertexList[0]);
        }
    }
    
    private void OnValidate()
    {
        GenerateVertex();
    }
#endif
    
    internal void GenerateVertex(bool autoAddRvoSystem = false)
    {
        mCacheVertexList.Clear();
#if UNITY_EDITOR
        mEditorDebugVertexList.Clear();
#endif

        CalculatePoints();
    }

    protected abstract void CalculatePoints();

    protected abstract class ObstacleBaker<ObstacleType> : Baker<ObstacleType> where ObstacleType : ShapeObstacleBase
    {
        protected void BakeInner(ObstacleType authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.None);

            DynamicBuffer<ObstacleVertex> obstacleVertices = AddBuffer<ObstacleVertex>(entity);
            for (int i = 0; i < authoring.mCacheVertexList.Count; ++i)
            {
                Vector2 v = authoring.mCacheVertexList[i];
                ObstacleVertex vertex = new ObstacleVertex()
                {
                    position = new float2(v.x, v.y)
                };
                obstacleVertices.Add(vertex);
            }
        }
    }
}