using System.Diagnostics.CodeAnalysis;
using System.Runtime.InteropServices;
using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;

[StructLayout(LayoutKind.Auto)]
public partial struct SimulatorSystem : ISystem
{
    public EntityQuery agentQuery;
    
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        agentQuery = state.GetEntityQuery(ComponentType.ReadWrite<Agent>());
    }

    [BurstCompile]
    [StructLayout(LayoutKind.Auto)]
    public partial struct MoveJob : IJobEntity
    {
        public KDTree kdTree;
        public float deltaTime;
        
        /// <summary>
        /// 遍历每一个Agent，并更新位置
        /// </summary>
        // ReSharper disable once UnusedParameter.Local
        private void Execute(Entity entity, [EntityIndexInQuery] [SuppressMessage("ReSharper", "UnusedParameter.Local")] int entityInQueryIndex,
            ref Agent agent, ref LocalTransform transform)
        {
            // 2. 计算预期速度
            agent.prefVelocity = math.normalizesafe(agent.targetPos - agent.position) * agent.maxSpeed;
            float prefDistSq = math.lengthsq(agent.targetPos - agent.position);
            if (prefDistSq < RVOMath.sqr(agent.radius * 0.5f))
            {
                agent.prefVelocity = float2.zero;
            }
            
            float2 newVelocity;
            NativeList<KDTree.AgentNeighbor> agentNeighbors = new NativeList<KDTree.AgentNeighbor>(agent.maxNeighbors, Allocator.Temp);
            NativeList<KDTree.ObstacleNeighbor> obstacleNeighbors = new NativeList<KDTree.ObstacleNeighbor>(agent.maxNeighbors, Allocator.Temp);
            agentNeighbors.Clear();
            
            // 3. 从KDTree里查找附近的圆
            ComputeNeighbors(ref agent, ref agentNeighbors, ref obstacleNeighbors);
            
            // 4. 计算新的移动速度
            ComputeNewVelocity(ref agent, in obstacleNeighbors, in agentNeighbors, out newVelocity);
            
            // 5. 根据新的移动速度，更新位置
            Update(ref agent, newVelocity);
            transform.Position = new float3(agent.position.x, 0.0f, agent.position.y);

            agentNeighbors.Dispose();
            obstacleNeighbors.Dispose();
        }

        private void Update(ref Agent agent, float2 newVelocity)
        {
            agent.velocity = newVelocity;
            agent.position += agent.velocity * deltaTime;
        }

        private void ComputeNeighbors(ref Agent agent, ref NativeList<KDTree.AgentNeighbor> agentNeighbors, ref NativeList<KDTree.ObstacleNeighbor> obstacleNeighbors)
        {
            float rangeSq = RVOMath.sqr(agent.timeHorizonObst * agent.maxSpeed + agent.radius);
            kdTree.QueryObstacleNeighborsRecursive(agent, rangeSq, 0, ref obstacleNeighbors);
            
            if (agent.maxNeighbors > 0)
            {
                rangeSq = RVOMath.sqr(agent.neighborDist);
                kdTree.QueryAgentNeighborsRecursive(agent, ref rangeSq, 0, ref agentNeighbors);
            }
        }

        private void ComputeNewVelocity(ref Agent agent, in NativeList<KDTree.ObstacleNeighbor> obstacleNeighbors, in NativeList<KDTree.AgentNeighbor> agentNeighbors, out float2 newVelocity)
        {
            NativeList<Line> orcaLines = new NativeList<Line>(Allocator.Temp);
            
            // 提前避障时间的倒数
            float invTimeHorizonObst = 1.0f / agent.timeHorizonObst;

            for (int i = 0; i < obstacleNeighbors.Length; ++i)
            {
                // 障碍物左端点O1
                Obstacle obstacle1 = obstacleNeighbors[i].obstacle;
                // 障碍物右端点O2
                Obstacle obstacle2 = kdTree.obstacles[obstacle1.nextIndex];
                
                // 相对于Agent的位置
                float2 relativePosition1 = obstacle1.point - agent.position;
                float2 relativePosition2 = obstacle2.point - agent.position;
                
                // 是否障碍物的VO已经被之前的障碍物的ORCA line处理过
                bool alreadyCovered = false;

                for (int j = 0; j < orcaLines.Length; ++j)
                {
                    // invTimeHorizonObst * relativePosition1是相对于obstacle1的相对速度。
                    // 因为line.direction是单位向量，所以distanceToLine1就是obstacle1的相对速度到line的距离。
                    float distanceToLine1 = RVOMath.det(invTimeHorizonObst * relativePosition1 - orcaLines[j].point, orcaLines[j].direction);
                    // distanceToLine2同上。
                    float distanceToLine2 = RVOMath.det(invTimeHorizonObst * relativePosition2 - orcaLines[j].point, orcaLines[j].direction);
                    // invTimeHorizonObst * agent.radius是提前避障时间内走完半径长度的速度
                    // 两者差大于0则说明完全被line覆盖
                    if (distanceToLine1 - invTimeHorizonObst * agent.radius >= -math.EPSILON &&
                        distanceToLine2 - invTimeHorizonObst * agent.radius >= -math.EPSILON)
                    {
                        alreadyCovered = true;
                        break;
                    }
                }
                
                // 已经被ORCA半平面覆盖，隐藏直接continue
                if (alreadyCovered)
                {
                    continue;
                }
                
                float distSq1 = math.lengthsq(relativePosition1);
                float distSq2 = math.lengthsq(relativePosition2);

                float radiusSq = RVOMath.sqr(agent.radius);
                
                // 静态障碍物的一条边的线段O2-O1
                float2 obstacleVector = obstacle2.point - obstacle1.point;
                // s等于：O1到Agent的向量，在O2-O1上的投影，除以O2-O1的长度
                // s也就是投影长度和线段长度的比值
                float s = math.dot(-relativePosition1, obstacleVector) / math.lengthsq(obstacleVector);
                // 注意，因为上一步s多除了一个O2-O1的长度，因此这里的s * obstacleVector
                // 其实就是Agent在O2-O1上的投影向量。
                // distSqLine就是Agent到静态障碍物边的垂直距离的平方
                float distSqLine = math.lengthsq(-relativePosition1 - s * obstacleVector);

                Line line;
                
                // s < 0.0说明Agent到O2-O1的垂足在线段外，O1的一侧；
                // distSq1 <= radiusSq，说明O1在Agent的圆内。
                if (s < 0.0f && distSq1 <= radiusSq)
                {
                    /* Agent和静态障碍物边的左端点有碰撞 */
                    if (obstacle1.convex)
                    {
                        // Agent只能是朝着不会碰撞O1的另一边运动，因此line和原点相交
                        line.point = new float2(0.0f, 0.0f);
                        // 方向垂直于Agent到O1的向量
                        line.direction = math.normalizesafe(new float2(-relativePosition1.y, relativePosition1.x));
                        orcaLines.Add(line);
                    }
                    continue;
                }
                // s > 1.0说明Agent到O2-O1的垂足在线段外，O2的一侧；
                // distSq1 <= radiusSq，说明O1在Agent的圆内。
                else if (s > 1.0f && distSq2 <= radiusSq)
                {
                    /* Agent和静态障碍物边的右端点有碰撞 */
                    // 如果Agent在相邻障碍物obstacles2的内部，否则在外部的话就交给下一个for循环去计算line
                    if (obstacle2.convex && RVOMath.det(relativePosition2, obstacle2.direction) >= 0.0f)
                    {
                        // Agent只能是朝着不会碰撞O2的另一边运动，因此line和原点相交
                        line.point = new float2(0.0f, 0.0f);
                        // 方向垂直于Agent到O2的向量
                        line.direction = math.normalizesafe(new float2(-relativePosition1.y, relativePosition1.x));
                        orcaLines.Add(line);
                    }
                }
                // s在0到1之间，说明Agent到线段O2-O1的垂足在线段内；
                // distSqLine <= radiusSq说明线段O2-O1和Agent相交
                else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
                {
                    // Agent只能朝背离线段方向移动，因此line和原点相交
                    line.point = new float2(0.0f, 0.0f);
                    // line的方向和线段O2-O1方向平行
                    line.direction = -obstacle1.direction;
                    orcaLines.Add(line);

                    continue;
                }
                
                /* Agent和线段O2-O1没有碰撞 */

                float2 leftLegDirection, rightLegDirection;
                
                // 如果Agent到O2-O1的垂足在线段外，O1的一侧；
                // 且如果Agent到O2-O1的距离小于半径；
                if (s < 0.0f && distSqLine <= radiusSq)
                {
                    if (!obstacle1.convex)
                    {
                        /* 忽略非凸多边形的顶点 */
                        continue;
                    }
                    
                    // Agent的碰撞只与O1有关
                    obstacle2 = obstacle1;
                    
                    // 这种情况下两条腿都从O1发出，且与Agent的圆外切
                    
                    // 其中一条腿的长度（两条腿的长度相等）
                    float leg1 = math.sqrt(distSq1 - radiusSq);
                    // 旋转Agent到O1的向量，得到两条腿向量的单位向量
                    leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * agent.radius,
                        relativePosition1.x * agent.radius + relativePosition1.y * leg1) / distSq1;
                    rightLegDirection = new float2(relativePosition1.x * leg1 + relativePosition1.y * agent.radius,
                        -relativePosition1.x * agent.radius + relativePosition1.y * leg1) / distSq1;
                }
                // 如果Agent到O2-O1的垂足在线段外，O2的一侧；
                // 且如果Agent到O2-O1的距离小于半径；
                else if (s > 1.0f && distSqLine <= radiusSq)
                {
                    if (!obstacle2.convex)
                    {
                        /* 忽略非凸多边形的顶点 */
                        continue;
                    }
                    
                    // Agent的碰撞只与O2有关
                    obstacle1 = obstacle2;
                    
                    // 这种情况下两条腿都从O2发出，且与Agent的圆外切
                    
                    // 其中一条腿的长度（两条腿的长度相等）
                    float leg2 = math.sqrt(distSq2 - radiusSq);
                    // 旋转Agent到O2的向量，得到两条腿向量的单位向量
                    leftLegDirection = new float2(relativePosition2.x * leg2 - relativePosition2.y * agent.radius,
                        relativePosition2.x * agent.radius + relativePosition2.y * leg2) / distSq2;
                    rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * agent.radius,
                        -relativePosition2.x * agent.radius + relativePosition2.y * leg2) / distSq2;
                }
                else
                {
                    if (obstacle1.convex)
                    {
                        float leg1 = math.sqrt(distSq1 - radiusSq);
                        // 左腿是从O1出发，和Agent的圆外切的射线方向
                        leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * agent.radius,
                            relativePosition1.x * agent.radius + relativePosition1.y * leg1) / distSq1;
                    }
                    else
                    {
                        /* Left vertex non-convex; left leg extends cut-off line. */
                        leftLegDirection = -obstacle1.direction;
                    }

                    if (obstacle2.convex)
                    {
                        float leg2 = math.sqrt(distSq2 - radiusSq);
                        // 右腿是从O2出发，和Agent的圆外切的射线方向
                        rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * agent.radius,
                            -relativePosition2.x * agent.radius + relativePosition2.y * leg2) / distSq2;
                    }
                    else
                    {
                        /* Right vertex non-convex; right leg extends cut-off line. */
                        rightLegDirection = obstacle1.direction;
                    }
                }

                Obstacle leftNeighbor = kdTree.obstacles[obstacle1.prevIndex];
                
                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (obstacle1.convex && RVOMath.det(leftLegDirection, -leftNeighbor.direction) >= 0.0f)
                {
                    /* 如果左腿在障碍物内部，则用临近边的方向当作左腿方向 */
                    leftLegDirection = -leftNeighbor.direction;
                    isLeftLegForeign = true;
                }
                if (obstacle2.convex && RVOMath.det(rightLegDirection, obstacle2.direction) <= 0.0f)
                {
                    /* 如果右腿在障碍物内部，则用临近边的方向当作右腿方向 */
                    rightLegDirection = obstacle2.direction;
                    isRightLegForeign = true;
                }
                
                // 左边小圆的圆心O1'
                float2 leftCutOff = invTimeHorizonObst * (obstacle1.point - agent.position);
                // 右边小圆的圆心O2'
                float2 rightCutOff = invTimeHorizonObst * (obstacle2.point - agent.position);
                // 连接两个圆心的向量O2'-O1'
                float2 cutOffVector = rightCutOff - leftCutOff;
                
                // 左边小圆圆心到Agent速度的向量w1
                float2 w1 = agent.velocity - leftCutOff;
                // 右边小圆圆心到Agent速度的向量w2
                float2 w2 = agent.velocity - rightCutOff;
                
                // w1到向量O2'-O1'方向上的投影长度，再比上O2'-O1'的长度；
                // t就是投影长度和O2'-O1'长度的比值。
                float t = obstacle1.id == obstacle2.id ? 0.5f : math.dot(w1, cutOffVector) / math.lengthsq(cutOffVector);
                // w1在左腿上的投影长度
                float tLeft = math.dot(w1, leftLegDirection);
                // w2在右腿上的投影长度
                float tRight = math.dot(w2, rightLegDirection);

                if ((t < 0.0f && tLeft < 0.0f) || (obstacle1.id == obstacle2.id && tLeft < 0.0f && tRight < 0.0f))
                {
                    /* 在左边小圆弧上求u */
                    float2 unitW = math.normalizesafe(w1);
                    
                    line.direction = new float2(unitW.y, -unitW.x);
                    line.point = leftCutOff + agent.radius * invTimeHorizonObst * unitW;
                    orcaLines.Add(line);
                    
                    continue;
                }
                else if (t > 1.0f && tRight < 0.0f)
                {
                    /* 在右边小圆弧上求u */
                    float2 unitW = math.normalizesafe(w2);
                    
                    line.direction = new float2(unitW.y, -unitW.x);
                    line.point = rightCutOff + agent.radius * invTimeHorizonObst * unitW;
                    orcaLines.Add(line);
                    
                    continue;
                }
                
                /* 在左腿、右腿或者O2'-O1'里面，离Agent.velocity最近的那个边上求u */
                
                float distSqCutoff = (t < 0.0f || t > 1.0f || obstacle1.id == obstacle2.id)
                    ? float.PositiveInfinity
                    : math.lengthsq(agent.velocity - (leftCutOff + t * cutOffVector));
                float distSqLeft = tLeft < 0.0f ? float.PositiveInfinity : math.lengthsq(agent.velocity - (leftCutOff + tLeft * leftLegDirection));
                float distSqRight = tRight < 0.0f ? float.PositiveInfinity : math.lengthsq(agent.velocity - (rightCutOff + tRight * rightLegDirection));
                
                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    /* 在O2'-O1'上求u */
                    line.direction = -obstacle1.direction;
                    line.point = leftCutOff + agent.radius * invTimeHorizonObst * new float2(-line.direction.y, line.direction.x);
                    orcaLines.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    /* 在左腿上求u */
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    line.direction = leftLegDirection;
                    line.point = leftCutOff + agent.radius * invTimeHorizonObst * new float2(-line.direction.y, line.direction.x);
                    orcaLines.Add(line);

                    continue;
                }

                /* 在右腿上求u */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.direction = -rightLegDirection;
                line.point = rightCutOff + agent.radius * invTimeHorizonObst * new float2(-line.direction.y, line.direction.x);
                orcaLines.Add(line);
            }
            
            int numObstLines = orcaLines.Length;
            
            // 提前避障时间的倒数
            float invTimeHorizon = 1.0f / agent.timeHorizon;

            for (int i = 0; i < agentNeighbors.Length; ++i)
            {
                Agent other = agentNeighbors[i].agent;
                
                // 相对位置：pB - pA
                float2 relativePosition = other.position - agent.position;
                // 相对速度：vA - vB = (pA - pB) / τ
                float2 relativeVelocity = agent.velocity - other.velocity;
                // 两个Agent之间的距离的平方
                float distSq = math.lengthsq(relativePosition);
                // 大圆的半径：rA + rB
                float combinedRadius = agent.radius + other.radius;
                // 两个Agent半径和的平方
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);

                Line line = new Line();
                float2 u;
                
                // 两个Agent之间的距离大于两者半径的和，说明还没有碰撞
                if (distSq > combinedRadiusSq)
                {
                    // 小圆圆心invTimeHorizon * relativePosition
                    // u为小圆圆心指向当前速度位置的向量
                    float2 w = relativeVelocity - invTimeHorizon * relativePosition;
                    float wLengthSq = math.lengthsq(w);
                    
                    // w向量在relativePosition向量上的投影
                    float dotProduct1 = math.dot(w, relativePosition);
                    
                    // dotProduct1小于0，说明w向量和relativePosition向量的夹角为钝角
                    if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        float wLength = math.sqrt(wLengthSq);
                        float2 unitW = w / wLength;
                        
                        line.direction = new float2(unitW.y, -unitW.x);
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    // 否则，在两条腿上计算u
                    else
                    {
                        // 根据勾股定理，求得腿长
                        float leg = math.sqrt(distSq - combinedRadiusSq);
                        
                        // det利用叉积（sin值）判断在哪条腿上
                        if (RVOMath.det(relativePosition, w) > 0.0f)
                        {
                            /* 向量（pB - pA）到左腿的投影向量 */
                            // 计算leg和大圆的切点，这个切点到原点的方向即为ORCA line的方向
                            line.direction = new float2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        else
                        {
                            /* 向量（pB - pA）到右腿的投影向量 */
                            // 计算leg和大圆的切点，这个切点到原点的方向的逆方向即为ORCA line的方向
                            line.direction = -new float2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        
                        // dotProduct2是相对速度点到腿上的投影长度
                        float dotProduct2 = math.dot(relativeVelocity, line.direction);
                        // 相对速度点到腿上的投影向量，减去相对速度向量，就得到了u
                        u = dotProduct2 * line.direction - relativeVelocity;
                    }
                }
                // 已经碰撞
                else
                {
                    float invTimestep = 1.0f / deltaTime;
                    // 小圆圆心到当前速度的向量
                    float2 w = relativeVelocity - invTimestep * relativePosition;
                    
                    float wLength = math.length(w);
                    float2 unitW = w / wLength;

                    line.direction = new float2(unitW.y, -unitW.x);
                    // u指向小圆圆周
                    u = (combinedRadius * invTimestep - wLength) * unitW;
                }
                
                line.point = agent.velocity + GetAgentWeight(agent, other.weight) * u;
                orcaLines.Add(line);
            }
            
            // 通过线性规划找到ORCA半平面的交集
            int lineFail = LinearProgram2(orcaLines, agent.maxSpeed, agent.prefVelocity, false, out newVelocity);
            if (lineFail < orcaLines.Length)
            {
                // 找不交集的情况下，使用3D线性规划计算
                LinearProgram3(in orcaLines, agent.weight, numObstLines, lineFail, agent.maxSpeed, ref newVelocity);
            }
            
            orcaLines.Dispose();
        }

        private float GetAgentWeight(Agent agent, float otherWeight)
        {
            return agent.weight / (agent.weight + otherWeight);
        }
        
        private bool LinearProgram1(in NativeList<Line> lines, int lineNo, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
        {
            // 设速度空间原点O到line的垂足为B，则dot(point, direction)算出的投影长度就是point到B的长度。
            float dotProduct = math.dot(lines[lineNo].point, lines[lineNo].direction);
            // 如果discriminant小于0，则说明radius小于向量OB的长度，则说明在lineNo这条线内找不到合适的Velocity
            // discriminant也就是垂足到radius交点的距离的平方
            float discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - math.lengthsq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = math.sqrt(discriminant);
            // Vnew左交点到line.point的距离
            float tLeft = -dotProduct - sqrtDiscriminant;
            // Vnew右交点到line.point的距离
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                // direction是单位向量，长度为1
                float denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                // direction是单位向量，长度为1；lines[lineNo].point - lines[i].point长度不为1
                float numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (math.length(denominator) <= math.EPSILON)
                {
                    /* lines[i]和lines[lineNo]几乎平行 */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }
                
                // 因为direction是单位向量长度为1，且根据相似三角形原理，t就是两条line的交点到lines[lineNo].point的距离
                float t = numerator / denominator;
                
                if (denominator >= 0.0f)
                {
                    /* 如果lines[i]在lines[lineNo]的右边 */
                    tRight = math.min(tRight, t);
                }
                else
                {
                    /* 如果lines[i]在lines[lineNo]的左边 */
                    tLeft = math.max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }
            
            // LinearProgram3调用进来的话，directionOpt为true
            if (directionOpt)
            {
                /* Optimize direction. */
                if (math.dot(optVelocity, lines[lineNo].direction) > 0.0f)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = math.dot(lines[lineNo].direction, (optVelocity - lines[lineNo].point));

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }

        /// <summary>
        /// 通过调用LinearProgram1线性规划，计算出Agent实际的移动速度
        /// </summary>
        /// <param name="lines">所有ORCA</param>
        /// <param name="radius">当前Agent的半径</param>
        /// <param name="optVelocity">当前Agent的期望移动速度</param>
        /// <param name="directionOpt">这个值通常是false，只有被LinearProgram3调用时候才为true</param>
        /// <param name="result">Agent实际的移动速度。如果某一条line在执行LinearProgram1的时候失败，则返回上次计算成功的速度点</param>
        /// <returns>如果计算成功，返回lines.Length；如果每一条line在执行LinearProgram1的时候失败，则返回失败的索引</returns>
        private int LinearProgram2(in NativeList<Line> lines, float radius, float2 optVelocity, bool directionOpt, out float2 result)
        {
            // 在LinearProgram3里面，如果要直接使用预期速度的话
            if (directionOpt)
            {
                // 直接赋值预期速度
                result = optVelocity * radius;
            }
            // 如果预期速度超过最大速度
            else if (math.lengthsq(optVelocity) > RVOMath.sqr(radius))
            {
                // 限制预期速度大小不能超过最大速度
                result = math.normalizesafe(optVelocity) * radius;
            }
            else
            {
                result = optVelocity;
            }

            for (int i = 0; i < lines.Length; ++i)
            {
                Line line = lines[i];
                // 当前Vopt在line的右边
                if (RVOMath.det(line.direction, line.point - result) > 0.0f)
                {
                    float2 tempResult = result;
                    // 调用LinearProgram1计算实际移动速度
                    if (!LinearProgram1(in lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }
            
            return lines.Length;
        }
        
        private void LinearProgram3(in NativeList<Line> lines, float agentWeight, int numObstLines, int beginLine, float radius, ref float2 result)
        {
            float distance = 0.0f;
            
            // 从上次失败的line开始往后遍历
            for (int i = beginLine; i < lines.Length; ++i)
            {
                // 如果result在line[i]的左侧distance远的区域以外，说明line[i]对最终的交集有影响
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* result并不能满足line[i]的约束 */
                    NativeList<Line> projLines = new NativeList<Line>(Allocator.Temp);
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }
                    
                    // 遍历line[i]之前的所有line
                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (math.length(determinant) <= math.EPSILON)
                        {
                            /* Lin[i]和line[j]平行 */
                            if (math.dot(lines[i].direction, lines[j].direction) > 0.0f)
                            {
                                /* Lin[i]和line[j]指向同一个方向 */
                                continue;
                            }
                            else
                            {
                                /* Lin[i]和line[j]指向相反的方向 */
                                line.point = agentWeight * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            // 两条line的交点
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }
                        
                        // 角平分线方向
                        line.direction = math.normalizesafe(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    float2 tempResult = result;
                    if (LinearProgram2(in projLines, radius, new float2(-lines[i].direction.y, lines[i].direction.x), true, out result) < projLines.Length)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    projLines.Dispose();

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        SystemHandle obstacleSysHandle = state.WorldUnmanaged.GetExistingUnmanagedSystem<ObstacleSystem>();
        if (!state.WorldUnmanaged.IsSystemValid(obstacleSysHandle))
        {
            return;
        }
        ObstacleCollection obstacleCollection = state.EntityManager.GetComponentData<ObstacleCollection>(obstacleSysHandle);
        
        NativeArray<Agent> agents = agentQuery.ToComponentDataArray<Agent>(Allocator.TempJob);

        KDTree kdTree = new KDTree();
        kdTree.obstacleTree = obstacleCollection.obstacleTree;
        kdTree.obstacles = obstacleCollection.obstacles;
        // 1. 构建KDTree
        kdTree.BuildAgentTree(ref agents);

        state.Dependency = new MoveJob()
        {
            kdTree = kdTree,
            deltaTime = SystemAPI.Time.DeltaTime,
        }.ScheduleParallel(state.Dependency);

        state.Dependency = agents.Dispose(state.Dependency);
        state.Dependency = kdTree.Dispose(state.Dependency);
    }

    [BurstCompile]
    public void OnDestroy(ref SystemState state)
    {

    }
}