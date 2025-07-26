using System.Runtime.CompilerServices;
using Unity.Mathematics;

public static class RVOMath
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float sqr(float scalar)
    {
        return scalar * scalar;
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float det(float2 vector1, float2 vector2)
    {
        return vector1.x * vector2.y - vector1.y * vector2.x;
    }
    
    /// <summary>
    /// c是否在线段ab的左边
    /// </summary>
    /// <param name="a">线段端点a</param>
    /// <param name="b">线段端点b</param>
    /// <param name="c">点c</param>
    /// <returns>返回值大于0，则c在线段ab的左边</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float leftOf(float2 a, float2 b, float2 c)
    {
        return det(a - c, b - a);
    }
    
    internal static float distSqPointLineSegment(float2 vector1, float2 vector2, float2 vector3)
    {
        float value = math.lengthsq(vector2 - vector1);
        float r = value > 1E-05f ? math.dot((vector3 - vector1), (vector2 - vector1)) / value : 0f;

        if (r < 0.0f)
        {
            return math.lengthsq(vector3 - vector1);
        }

        if (r > 1.0f)
        {
            return math.lengthsq(vector3 - vector2);
        }

        return math.lengthsq(vector3 - (vector1 + r * (vector2 - vector1)));
    }
}