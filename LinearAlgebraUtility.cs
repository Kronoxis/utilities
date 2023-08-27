using UnityEngine;
using System.Collections;
using System.IO;

namespace Dreamteck
{
    public static class LinearAlgebraUtility
    {
        public enum Axis
        {
            X = 0,
            Y = 1,
            Z = 2
        }

        public static Vector3 ProjectOnLine(Vector3 fromPoint, Vector3 toPoint, Vector3 project)
        {
            Vector3 projectedPoint = Vector3.Project((project - fromPoint), (toPoint - fromPoint)) + fromPoint;
            Vector3 dir = toPoint - fromPoint;
            Vector3 projectedDir = projectedPoint - fromPoint;
            float dot = Vector3.Dot(projectedDir, dir);
            if(dot > 0f)
            {
                if(projectedDir.sqrMagnitude <= dir.sqrMagnitude) return projectedPoint;
                else return toPoint;
            } else return fromPoint;
        }

        public static float InverseLerp(Vector3 a, Vector3 b, Vector3 value)
        {
            Vector3 ab = b - a;
            Vector3 av = value - a;
            return Vector3.Dot(av, ab) / Vector3.Dot(ab, ab);
        }

        public static float DistanceOnSphere(Vector3 from, Vector3 to, float radius)
        {
            float distance = 0;
            
            if (from == to)
            {
                distance = 0;
            }
            else if (from == -to)
            {
                distance = Mathf.PI * radius;
            }
            else
            {
                distance = Mathf.Sqrt(2) * radius * Mathf.Sqrt(1.0f - Vector3.Dot(from, to));
            }

            return distance;
        }

        public static Vector3 FlattenVector(Vector3 input, Axis axis, float flatValue = 0f)
        {
            switch (axis)
            {
                case Axis.X: input.x = flatValue; break;
                case Axis.Y: input.y = flatValue; break;
                case Axis.Z: input.z = flatValue; break;
            }
            return input;
        }


        public struct Intersection
        {
            /// <summary>
            /// Intersection point from segment A
            /// </summary>
            public Vector3 pointA;
            /// <summary>
            /// Intersection percent from segment A
            /// </summary>
            public double percentA;
            /// <summary>
            /// Intersection point from segment B
            /// </summary>
            public Vector3 pointB;
            /// <summary>
            /// Intersection percent from segment B
            /// </summary>
            public double percentB;
        }

        /// <summary>
        /// Performs a line-line intersection within a given distance tolerance
        /// </summary>
        /// <param name="startA">Start of line A</param>
        /// <param name="endA">End of line A</param>
        /// <param name="startB">Start of line B</param>
        /// <param name="endB">End of line B</param>
        /// <param name="intersection">The intersection in world space</param>
        /// <param name="tolerance">The maximum distance between the two lines to count as intersecting</param>
        /// <returns>Whether an intersection was found</returns>
        // Based on https://stackoverflow.com/questions/10551555/need-an-algorithm-for-3d-vectors-intersection
        public static bool LineLineIntersect(Vector3 startA, Vector3 endA, Vector3 startB, Vector3 endB, out Intersection intersection, float tolerance = 0.001f)
        {
            intersection = default;

            // The algorithm works for lines, not segments, so we need unit directions
            // We can optimize this call by using the length to create the unit direction,
            // since vector.normalized is the same as dividing the vector by its length,
            // thus getting the length first and then dividing the vector by it, we avoid
            // doing two length calculations.
            // Also validate that each segment has a length greater than 0!
            Vector3 dirA = endA - startA;
            float lengthA = dirA.magnitude;
            if (Mathf.Approximately(lengthA, 0f)) return false;
            dirA /= lengthA;
            Vector3 dirB = endB - startB;
            float lengthB = dirB.magnitude;
            if (Mathf.Approximately(lengthB, 0f)) return false;
            dirB /= lengthB;


            // Find the distance projection
            // If the projection is 1, the lines are parallel and intersect
            // either 0 or infinitely many times.
            float u = Vector3.Dot(dirA, dirB);
            if (Mathf.Abs(u) > 0.999f) return false;

            // Find the separation projections
            Vector3 diff = startB - startA;
            float t1 = Vector3.Dot(diff, dirA);
            float t2 = Vector3.Dot(diff, dirB);

            // Compute the distance to intersection for both lines
            float u2 = u * u;
            float d1 = (t1 - u * t2) / (1 - u2);
            float d2 = (t2 - u * t1) / (u2 - 1);

            // Find the points on both lines that are closest to each other
            Vector3 p1 = startA + dirA * d1;
            Vector3 p2 = startB + dirB * d2;

            // Assign the intersection data
            intersection.pointA = p1;
            intersection.percentA = d1 / lengthA;
            intersection.pointB = p2;
            intersection.percentB = d2 / lengthB;

            // If the percentages are negative or beyond the segment length, the
            // intersection occurs on part of the line that goes beyond the desired
            // segment, thus meaning there is no intersection on these segments.
            if (d1 <= 0.0001f || d1 >= lengthA - 0.0001f) return false;
            if (d2 <= 0.0001f || d2 >= lengthB - 0.0001f) return false;

            // If the distance between the two points is greater than the allowed tolerance,
            // there is also no intersection.
            if ((p2 - p1).sqrMagnitude > tolerance * tolerance) return false;

            // All conditions are met, intersection found.
            return true;
        }
    }
}