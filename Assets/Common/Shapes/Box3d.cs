using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using Common.Mathematics.LinearAlgebra;

namespace Common.Geometry.Shapes
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Box3d
    {

        public Vector3 Center { get { return (Min + Max) / 2.0f; } }

        public Vector3d Size { get { return new Vector3d(Width, Height, Depth); } }

        public double Width { get { return Max.x - Min.x; } }

        public double Height { get { return Max.y - Min.y; } }

        public double Depth { get { return Max.z - Min.z; } }

        public double Area
        {
            get
            {
                return (Max.x - Min.x) * (Max.y - Min.y) * (Max.z - Min.z);
            }
        }

        public double SurfaceArea
        {
            get
            {
                Vector3 d = Max - Min;
                return 2.0 * (d.x * d.y + d.x * d.z + d.y * d.z);
            }
        }

        public Vector3 Min { get; set; }

        public Vector3 Max { get; set; }

        public Box3d(double min, double max)
        {
            Min = new Vector3((float)min, (float)min, (float)min);
            Max = new Vector3((float)max, (float)max, (float)max);
        }

        public Box3d(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
        {
            Min = new Vector3((float)minX, (float)minY, (float)minZ);
            Max = new Vector3((float)maxX, (float)maxY, (float)maxZ);
        }

        public Box3d(Vector3 min, Vector3 max)
        {
            Min = min;
            Max = max;
        }

        public Box3d(Vector3i min, Vector3i max)
        {
            Min = new Vector3(min.x, min.y, min.z);
            Max = new Vector3(max.x, max.y, max.z); ;
        }

        public Box3d(Box3d box)
        {
            Min = box.Min;
            Max = box.Max;
        }

        public void GetCorners(IList<Vector3d> corners)
        {
            corners[0] = new Vector3d(Min.x, Min.y, Min.z);
            corners[1] = new Vector3d(Min.x, Min.y, Max.z);
            corners[2] = new Vector3d(Max.x, Min.y, Max.z);
            corners[3] = new Vector3d(Max.x, Min.y, Min.z);

            corners[4] = new Vector3d(Min.x, Max.y, Min.z);
            corners[5] = new Vector3d(Min.x, Max.y, Max.z);
            corners[6] = new Vector3d(Max.x, Max.y, Max.z);
            corners[7] = new Vector3d(Max.x, Max.y, Min.z);
        }

        public void GetCorners(IList<Vector4d> corners)
        {
            corners[0] = new Vector4d(Min.x, Min.y, Min.z, 1);
            corners[1] = new Vector4d(Min.x, Min.y, Max.z, 1);
            corners[2] = new Vector4d(Max.x, Min.y, Max.z, 1);
            corners[3] = new Vector4d(Max.x, Min.y, Min.z, 1);

            corners[4] = new Vector4d(Min.x, Max.y, Min.z, 1);
            corners[5] = new Vector4d(Min.x, Max.y, Max.z, 1);
            corners[6] = new Vector4d(Max.x, Max.y, Max.z, 1);
            corners[7] = new Vector4d(Max.x, Max.y, Min.z, 1);
        }

        /// <summary>
        /// Returns the bounding box containing this box and the given point.
        /// </summary>
        public Box3d Enlarge(Vector3d p)
        {
            return new Box3d(Math.Min(Min.x, p.x), Math.Max(Max.x, p.x),
                             Math.Min(Min.y, p.y), Math.Max(Max.y, p.y),
                             Math.Min(Min.z, p.z), Math.Max(Max.z, p.z));
        }

        /// <summary>
        /// Returns the bounding box containing this box and the given box.
        /// </summary>
        public Box3d Enlarge(Box3d r)
        {
            return new Box3d(Math.Min(Min.x, r.Min.x), Math.Max(Max.x, r.Max.x),
                             Math.Min(Min.y, r.Min.y), Math.Max(Max.y, r.Max.y),
                             Math.Min(Min.z, r.Min.z), Math.Max(Max.z, r.Max.z));
        }

        /// <summary>
        /// Returns true if this bounding box contains the given bounding box.
        /// </summary>
        public bool Intersects(Box3d a)
        {
            if (Max.x < a.Min.x || Min.x > a.Max.x) return false;
            if (Max.y < a.Min.y || Min.y > a.Max.y) return false;
            if (Max.z < a.Min.z || Min.z > a.Max.z) return false;
            return true;
        }

        /// <summary>
        /// Returns true if this bounding box contains the given point.
        /// </summary>
        public bool Contains(Vector3 p)
        {
            if (p.x > Max.x || p.x < Min.x) return false;
            if (p.y > Max.y || p.y < Min.y) return false;
            if (p.z > Max.z || p.z < Min.z) return false;
            return true;
        }

        /// <summary>
        /// Returns the closest point to a on the box.
        /// </summary>
        public Vector3d Closest(Vector3d p)
        {
            Vector3d c;

            if (p.x < Min.x)
                c.x = Min.x;
            else if (p.x > Max.x)
                c.x = Max.x;
            else if (p.x - Min.x < Width * 0.5)
                c.x = Min.x;
            else
                c.x = Max.x;

            if (p.y < Min.y)
                c.y = Min.y;
            else if (p.y > Max.y)
                c.y = Max.y;
            else if (p.y - Min.y < Height * 0.5)
                c.y = Min.y;
            else
                c.y = Max.y;

            if (p.z < Min.z)
                c.z = Min.z;
            else if (p.z > Max.z)
                c.z = Max.z;
            else if (p.z - Min.z < Depth * 0.5)
                c.z = Min.z;
            else
                c.z = Max.z;

            return c;
        }

        public bool Intersects(Vector3d p1, Vector3d p2)
        {
            Vector3 p1R = new Vector3((float)p1.x, (float)p1.y, (float)p1.z);
            Vector3 p2R = new Vector3((float)p2.x, (float)p2.y, (float)p2.z);

            Vector3 d = (p2R - p1R) * 0.5f;
            Vector3 e = (Max - Min) * 0.5f;
            Vector3 c = p1R + d - (Min + Max) * 0.5f;
            Vector3 ad = new Vector3(Math.Abs(d.x), Math.Abs(d.y), Math.Abs(d.z)); 

            if (Math.Abs(c.x) > e.x + ad.x) return false;
            if (Math.Abs(c.y) > e.y + ad.y) return false;
            if (Math.Abs(c.z) > e.z + ad.z) return false;

            double eps = 1e-12;

            if (Math.Abs(d.y * c.z - d.z * c.y) > e.y * ad.z + e.z * ad.y + eps) return false;
            if (Math.Abs(d.z * c.x - d.x * c.z) > e.z * ad.x + e.x * ad.z + eps) return false;
            if (Math.Abs(d.x * c.y - d.y * c.x) > e.x * ad.y + e.y * ad.x + eps) return false;

            return true;
        }


    }

}




















