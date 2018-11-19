using System;
using System.Collections.Generic;

using Common.Unity.Mathematics;
using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;
using UnityEngine;
namespace PositionBasedDynamics.Sources
{
    public class ParticlesFromBounds : ParticleSource
    {

        public Box3d Bounds { get; private set; }

        public ParticlesFromBounds(double spacing, Box3d bounds) : base(spacing)
        {
            Bounds = bounds;
            CreateParticles();
        }

        public ParticlesFromBounds(double spacing, Box3d bounds, Box3d exclusion) : base(spacing)
        {
            Bounds = bounds;
            CreateParticles(exclusion);
        }

        private void CreateParticles()
        {

            int numX = (int)(Bounds.Width / Diameter);
            int numY = (int)(Bounds.Height / Diameter);
            int numZ = (int)(Bounds.Depth / Diameter);

            Positions = new List<Vector3d>(numX * numY * numZ);

            for (int z = 0; z < numZ; z++)
            {
                for (int y = 0; y < numY; y++)
                {
                    for (int x = 0; x < numX; x++)
                    {
                        Vector3d pos = new Vector3d();
                        pos.x = Diameter * x + Bounds.Min.x + Spacing;
                        pos.y = Diameter * y + Bounds.Min.y + Spacing;
                        pos.z = Diameter * z + Bounds.Min.z + Spacing;

                        Positions.Add(pos);
                    }
                }
            }

        }

        private void CreateParticles(Box3d exclusion)
        {

            int numX = (int)(Bounds.Width / Diameter);
            int numY = (int)(Bounds.Height / Diameter);
            int numZ = (int)(Bounds.Depth / Diameter);

            Positions = new List<Vector3d>();

            for (int z = 0; z < numZ; z++)
            {
                for (int y = 0; y < numY; y++)
                {
                    for (int x = 0; x < numX; x++)
                    {
                        Vector3 pos = new Vector3();
                        pos.x = (float)Diameter * x + Bounds.Min.x + (float)Spacing;
                        pos.y = (float)Diameter * y + Bounds.Min.y + (float)Spacing;
                        pos.z = (float)Diameter * z + Bounds.Min.z + (float)Spacing;

                        if(!exclusion.Contains(pos))
                            Positions.Add(MathConverter.ToVector3d(pos));

                    }
                }
            }

        }



    }

}