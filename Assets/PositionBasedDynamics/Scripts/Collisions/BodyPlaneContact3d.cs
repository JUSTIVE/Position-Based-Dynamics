using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;
using UnityEngine;
namespace PositionBasedDynamics.Collisions
{

    internal class BodyPlaneContact3d : CollisionContact3d
    {

        private Body3d Body0;

        private int i0;

        private Vector3 Normal;

        private double Distance;

        internal BodyPlaneContact3d(Body3d body0, int i0, Vector3 normal, double dist)
        {
            Body0 = body0;
            this.i0 = i0;

            Normal = normal;
            Distance = dist;
        }

        internal override void ResolveContact(double di)
        {
            double d = Vector3.Dot(Normal, Body0.Predicted[i0]) + Distance - Body0.ParticleRadius;

            if (d < 0.0)
            {
                Vector3 delta = Normal * (float)-d * (float)di;
                Body0.Positions[i0] += delta;
                Body0.Predicted[i0] += delta;
            }
        }

    }

}