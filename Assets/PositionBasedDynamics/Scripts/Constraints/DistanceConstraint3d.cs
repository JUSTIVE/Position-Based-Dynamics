using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;
using UnityEngine;

namespace PositionBasedDynamics.Constraints
{

    public class DistanceConstraint3d : Constraint3d
    {

        private double RestLength;

        private double CompressionStiffness;

        private double StretchStiffness;

        private readonly int i0, i1;

        internal DistanceConstraint3d(Body3d body, int i0, int i1, double stiffness) : base(body)
        {
            this.i0 = i0;
            this.i1 = i1;

            CompressionStiffness = stiffness;
            StretchStiffness = stiffness;
            RestLength = (Body.Positions[i0] - Body.Positions[i1]).magnitude;
        }

        internal override void ConstrainPositions(double di)
        {
            double mass = Body.ParticleMass;
            double invMass = 1.0 / mass;
            double sum = mass * 2.0;

            Vector3 n = Body.Predicted[i1] - Body.Predicted[i0];
            double d = n.magnitude;
            n.Normalize();

            Vector3 corr;
            if (d < RestLength)
                corr = (float)CompressionStiffness * n * (float)(d - RestLength) * (float)sum;
            else
                corr = (float)StretchStiffness * n * (float)(d - RestLength) * (float)sum;

            Body.Predicted[i0] += (float)invMass * corr * (float)di;

            Body.Predicted[i1] -= (float)invMass * corr * (float)di;

        }

    }

}
