using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;
using UnityEngine;

namespace PositionBasedDynamics.Constraints
{

    public class BendingConstraint3d : Constraint3d
    {

        private double RestLength { get; set; }

        private double Stiffness { get; set; }

        private readonly int i0, i1, i2;

        internal BendingConstraint3d(Body3d body, int i0, int i1, int i2, double stiffness) : base(body)
        {
            this.i0 = i0;
            this.i1 = i1;
            this.i2 = i2;

            Stiffness = stiffness;

            Vector3 center = (Body.Positions[i0] + Body.Positions[i1] + Body.Positions[i2]) / 3.0f;
            RestLength = (Body.Positions[i2] - center).magnitude;
        }

        internal override void ConstrainPositions(double di)
        {

            Vector3 center = (Body.Predicted[i0] + Body.Predicted[i1] + Body.Predicted[i2]) / 3.0f;

            Vector3 dirCenter = Body.Predicted[i2] - center;

            double distCenter = dirCenter.magnitude;
            double diff = 1.0 - (RestLength / distCenter);
            double mass = Body.ParticleMass;

            double w = mass + mass * 2.0f + mass;

            Vector3 dirForce = dirCenter * (float)diff;

            Vector3 fa = (float)Stiffness * (float)(2.0f * mass / w) * dirForce * (float)di;
            Body.Predicted[i0] += fa;

            Vector3 fb = (float)Stiffness * (float)(2.0f * mass / w) * dirForce * (float)di;
            Body.Predicted[i1] += fb;

            Vector3 fc = (float)-Stiffness * (float)(4.0f * mass / w) * dirForce * (float)di;
            Body.Predicted[i2] += fc;

        }

    }

}
