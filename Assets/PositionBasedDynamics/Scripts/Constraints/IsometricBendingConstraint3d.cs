using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;
using UnityEngine;
namespace PositionBasedDynamics.Constraints
{

    public class IsometricBendingConstraint3d : Constraint3d
    {

        private readonly int i0, i1, i2, i3;

        private double BendStiffness;

        private Matrix4x4d Q;

        internal IsometricBendingConstraint3d(Body3d body, int i0, int i1, int i2, int i3, float stiffness) : base(body)
        {

            this.i0 = i0;
            this.i1 = i1;
            this.i2 = i2;
            this.i3 = i3;
            BendStiffness = stiffness;

            Vector3 p0 = body.Positions[i0];
            Vector3 p1 = body.Positions[i1];
            Vector3 p2 = body.Positions[i2];
            Vector3 p3 = body.Positions[i3];

            // Compute matrix Q for quadratic bending
	        Vector3[] x = new Vector3[]{ p2, p3, p0, p1 };

	        Vector3 e0 = x[1] - x[0];
	        Vector3 e1 = x[2] - x[0];
	        Vector3 e2 = x[3] - x[0];
	        Vector3 e3 = x[2] - x[1];
	        Vector3 e4 = x[3] - x[1];

            double c01 = CotTheta(e0, e1);
            double c02 = CotTheta(e0, e2);
            double c03 = CotTheta(e0 * -1.0f, e3);
            double c04 = CotTheta(e0 * -1.0f, e4);

            double A0 = 0.5f * Vector3.Cross(e0,e1).magnitude;
            double A1 = 0.5f * Vector3.Cross(e0, e2).magnitude;

            double coef = -3.0 / (2.0 * (A0 + A1));
            double[] K = new double[] { c03 + c04, c01 + c02, -c01 - c03, -c02 - c04 };
            double[] K2 = new double[] { coef * K[0], coef * K[1], coef * K[2], coef * K[3] };

            Q = new Matrix4x4d();

            for (int j = 0; j < 4; j++)
	        {
		        for(int k = 0; k < j; k++)
		        {
			        Q[j, k] = Q[k, j] = K[j] * K2[k];
		        }

		        Q[j, j] = K[j] * K2[j];
	        }

        }

        private double CotTheta(Vector3 v, Vector3 w)
        {
            double cosTheta = Vector3.Dot(v, w);
            double sinTheta = Vector3.Cross(v, w).magnitude;
	        return cosTheta / sinTheta;
        }

        internal override void ConstrainPositions(double di)
        {

            Vector3 p0 = Body.Predicted[i0];
            Vector3 p1 = Body.Predicted[i1];
            Vector3 p2 = Body.Predicted[i2];
            Vector3 p3 = Body.Predicted[i3];

            double invMass = 1.0 / Body.ParticleMass;

            Vector3[] x = new Vector3[]{ p2, p3, p0, p1 };
	
	        double energy = 0.0;
	        for (int k = 0; k < 4; k++)
		        for (int j = 0; j < 4; j++)
			        energy += Q[j, k] * Vector3.Dot(x[k], x[j]);

	        energy *= 0.5;

	        Vector3[] gradC = new Vector3[4];

	        for (int k = 0; k < 4; k++)
		        for (int j = 0; j < 4; j++)
			        gradC[j] += (float)Q[j,k] * x[k];

	        double sum_normGradC = 0.0;
	        for (int j = 0; j < 4; j++)
	        {
		        // compute sum of squared gradient norms
			    sum_normGradC += invMass * gradC[j].sqrMagnitude;
	        }

	        // exit early if required
	        if (Math.Abs(sum_normGradC) > 1e-9)
	        {
		        // compute impulse-based scaling factor
		        double s = energy / sum_normGradC;

                Body.Predicted[i0] += (float)(-BendStiffness * (s * invMass)) * gradC[2] * (float)di;
                Body.Predicted[i1] += (float)(-BendStiffness * (s * invMass)) * gradC[3] * (float)di;
                Body.Predicted[i2] += (float)(-BendStiffness * (s * invMass)) * gradC[0] * (float)di;
                Body.Predicted[i3] += (float)(-BendStiffness * (s * invMass)) * gradC[1] * (float)di;
	        }

        }

    }

}
