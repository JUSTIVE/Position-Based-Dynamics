using System;
using System.Collections.Generic;
using Common.Unity.Mathematics;
using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;
using UnityEngine;
namespace PositionBasedDynamics.Constraints
{

    public class StrainTetConstraint3d : Constraint3d
    {

        public bool NormalizeStretch { get; set; }

        public bool NormalizeShear { get; set; }

        private readonly int P0, P1, P2, P3;

        private Matrix3x3d InvRestMatrix;

        private double Stiffness;

        private Vector3[] Correction;

        internal StrainTetConstraint3d(Body3d body, int p0, int p1, int p2, int p3, double stiffness) : base(body)
        {

            P0 = p0;
            P1 = p1;
            P2 = p2;
            P3 = p3;

            Stiffness = stiffness;
            NormalizeStretch = false;
            NormalizeShear = false;

            Correction = new Vector3[4];

            Vector3 x0 = body.Positions[P0];
            Vector3 x1 = body.Positions[P1];
            Vector3 x2 = body.Positions[P2];
            Vector3 x3 = body.Positions[P3];

            Matrix3x3d restMatrix = new Matrix3x3d();

            restMatrix.SetColumn(0, MathConverter.ToVector3d(x1 - x0));
            restMatrix.SetColumn(1, MathConverter.ToVector3d(x2 - x0));
            restMatrix.SetColumn(2, MathConverter.ToVector3d(x3 - x0));

            InvRestMatrix = restMatrix.Inverse;

        }

        internal override void ConstrainPositions(double di)
        {

            Vector3 x0 = Body.Predicted[P0];
            Vector3 x1 = Body.Predicted[P1];
            Vector3 x2 = Body.Predicted[P2];
            Vector3 x3 = Body.Predicted[P3];

            double invMass = 1.0 / Body.ParticleMass;

            bool res = SolveConstraint(x0, x1, x2, x3, invMass);

            if (res)
            {
                Body.Predicted[P0] += Correction[0] * (float)di;
                Body.Predicted[P1] += Correction[1] * (float)di;
                Body.Predicted[P2] += Correction[2] * (float)di;
                Body.Predicted[P3] += Correction[3] * (float)di;
            }

        }

        private bool SolveConstraint(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, double invMass)
        {

            double eps = 1e-9;

            Correction[0] = Vector3.zero;
            Correction[1] = Vector3.zero;
            Correction[2] = Vector3.zero;
            Correction[3] = Vector3.zero;

            Vector3[] c = new Vector3[3];
            c[0] = MathConverter.ToVector3(InvRestMatrix.GetColumn(0));
            c[1] = MathConverter.ToVector3(InvRestMatrix.GetColumn(1));
            c[2] = MathConverter.ToVector3(InvRestMatrix.GetColumn(2));

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    Matrix3x3d P = new Matrix3x3d();

                    // Jacobi
                    //P.col(0) = p1 - p0;		
                    //P.col(1) = p2 - p0;
                    //P.col(2) = p3 - p0;

                    // Gauss - Seidel
                    P.SetColumn(0, MathConverter.ToVector3d((p1 + Correction[1]) - (p0 + Correction[0]))); 
                    P.SetColumn(1, MathConverter.ToVector3d((p2 + Correction[2]) - (p0 + Correction[0])));
                    P.SetColumn(2, MathConverter.ToVector3d((p3 + Correction[3]) - (p0 + Correction[0])));

                    Vector3 fi = MathConverter.ToVector3(P * MathConverter.ToVector3d(c[i]));
                    Vector3 fj = MathConverter.ToVector3(P * MathConverter.ToVector3d(c[j]));

                    double Sij = Vector3.Dot(fi, fj);

                    double wi = 0.0, wj = 0.0, s1 = 0.0, s3 = 0.0;
                    if (NormalizeShear && i != j)
                    {
                        wi = fi.magnitude;
                        wj = fj.magnitude;
                        s1 = 1.0f / (wi * wj);
                        s3 = s1 * s1 * s1;
                    }

                    Vector3[] d = new Vector3[4];
                    d[0] = Vector3.zero;

                    for (int k = 0; k < 3; k++)
                    {
                        d[k + 1] = fj * (float)InvRestMatrix[k, i] + fi * (float)InvRestMatrix[k, j];

                        if (NormalizeShear && i != j)
                        {
                            d[k + 1] = (float)s1 * d[k + 1] - (float)Sij * (float)s3 * ((float)(wj * wj) * fi * (float)InvRestMatrix[k, i] + (float)(wi * wi) * fj * (float)InvRestMatrix[k, j]);
                        }

                        d[0] -= d[k + 1];
                    }

                    if (NormalizeShear && i != j)
                        Sij *= s1;

                    double lambda = (d[0].sqrMagnitude + d[1].sqrMagnitude + d[2].sqrMagnitude + d[3].sqrMagnitude) * invMass;

                    if (Math.Abs(lambda) < eps) continue;

                    if (i == j)
                    {
                        // diagonal, stretch
                        if (NormalizeStretch)
                        {
                            double s = Math.Sqrt(Sij);
                            lambda = 2.0 * s * (s - 1.0) / lambda * Stiffness;
                        }
                        else
                        {
                            lambda = (Sij - 1.0) / lambda * Stiffness;
                        }
                    }
                    else
                    {
                        // off diagonal, shear
                        lambda = Sij / lambda * Stiffness;
                    }

                    Correction[0] -= (float)lambda * (float)invMass * d[0];
                    Correction[1] -= (float)lambda * (float)invMass * d[1];
                    Correction[2] -= (float)lambda * (float)invMass * d[2];
                    Correction[3] -= (float)lambda * (float)invMass * d[3];
                }
            }

            return true;
        }

    }

}