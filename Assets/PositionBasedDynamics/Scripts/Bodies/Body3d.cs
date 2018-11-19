using System;
using System.Collections.Generic;
using UnityEngine;
using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;

using PositionBasedDynamics.Constraints;

namespace PositionBasedDynamics.Bodies
{

    public abstract class Body3d
    {
        public int NumParticles { get { return Positions.Length; } }

        public int NumConstraints { get { return Constraints.Count; } }

        public double Dampning { get; set; }

        public double ParticleRadius { get; protected set; }

        public double ParticleDiameter { get { return ParticleRadius * 2.0; } }

        public double ParticleMass { get; protected set; }

        public Vector3[] Positions { get; private set; }

        public Vector3[] Predicted { get; private set; }

        public Vector3[] Velocities { get; private set; }

        public Box3d Bounds { get; private set; }

        public List<Constraint3d> Constraints { get; private set; }

        private List<StaticConstraint3d> StaticConstraints { get; set; }

        public Body3d(int numParticles, double radius, double mass)
        {
            Positions = new Vector3[numParticles];
            Predicted = new Vector3[numParticles];
            Velocities = new Vector3[numParticles];
            Constraints = new List<Constraint3d>();
            StaticConstraints = new List<StaticConstraint3d>();

            ParticleRadius = radius;
            ParticleMass = mass;
            Dampning = 1;

            if (ParticleMass <= 0)
                throw new ArgumentException("Particles mass <= 0");

            if (ParticleRadius <= 0)
                throw new ArgumentException("Particles radius <= 0");
        }

        internal void ConstrainPositions(double di)
        {
            for (int i = 0; i < Constraints.Count; i++)
            {
                Constraints[i].ConstrainPositions(di);
            }

            for (int i = 0; i < StaticConstraints.Count; i++)
            {
                StaticConstraints[i].ConstrainPositions(di);
            }
        }

        internal void ConstrainVelocities()
        {

            for (int i = 0; i < Constraints.Count; i++)
            {
                Constraints[i].ConstrainVelocities();
            }

            for (int i = 0; i < StaticConstraints.Count; i++)
            {
                StaticConstraints[i].ConstrainVelocities();
            }

        }

        public void RandomizePositions(System.Random rnd, double amount)
        {
            for(int i = 0; i < NumParticles; i++)
            {
                double rx = rnd.NextDouble() * 2.0 - 1.0;
                double ry = rnd.NextDouble() * 2.0 - 1.0;
                double rz = rnd.NextDouble() * 2.0 - 1.0;

                Positions[i] += new Vector3((float)rx, (float)ry, (float)rz) * (float)amount;
            }
        }

        public void RandomizeConstraintOrder(System.Random rnd)
        {
            int count = Constraints.Count;
            if (count <= 1) return;

            List<Constraint3d> tmp = new List<Constraint3d>();

            while (tmp.Count != count)
            {
                int i = rnd.Next(0, Constraints.Count - 1);

                tmp.Add(Constraints[i]);
                Constraints.RemoveAt(i);
            }

            Constraints = tmp;
        }

        public void MarkAsStatic(Box3d bounds)
        {
            for (int i = 0; i < NumParticles; i++)
            {
                if (bounds.Contains(Positions[i]))
                {
                    StaticConstraints.Add(new StaticConstraint3d(this, i));
                }
            }
        }

        public void UpdateBounds()
        {
            Vector3 min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector3 max = new Vector3(float.MinValue, float.MinValue, float.MinValue);

            for (int i = 0; i < NumParticles; i++)
            {
                min.x = min.x < Positions[i].x ? min.x : Positions[i].x;
                min.y = min.y < Positions[i].y ? min.y : Positions[i].y;
                min.z = min.z < Positions[i].z ? min.z : Positions[i].z;

                max.x = max.x > Positions[i].x ? max.x : Positions[i].x;
                max.y = max.y > Positions[i].y ? max.y : Positions[i].y;
                max.z = max.z > Positions[i].z ? max.z : Positions[i].z;
            }

            min.x -= (float)ParticleRadius;
            min.y -= (float)ParticleRadius;
            min.z -= (float)ParticleRadius;

            max.x += (float)ParticleRadius;
            max.y += (float)ParticleRadius;
            max.z += (float)ParticleRadius;

            Bounds = new Box3d(min, max);
        }

    }

}