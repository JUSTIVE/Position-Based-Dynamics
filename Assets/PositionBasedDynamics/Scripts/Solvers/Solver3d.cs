using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Constraints;
using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Collisions;
using UnityEngine;
namespace PositionBasedDynamics.Solvers
{

    public class Solver3d
    {
        public int SolverIterations { get; set; }

        public int CollisionIterations { get; set; }

        public double SleepThreshold { get; set; }

        public List<Body3d> Bodies { get; private set; }

        private List<ExternalForce3d> Forces { get; set; }

        private List<Collision3d> Collisions { get; set; }
        //computeShaders
        public ComputeShader ApplyExternalForcesShader;
        public ComputeShader EstimatePositionsShader;
        public ComputeShader ResolveCollisionsShader;
        public ComputeShader ConstraintPositionsShader;
        public ComputeShader UpdateVelocitiesShader;
        public ComputeShader ConstraintVelocitiesShader;
        public ComputeShader UpdatePositionsShader;
        //
        private ComputeBuffer[] EPS_UV_UPpredictedComputeBuffer;
        private ComputeBuffer[] EPS_UV_UPpositionsComputeBuffer;
        private ComputeBuffer[] AEFS_EPS_UVvelocityComputeBuffer;

        //computeShaderHandles
        private int applyExternalForcesShaderHandle;
        private int estimatePositionsShaderHandle;
        private int resolveCollisionsShaderHandle;
        private int constraintPositionsShaderHandle;
        private int updateVelocitiesShaderHandle;
        private int constraintVelocitiesShaderHandle;
        private int updatePositionsShaderHandle;

        public bool GPUmode = true;//true for GPU;
        public void init()
        {
            //init applyExternalForcesShaderHandle
            applyExternalForcesShaderHandle = ApplyExternalForcesShader.FindKernel("APPLYEXTERNALFORCE_1");
            ApplyExternalForcesShader.SetFloat("gravity", -9.81f);

            AEFS_EPS_UVvelocityComputeBuffer = new ComputeBuffer[Bodies.Count];
            for (int j = 0; j < Bodies.Count; j++)
            {
                Body3d body = Bodies[j];
                AEFS_EPS_UVvelocityComputeBuffer[j] = new ComputeBuffer(12, body.NumParticles);
            }

            estimatePositionsShaderHandle = EstimatePositionsShader.FindKernel("ESTIMATEPOSITIONS");
            EPS_UV_UPpredictedComputeBuffer = new ComputeBuffer[Bodies.Count];
            EPS_UV_UPpositionsComputeBuffer = new ComputeBuffer[Bodies.Count];
            AEFS_EPS_UVvelocityComputeBuffer = new ComputeBuffer[Bodies.Count];
            for (int j = 0; j < Bodies.Count; j++)
            {
                Body3d body = Bodies[j];
                EPS_UV_UPpredictedComputeBuffer[j] = new ComputeBuffer(16, body.NumParticles);
                EPS_UV_UPpositionsComputeBuffer[j] = new ComputeBuffer(16, body.NumParticles);
                AEFS_EPS_UVvelocityComputeBuffer[j] = new ComputeBuffer(16, body.NumParticles);
            }




            updatePositionsShaderHandle = UpdatePositionsShader.FindKernel("UPDATEPOSITION");
            updateVelocitiesShaderHandle = UpdateVelocitiesShader.FindKernel("UPDATEVELOCITIES");
        }

        public Solver3d()
        {
            SolverIterations = 4;
            CollisionIterations = 1;

            Forces = new List<ExternalForce3d>();
            Collisions = new List<Collision3d>();
            Bodies = new List<Body3d>();
        }

        public void AddForce(ExternalForce3d force)
        {
            if (Forces.Contains(force)) return;
            Forces.Add(force);
        }

        public void AddCollision(Collision3d collision)
        {
            if (Collisions.Contains(collision)) return;
            Collisions.Add(collision);
        }

        public void AddBody(Body3d body)
        {
            if (Bodies.Contains(body)) return;
            Bodies.Add(body);
        }

        public void StepPhysics(double dt)
        {
            if (dt == 0.0) return;

            AppyExternalForces(dt);

            EstimatePositions((float)dt);

            UpdateBounds();

            ResolveCollisions();

            ConstrainPositions();

            UpdateVelocities(dt);

            UpdatePositions();

            UpdateBounds();

        }

        private void AppyExternalForces(double dt)
        {
            ApplyExternalForcesShader.SetFloat("dt", (float)dt);
            
            for (int j = 0; j < Bodies.Count; j++)
            {
                Body3d body = Bodies[j];
                if (GPUmode) {
                    ApplyExternalForcesShader.SetInt("numParticles", body.NumParticles);
                    ApplyExternalForcesShader.SetFloat("damping", (float)body.Dampning);
                    AEFS_EPS_UVvelocityComputeBuffer[j].SetData(body.Velocities);

                    ApplyExternalForcesShader.SetBuffer(applyExternalForcesShaderHandle, "Velocities", AEFS_EPS_UVvelocityComputeBuffer[j]);
                    ApplyExternalForcesShader.Dispatch(applyExternalForcesShaderHandle, (body.NumParticles/ 32)+1, 1, 1);

                    AEFS_EPS_UVvelocityComputeBuffer[j].GetData(body.Velocities);

                    
                }
                else { 
                    for (int i = 0; i < body.NumParticles; i++)
                    {
                        body.Velocities[i] -= (body.Velocities[i] * (float)body.Dampning) * (float)dt;
                    }

                    for (int i = 0; i < Forces.Count; i++)
                    {
                        Forces[i].ApplyForce(dt, body);
                    }
                }
            }
        }
        private void EstimatePositions(float dt)
        {
            EstimatePositionsShader.SetFloat("dt", (float)dt);
            for (int j = 0; j < Bodies.Count; j++)
            {
                Body3d body = Bodies[j];
                if (GPUmode) {
                    EstimatePositionsShader.SetInt("numParticles", body.NumParticles);

                    EPS_UV_UPpredictedComputeBuffer[j].SetData(body.Predicted);
                    EPS_UV_UPpositionsComputeBuffer[j].SetData(body.Positions);
                    AEFS_EPS_UVvelocityComputeBuffer[j].SetData(body.Velocities);

                    EstimatePositionsShader.SetBuffer(estimatePositionsShaderHandle,"Predicted", EPS_UV_UPpredictedComputeBuffer[j]);
                    EstimatePositionsShader.SetBuffer(estimatePositionsShaderHandle, "Positions", EPS_UV_UPpositionsComputeBuffer[j]);
                    EstimatePositionsShader.SetBuffer(estimatePositionsShaderHandle, "Velocities", AEFS_EPS_UVvelocityComputeBuffer[j]);

                    EstimatePositionsShader.Dispatch(estimatePositionsShaderHandle, (body.NumParticles/ 32)+1, 1, 1);
                    EPS_UV_UPpredictedComputeBuffer[j].GetData(body.Predicted);
                }
                else { 
                    for (int i = 0; i < body.NumParticles; i++)
                        body.Predicted[i] = body.Positions[i] + dt * body.Velocities[i];
                }
            }
        }
        //CANNOT BE PARALLELED
        private void UpdateBounds()
        {
            for (int i = 0; i < Bodies.Count; i++)
                Bodies[i].UpdateBounds();
        }

        private void ResolveCollisions()
        {
            List<CollisionContact3d> contacts = new List<CollisionContact3d>();

            for (int i = 0; i < Collisions.Count; i++)
                Collisions[i].FindContacts(Bodies, contacts);

            double di = 1.0 / CollisionIterations;

            for(int i = 0; i < CollisionIterations; i++)
                for (int j = 0; j < contacts.Count; j++)
                    contacts[j].ResolveContact(di);
        }

        private void ConstrainPositions()
        {
            double di = 1.0 / SolverIterations;

            for (int i = 0; i < SolverIterations; i++)
                for (int j = 0; j < Bodies.Count; j++)
                    Bodies[j].ConstrainPositions(di);
        }

        private void UpdateVelocities(double dt)
        {
            double invDt = 1.0 / dt;
            double threshold2 = SleepThreshold * dt;
            threshold2 *= threshold2;

            UpdateVelocitiesShader.SetFloat("invDt", (float)invDt);
            UpdateVelocitiesShader.SetFloat("threshold2", (float)threshold2);

            for (int j = 0; j < Bodies.Count; j++)
            {
                Body3d body = Bodies[j];

                if (GPUmode)
                {
                    UpdateVelocitiesShader.SetInt("numParticles", body.NumParticles);

                    EPS_UV_UPpredictedComputeBuffer[j].SetData(body.Predicted);
                    EPS_UV_UPpositionsComputeBuffer[j].SetData(body.Positions);
                    AEFS_EPS_UVvelocityComputeBuffer[j].SetData(body.Velocities);

                    UpdateVelocitiesShader.SetBuffer(updateVelocitiesShaderHandle,"Predicted", EPS_UV_UPpredictedComputeBuffer[j]);
                    UpdateVelocitiesShader.SetBuffer(updateVelocitiesShaderHandle, "Positions", EPS_UV_UPpositionsComputeBuffer[j]);
                    UpdateVelocitiesShader.SetBuffer(updateVelocitiesShaderHandle, "Velocities", AEFS_EPS_UVvelocityComputeBuffer[j]);

                    UpdateVelocitiesShader.Dispatch(updateVelocitiesShaderHandle, (body.NumParticles / 32) + 1, 1, 1);

                    AEFS_EPS_UVvelocityComputeBuffer[j].GetData(body.Velocities);
                }
                else { 
                    for (int i = 0; i < body.NumParticles; i++)
                    {
                        Vector3 d = body.Predicted[i] - body.Positions[i];
                        body.Velocities[i] = d * (float)invDt;

                        double m = body.Velocities[i].sqrMagnitude;
                        if (m < threshold2)
                            body.Velocities[i] = Vector3.zero;
                    }
                }
            }
        }

        //private void ConstrainVelocities()
        //{
        //    for (int i = 0; i < Bodies.Count; i++)
        //        Bodies[i].ConstrainVelocities();
        //}

        private void UpdatePositions()
        {
            for (int j = 0; j < Bodies.Count; j++)
            {
                Body3d body = Bodies[j];

                if (GPUmode)
                {
                    UpdatePositionsShader.SetInt("numParticles", body.NumParticles);
                    
                    EPS_UV_UPpredictedComputeBuffer[j].SetData(body.Predicted);
                    EPS_UV_UPpositionsComputeBuffer[j].SetData(body.Positions);

                    UpdatePositionsShader.SetBuffer(updatePositionsShaderHandle, "Predicted", EPS_UV_UPpredictedComputeBuffer[j]);
                    UpdatePositionsShader.SetBuffer(updatePositionsShaderHandle, "Positions", EPS_UV_UPpositionsComputeBuffer[j]);

                    UpdatePositionsShader.Dispatch(updatePositionsShaderHandle, (body.NumParticles/32)+1, 1, 1);

                    EPS_UV_UPpositionsComputeBuffer[j].GetData(body.Positions);

                }
                else
                {
                    for (int i = 0; i < body.NumParticles; i++)
                        body.Positions[i] = body.Predicted[i];
                }
            }
        }

    }

}