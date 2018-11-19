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
            applyExternalForcesShaderHandle = ApplyExternalForcesShader.FindKernel("APPLYEXTERNALFORCE_1");
            estimatePositionsShaderHandle = EstimatePositionsShader.FindKernel("ESTIMATEPOSITIONS");
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
                    ApplyExternalForcesShader.SetFloat("gravity",- 9.81f);

                    ComputeBuffer velocityComputeBuffer = new ComputeBuffer(12, body.NumParticles);
                    velocityComputeBuffer.SetData(body.Velocities);

                    ApplyExternalForcesShader.SetBuffer(applyExternalForcesShaderHandle, "Velocities", velocityComputeBuffer);
                    ApplyExternalForcesShader.Dispatch(applyExternalForcesShaderHandle, (body.NumParticles/ 32)+1, 1, 1);

                    velocityComputeBuffer.GetData(body.Velocities);

                    velocityComputeBuffer.Release();
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

                    ComputeBuffer predictedComputeBuffer = new ComputeBuffer(16, body.NumParticles);
                    predictedComputeBuffer.SetData(body.Predicted);

                    ComputeBuffer positionsComputeBuffer = new ComputeBuffer(16, body.NumParticles);
                    positionsComputeBuffer.SetData(body.Positions);

                    ComputeBuffer velocityComputeBuffer = new ComputeBuffer(16, body.NumParticles);
                    velocityComputeBuffer.SetData(body.Velocities);

                    EstimatePositionsShader.SetBuffer(estimatePositionsShaderHandle,"Predicted", predictedComputeBuffer);
                    EstimatePositionsShader.SetBuffer(estimatePositionsShaderHandle, "Positions", positionsComputeBuffer);
                    EstimatePositionsShader.SetBuffer(estimatePositionsShaderHandle, "Velocities", velocityComputeBuffer);

                    EstimatePositionsShader.Dispatch(estimatePositionsShaderHandle, (body.NumParticles/ 32)+1, 1, 1);
                    predictedComputeBuffer.GetData(body.Predicted);

                    predictedComputeBuffer.Release();
                    positionsComputeBuffer.Release();
                    velocityComputeBuffer.Release();
                }
                else { 
                    for (int i = 0; i < body.NumParticles; i++)
                    {
                        body.Predicted[i] = body.Positions[i] + dt * body.Velocities[i];
                    }
                }
            }
        }

        private void UpdateBounds()
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                Bodies[i].UpdateBounds();
            }
        }

        private void ResolveCollisions()
        {
            List<CollisionContact3d> contacts = new List<CollisionContact3d>();

            for (int i = 0; i < Collisions.Count; i++)
            {
                Collisions[i].FindContacts(Bodies, contacts);
            }

            double di = 1.0 / CollisionIterations;

            for(int i = 0; i < CollisionIterations; i++)
            {
                for (int j = 0; j < contacts.Count; j++)
                {
                    contacts[j].ResolveContact(di);
                }
            }
        }

        private void ConstrainPositions()
        {
            double di = 1.0 / SolverIterations;

            for (int i = 0; i < SolverIterations; i++)
            {
                for (int j = 0; j < Bodies.Count; j++)
                {
                    Bodies[j].ConstrainPositions(di);
                }
            }
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
                    ComputeBuffer predictedComputeBuffer = new ComputeBuffer(12, body.NumParticles);
                    predictedComputeBuffer.SetData(body.Predicted);

                    ComputeBuffer positionComputeBuffer = new ComputeBuffer(12, body.NumParticles);
                    positionComputeBuffer.SetData(body.Positions);

                    ComputeBuffer velocityComputeBuffer = new ComputeBuffer(12, body.NumParticles);
                    velocityComputeBuffer.SetData(body.Velocities);

                    UpdateVelocitiesShader.SetBuffer(updateVelocitiesShaderHandle,"Predicted",predictedComputeBuffer);
                    UpdateVelocitiesShader.SetBuffer(updateVelocitiesShaderHandle, "Positions", positionComputeBuffer);
                    UpdateVelocitiesShader.SetBuffer(updateVelocitiesShaderHandle, "Velocities", velocityComputeBuffer);

                    UpdateVelocitiesShader.Dispatch(updateVelocitiesShaderHandle, (body.NumParticles / 32) + 1, 1, 1);

                    velocityComputeBuffer.GetData(body.Velocities);

                    velocityComputeBuffer.Release();
                    positionComputeBuffer.Release();
                    predictedComputeBuffer.Release();

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

        private void ConstrainVelocities()
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                Bodies[i].ConstrainVelocities();
            }
        }

        private void UpdatePositions()
        {
            for (int j = 0; j < Bodies.Count; j++)
            {
                Body3d body = Bodies[j];

                if (GPUmode)
                {
                    UpdatePositionsShader.SetInt("numParticles", body.NumParticles);
                
                    ComputeBuffer predictedComputeBuffer = new ComputeBuffer(12, body.NumParticles);
                    predictedComputeBuffer.SetData(body.Predicted);

                    ComputeBuffer positionComputeBuffer = new ComputeBuffer(12, body.NumParticles);
                    positionComputeBuffer.SetData(body.Positions);

                    UpdatePositionsShader.SetBuffer(updatePositionsShaderHandle, "Predicted", predictedComputeBuffer);
                    UpdatePositionsShader.SetBuffer(updatePositionsShaderHandle, "Positions", positionComputeBuffer);

                    UpdatePositionsShader.Dispatch(updatePositionsShaderHandle, (body.NumParticles/32)+1, 1, 1);

                    positionComputeBuffer.GetData(body.Positions);

                    predictedComputeBuffer.Dispose();
                    positionComputeBuffer.Dispose();
                }
                else
                {
                    for (int i = 0; i < body.NumParticles; i++)
                    {
                        body.Positions[i] = body.Predicted[i];
                    }
                }
            }
        }

    }

}