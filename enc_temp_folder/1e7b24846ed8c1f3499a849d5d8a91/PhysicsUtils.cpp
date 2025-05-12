// Fill out your copyright notice in the Description page of Project Settings.


#include "PhysicsUtils.h"

void FPlaneCollisionConstraint::Solve(TArray<FPBDParticle>& particles, float stiffness, float dt)
{
	FPBDParticle& Particle = particles[Index1];

	if (PenetrationDepth > 0.0)
	{
		const FVector Delta = PenetrationDepth * CollisionNormal;
		Particle.PredictedPosition = Particle.PredictedPosition + Delta;
	}
}