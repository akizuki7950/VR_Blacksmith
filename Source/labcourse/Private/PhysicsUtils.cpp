// Fill out your copyright notice in the Description page of Project Settings.


#include "PhysicsUtils.h"

void FPlaneCollisionConstraint::Solve(TArray<FPBDParticle>& particles, float stiffness, float dt)
{
	FPBDParticle& Particle = particles[Index1];

	if (PenetrationDepth > 0.0)
	{
		const FVector Delta = PenetrationDepth * CollisionNormal;
		Particle.PredictedPosition = Particle.PredictedPosition + Delta;
		//Particle.AccDelta += Delta;
	}
}

void FDistanceConstraint::Solve(TArray<FPBDParticle>& particles, float stiffness, float dt)
{
	FPBDParticle& p1 = particles[Index1];
	FPBDParticle& p2 = particles[Index2];

	FVector Delta = p2.PredictedPosition - p1.PredictedPosition;
	float CurrentLength = Delta.Length();
	FVector Direction = Delta / CurrentLength;

	FVector AdjustLength = (CurrentLength - RestLength) * Direction;

	float Weight_p1 = p1.InvMass / (p1.InvMass + p2.InvMass + 1e-9f);
	float Weight_p2 = -p2.InvMass / (p1.InvMass + p2.InvMass + 1e-9f);

	p1.PredictedPosition = p1.PredictedPosition + Weight_p1 * AdjustLength * stiffness;
	p2.PredictedPosition = p2.PredictedPosition + Weight_p2 * AdjustLength * stiffness;

	//p1.AccDelta += Weight_p1 * AdjustLength * stiffness;
	//p2.AccDelta += Weight_p2 * AdjustLength * stiffness;

}

void FVolumeConstraint::Solve(TArray<FPBDParticle>& particles, float stiffness, float dt)
{
	FPBDParticle& p1 = particles[Index1];
	FPBDParticle& p2 = particles[Index2];
	FPBDParticle& p3 = particles[Index3];
	FPBDParticle& p4 = particles[Index4];

	CurrentVolume = GetTetVolume(p1.PredictedPosition, p2.PredictedPosition, p3.PredictedPosition, p4.PredictedPosition);

	//ensure(CurrentVolume > 0.0f);

	float CVolumeDiff = 6 * (CurrentVolume - RestVolume);

	FVector dC_p1 = FVector::CrossProduct(p4.PredictedPosition - p2.PredictedPosition, p3.PredictedPosition - p2.PredictedPosition);
	FVector dC_p2 = FVector::CrossProduct(p3.PredictedPosition - p1.PredictedPosition, p4.PredictedPosition - p1.PredictedPosition);
	FVector dC_p3 = FVector::CrossProduct(p4.PredictedPosition - p1.PredictedPosition, p2.PredictedPosition - p1.PredictedPosition);
	FVector dC_p4 = FVector::CrossProduct(p2.PredictedPosition - p1.PredictedPosition, p3.PredictedPosition - p1.PredictedPosition);
	float t1 = FVector::DotProduct(dC_p1, p1.PredictedPosition - p2.PredictedPosition);
	float t2 = FVector::DotProduct(dC_p2, p2.PredictedPosition - p1.PredictedPosition);
	float t3 = FVector::DotProduct(dC_p3, p3.PredictedPosition - p1.PredictedPosition);
	float t4 = FVector::DotProduct(dC_p4, p4.PredictedPosition - p1.PredictedPosition);


	float Lambda = -CVolumeDiff / (p1.InvMass * dC_p1.SquaredLength() + p2.InvMass * dC_p2.SquaredLength() + p3.InvMass * dC_p3.SquaredLength() + p4.InvMass * dC_p4.SquaredLength() + 1e-9f);

	p1.PredictedPosition = p1.PredictedPosition + Lambda * p1.InvMass * dC_p1 * stiffness;
	p2.PredictedPosition = p2.PredictedPosition + Lambda * p2.InvMass * dC_p2 * stiffness;
	p3.PredictedPosition = p3.PredictedPosition + Lambda * p3.InvMass * dC_p3 * stiffness;
	p4.PredictedPosition = p4.PredictedPosition + Lambda * p4.InvMass * dC_p4 * stiffness;

	//p1.AccDelta += Lambda * p1.InvMass * dC_p1 * stiffness;
	//p2.AccDelta += Lambda * p2.InvMass * dC_p2 * stiffness;
	//p3.AccDelta += Lambda * p3.InvMass * dC_p3 * stiffness;
	//p4.AccDelta += Lambda * p4.InvMass * dC_p4 * stiffness;

}

void FDistanceConstraint::UpdateConstraintPlasticity(TArray<FPBDParticle>& particles, float YieldFactor)
{
	float CurrentStrain = CalculateCurrentStrain(particles);

	float CurrentHardenedBaseYieldStrain = BaseYieldStrain * (1.0f + AccumulatedPlasticStrain);
	float EffectiveYieldStrain = YieldFactor * CurrentHardenedBaseYieldStrain;

	if (FMath::Abs(CurrentStrain) > EffectiveYieldStrain)
	{
		float PlasticDeformationAmount = CurrentStrain - FMath::Sign(CurrentStrain) * EffectiveYieldStrain;
		RestLength += PlasticDeformationAmount;

		if (FMath::Abs(InitRestLength) > KINDA_SMALL_NUMBER)
		{
			AccumulatedPlasticStrain += FMath::Abs(PlasticDeformationAmount / InitRestLength);
		}

	}
}

float FDistanceConstraint::CalculateAverageTemperature(TArray<FPBDParticle>& particles)
{
	return (particles[Index1].Temperature + particles[Index2].Temperature) / 2.0;
}
float FDistanceConstraint::CalculateCurrentStrain(TArray<FPBDParticle>& particles)
{
	float CurrentLength = (particles[Index1].Position - particles[Index2].Position).Length();
	return CurrentLength - RestLength;
}

float FVolumeConstraint::CalculateAverageTemperature(TArray<FPBDParticle>& particles)
{
	return (particles[Index1].Temperature + particles[Index2].Temperature + particles[Index3].Temperature + particles[Index4].Temperature) / 4.0;
}
float FVolumeConstraint::CalculateCurrentStrain(TArray<FPBDParticle>& particles)
{
	return CurrentVolume - RestVolume;
}