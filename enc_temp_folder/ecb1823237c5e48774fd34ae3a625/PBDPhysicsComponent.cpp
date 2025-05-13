// Fill out your copyright notice in the Description page of Project Settings.


#include "PBDPhysicsComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"

// Sets default values for this component's properties
UPBDPhysicsComponent::UPBDPhysicsComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UPBDPhysicsComponent::BeginPlay()
{
	Super::BeginPlay();

	Init();

	CachedStiffnessTemperatureFactorCurve = nullptr;
	if (StiffnessTemperatureFactorCurve.IsValid())
	{
		CachedStiffnessTemperatureFactorCurve = StiffnessTemperatureFactorCurve.Get();
		if (!CachedStiffnessTemperatureFactorCurve)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to load Stiffness Curve: %s"), *StiffnessTemperatureFactorCurve.ToString());
		}
	}

	CachedYieldTemperatureFactorCurve = nullptr;
	if (YieldTemperatureFactorCurve.IsValid())
	{
		CachedYieldTemperatureFactorCurve = YieldTemperatureFactorCurve.Get();
		if (!CachedYieldTemperatureFactorCurve)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to load Yield Curve: %s"), *YieldTemperatureFactorCurve.ToString());
		}
	}
}


// Called every frame
void UPBDPhysicsComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	//UE_LOG(LogTemp, Warning, TEXT("Tick PBD Comp"));

	if (bDrawDebugShapes)
	{
		DrawDebugShapes();
	}

	UpdateGrabComponents();
	UpdateKinematics();
	Simulate(DeltaTime);
	UpdateOwnerPos();
	// ...
}

void UPBDPhysicsComponent::UpdateKinematics()
{
	
}

void UPBDPhysicsComponent::UpdateGrabComponents()
{
	
}


void UPBDPhysicsComponent::Simulate(float dt)
{
	FTransform TOwnerWorld = GetOwner()->GetActorTransform();
	FTransform TOwnerWorldInv = TOwnerWorld.Inverse();

	FVector Gravity(0, 0, -980);
	Gravity = TOwnerWorldInv.TransformVectorNoScale(Gravity) * SimulationScale;

	// Calc predicted positions
	for(auto& Particle : PBDParticles)
	{
		if (!Particle.bIsKinematic)
		{
			Particle.Velocity += Gravity * dt;
			Particle.Velocity *= DampingFactor;
			Particle.PredictedPosition = Particle.Position + Particle.Velocity * dt;
		}
	}

	//UE_LOG(LogTemp, Warning, TEXT("Num Pool: %d"), CollisionConstraintPool.Num());

	// Substep
	float step = dt / SolverIterations;
	for (int ite = 0; ite < SolverIterations; ite++)
	{
		// Reset collision pool
		CollisionConstraintPool.Append(ActiveCollisionConstraints);
		ActiveCollisionConstraints.Empty();

		// Check plane penetration
		for (int i = 0; i < PBDParticles.Num(); i++)
		{
			auto Particle = PBDParticles[i];
			for (auto& Plane : CollisionPlanes)
			{
				// Transform plane into simulation space
				FVector PlanePointSimSpace = TOwnerWorldInv.TransformPosition(Plane.Point) * SimulationScale;
				FVector PlaneNormalSimSpace = TOwnerWorldInv.TransformVectorNoScale(Plane.Normal);

				float SD = FVector::DotProduct(Particle.PredictedPosition - PlanePointSimSpace, PlaneNormalSimSpace);
				if (SD < 0)
				{
					if (CollisionConstraintPool.Num() > 0) {
						FPlaneCollisionConstraint* CollisionConstraint = CollisionConstraintPool.Pop();
						if (CollisionConstraint)
						{
							CollisionConstraint->Set(i, PlaneNormalSimSpace, SD, Plane.FrictionFactor);
							ActiveCollisionConstraints.Add(CollisionConstraint);
						}
					}
				}
			}
		}

		// Solve permanent constraints
		for (auto Constraint : PBDConstraints)
		{
			float AvgTemp = Constraint->CalculateAverageTemperature(PBDParticles);
			float EffectiveStiffnessFactor = GetEffectiveTemperatureFactorFromCurve(CachedStiffnessTemperatureFactorCurve, AvgTemp);
			float EffectiveStiffness = BaseStiffness * EffectiveStiffnessFactor;
			Constraint->Solve(PBDParticles, EffectiveStiffness, step);
		}

		// Solve temporary collision constraints
		for (auto TempConstraint : ActiveCollisionConstraints)
		{
			TempConstraint->Solve(PBDParticles, 1.0, step);
		}
	}

	for (auto& Particle : PBDParticles)
	{
		if (!Particle.bIsKinematic)
		{
			Particle.Velocity = (Particle.PredictedPosition - Particle.Position) / dt;
			Particle.Position = Particle.PredictedPosition;
		}
	}

	// Friction
	for (auto& TempConstraint : ActiveCollisionConstraints)
	{
		FPBDParticle& Particle = PBDParticles[TempConstraint->Index1];
		FVector pv = Particle.Velocity;
		FVector pvNorm = FVector::DotProduct(pv, TempConstraint->CollisionNormal) * TempConstraint->CollisionNormal;
		FVector pvTan = pv - pvNorm;
		pvTan *= TempConstraint->DampingFactor;
		Particle.Velocity = pvNorm + pvTan;
	}

	UpdatePlasticity();

}

void UPBDPhysicsComponent::UpdatePlasticity()
{
	if (PBDParticles.IsEmpty()) return;

	for (auto Constraint : PBDConstraints)
	{
		float AvgTemp = Constraint->CalculateAverageTemperature(PBDParticles);
		float EffectiveYieldFactor = GetEffectiveTemperatureFactorFromCurve(CachedYieldTemperatureFactorCurve, AvgTemp);

		Constraint->UpdateConstraintPlasticity(PBDParticles, EffectiveYieldFactor);

	}

}

void UPBDPhysicsComponent::UpdateOwnerPos()
{
	if (PBDParticles.IsEmpty()) return;

	FVector CenterOfMass = FVector::ZeroVector;
	float TotalWeight = 0.0f;
	for (auto& Particle : PBDParticles)
	{
		if (Particle.InvMass > 0)
		{
			CenterOfMass += Particle.Mass * Particle.Position;
			TotalWeight += Particle.Mass;
		}
	}

	CenterOfMass /= TotalWeight;
	FVector CenterOfMass_Sim = CenterOfMass;

	FTransform TOwnerWorld = GetOwner()->GetActorTransform();
	CenterOfMass = TOwnerWorld.TransformPositionNoScale(CenterOfMass / SimulationScale);

	GetOwner()->SetActorLocation(CenterOfMass);

	for (auto& Particle : PBDParticles)
	{
		Particle.Position -= CenterOfMass_Sim;
		Particle.PredictedPosition -= CenterOfMass_Sim;
	}
}


void UPBDPhysicsComponent::Init()
{
	InitParticles();
	InitConstraints();
	InitCollisionPlanes();
}

void UPBDPhysicsComponent::InitParticles()
{
	FVector ND = Dimension / Res;
	FVector HalfDim = Dimension / 2.0;
	Nx = ND.X, Ny = ND.Y, Nz = ND.Z;

	float interval = Dimension.X / Nx;

	for (int i = 0; i < Nz; i++)
	{
		for (int j = 0; j < Ny; j++)
		{
			for (int k = 0; k < Nx; k++)
			{
				FVector pos(k * interval, j * interval, i * interval);
				pos -= HalfDim;
				pos *= SimulationScale;
				PBDParticles.Add(FPBDParticle(pos, 1.0));

				// Grab component
				if (i % GrabComponentInterval == 0 && j % GrabComponentInterval == 0 && k % GrabComponentInterval == 0)
				{
					int32 Idx = GetOffset(k, j, i);
					TObjectPtr<UPBDGrabComponent> NewGC = NewObject<UPBDGrabComponent>();
					GetOwner()->AddOwnedComponent(NewGC);
					PBDGrabComponents.Add(NewGC);
					NewGC->ParticleToFollow = &PBDParticles[Idx];
					NewGC->Radius = GrabComponentRadius;
					FVector pLocWorld = ConvertPositionSimToWorld(PBDParticles[Idx].Position);
					NewGC->SetWorldLocation(pLocWorld);
				}
			}
		}
	}

	for (int i = 0; i < Nz; i++)
	{
		for (int j = 0; j < Ny; j++)
		{
			for (int k = 0; k < Nx; k++)
			{
				int32 Idx = GetOffset(k, j, i);
				int32 nIdx1 = Idx + GetOffset(1, 0, 0);
				int32 nIdx2 = Idx + GetOffset(0, 1, 0);
				int32 nIdx3 = Idx + GetOffset(0, 0, 1);

				if (k < Nx - 1)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdx1]);
				}
				if (j < Ny - 1)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdx2]);
				}
				if (i < Nz - 1)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdx3]);
				}
			}
		}
	}
	
}

void UPBDPhysicsComponent::InitConstraints()
{
	TSet<TTuple<int32, int32>> ProcessedEdges;
	
	for (int i = 0; i < PBDParticles.Num(); i++) {

		TArray<TArray<int32>> TetrahedronOffsetSets;
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(0, 1, 0),  GetOffset(1, 1, 1),  GetOffset(0, 1, 1) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(0, 1, 1),  GetOffset(0, 0, 1),  GetOffset(1, 1, 1) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(0, 0, 1),  GetOffset(1, 1, 1),  GetOffset(1, 0, 1) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(1, 0, 1),  GetOffset(1, 1, 1),  GetOffset(1, 0, 0) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(1, 0, 0),  GetOffset(1, 1, 1),  GetOffset(1, 1, 0) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(1, 1, 0),  GetOffset(1, 1, 1),  GetOffset(0, 1, 0) });

		
		for (int t = 0; t < TetrahedronOffsetSets.Num(); t++)
		{
			auto TOffset = TetrahedronOffsetSets[t];
			bool IsTetraValid = true;

			TArray<int32> Tet = TOffset;

			for (int j = 0; j < 4; j++)
			{
				// Get vertex indices
				Tet[j] += i;
				if (Tet[j] < 0 || Tet[j] >= PBDParticles.Num())
				{
					IsTetraValid = false;
				}
			}

			if (IsTetraValid)
			{

				// Distance Constraints
				for (int j = 0; j < 4; j++)
				{
					for (int k = j+1; k < 4; k++)
					{
						// Check if edge processed
						bool IsEdgeProcessed = false;
						ProcessedEdges.FindOrAdd({ Tet[j], Tet[k] }, &IsEdgeProcessed);
						if (!IsEdgeProcessed)
						{
							float Dist = (PBDParticles[Tet[j]].Position - PBDParticles[Tet[k]].Position).Length();
							FDistanceConstraint* NewConstr = new FDistanceConstraint(Tet[j], Tet[k], Dist, YieldFactor);
							PBDConstraints.Add(NewConstr);
						}
					}
				}

				// Volume Constraint
				float Vol = GetTetVolume(PBDParticles[Tet[0]].Position, PBDParticles[Tet[1]].Position, PBDParticles[Tet[2]].Position, PBDParticles[Tet[3]].Position);
				PBDConstraints.Add(new FVolumeConstraint(Tet[0], Tet[1], Tet[2], Tet[3], Vol));
			}

		}
	}

	//UE_LOG(LogTemp, Warning, TEXT("Num Constraints: %d"), PBDConstraints.Num());

}

void UPBDPhysicsComponent::InitCollisionPlanes()
{

	for (int i = 0; i < CollisionConstraintPoolSize; i++)
	{
		CollisionConstraintPool.Add(new FPlaneCollisionConstraint());
	}

	for (auto PlaneOpt : PlaneOpts)
	{
		CollisionPlanes.Add(FPBDCollisionPlane(PlaneOpt.Point, PlaneOpt.Normal, PlaneOpt.FrictionFactor));
	}
}

float UPBDPhysicsComponent::GetEffectiveTemperatureFactorFromCurve(const UCurveFloat* Curve, const float Temperature)
{
	if (Curve)
	{
		return Curve->GetFloatValue(Temperature);
	}

	return 1.0;
}


void UPBDPhysicsComponent::DrawDebugShapes()
{
	//UE_LOG(LogTemp, Warning, TEXT("Num Particles / Constraints: %d / %d"), PBDParticles.Num(), PBDConstraints.Num());
	for (FPBDParticle& particle : PBDParticles)
	{
		FLinearColor Color = FMath::Lerp(FLinearColor::Black, FLinearColor(0.8f, 0, 0), FMath::Clamp(particle.Temperature / 300.0, 0.0, 1.0));
		FVector pWorld = GetOwner()->GetActorTransform().TransformPosition(particle.Position / SimulationScale);
		UKismetSystemLibrary::DrawDebugPoint(this, pWorld, 8.0, Color);

		for (auto Neighbor : particle.Neighbors)
		{
			FVector nWorld = GetOwner()->GetActorTransform().TransformPosition(Neighbor->Position / SimulationScale);
			UKismetSystemLibrary::DrawDebugLine(this, pWorld, nWorld, Color);
		}

	}

}

