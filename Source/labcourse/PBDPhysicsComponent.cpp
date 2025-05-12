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
	// ...
	
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

	Simulate(DeltaTime);

	// ...
}

void UPBDPhysicsComponent::Simulate(float dt)
{
	FTransform ActorTransform = GetOwner()->GetActorTransform();
	FVector Gravity(0, 0, -980);
	Gravity *= SimulationScale;

	// Calc predicted positions
	for(auto& Particle : PBDParticles)
	{
		Particle.Velocity += Gravity * dt;
		Particle.PredictedPosition = Particle.Position + Particle.Velocity * dt;
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
				FTransform TOwnerWorld = GetOwner()->GetActorTransform();
				FTransform TOwnerWorldInv = TOwnerWorld.Inverse();

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
			Constraint->Solve(PBDParticles, Stiffness, step);
		}

		// Solve temporary collision constraints
		for (auto TempConstraint : ActiveCollisionConstraints)
		{
			TempConstraint->Solve(PBDParticles, 1.0, step);
		}
	}

	for (auto& Particle : PBDParticles)
	{
		Particle.Velocity = (Particle.PredictedPosition - Particle.Position) / dt;
		Particle.Position = Particle.PredictedPosition;
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
	Nx = ND.X, Ny = ND.Y, Nz = ND.Z;

	float interval = Dimension.X / Nx;

	for (int i = 0; i < Nz; i++)
	{
		for (int j = 0; j < Ny; j++)
		{
			for (int k = 0; k < Nx; k++)
			{
				FVector pos(k * interval, j * interval, i * interval);
				pos *= SimulationScale;
				PBDParticles.Add(FPBDParticle(pos, 1.0));
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
							FDistanceConstraint* NewConstr = new FDistanceConstraint(Tet[j], Tet[k], Dist);
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


void UPBDPhysicsComponent::DrawDebugShapes()
{
	//UE_LOG(LogTemp, Warning, TEXT("Num Particles / Constraints: %d / %d"), PBDParticles.Num(), PBDConstraints.Num());
	for (FPBDParticle& particle : PBDParticles)
	{
		FColor Color = FColor::MakeRedToGreenColorFromScalar(1.0 - FMath::Clamp(particle.Temperature / 300.0, 0.0, 1.0));
		Color = FColor::MakeFromColorTemperature(particle.Temperature);
		FVector pWorld = GetOwner()->GetActorTransform().TransformPosition(particle.Position / SimulationScale);
		UKismetSystemLibrary::DrawDebugPoint(this, pWorld, 8.0, Color);

		for (auto Neighbor : particle.Neighbors)
		{
			FVector nWorld = GetOwner()->GetActorTransform().TransformPosition(Neighbor->Position / SimulationScale);
			UKismetSystemLibrary::DrawDebugLine(this, pWorld, nWorld, Color);
		}

	}

}

