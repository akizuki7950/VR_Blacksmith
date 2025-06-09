// Fill out your copyright notice in the Description page of Project Settings.


#include "PBDPhysicsComponent.h"

#include "Algo/RandomShuffle.h"
#include "Kismet/KismetMathLibrary.h"
#include "GeometryScript/MeshBasicEditFunctions.h"
#include "GeometryScript/MeshQueryFunctions.h"
#include "GeometryScript/MeshNormalsFunctions.h"
#include "GeometryScript/GeometryScriptTypes.h"
#include "UDynamicMesh.h"
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

	UpdateKinematics(DeltaTime);
	Simulate(DeltaTime);
	UpdateDMC();
	UpdateOwnerPos();
	UpdateGrabComponents();
	// ...
}

void UPBDPhysicsComponent::UpdateKinematics(float dt)
{
	for (auto& GC : PBDGrabComponents)
	{
		if (GC->bIsKinematic)
		{
			FTransform GrabberCurrentWorldTransform = GC->GrabberToFollow->GetComponentTransform();
			FVector TargetPos = GrabberCurrentWorldTransform.TransformPosition(GC->GrabberInitialOffsetLocation);
			FQuat TargetRotation = GrabberCurrentWorldTransform.GetRotation() * GC->GrabberInitialOffsetQuat;
			GC->SetWorldRotation(TargetRotation, false, nullptr, ETeleportType::TeleportPhysics);
			GC->SetWorldLocation(TargetPos);
		}
	}
	for (auto& Particle : PBDParticles)
	{
		if (Particle.bIsKinematic)
		{
			FTransform CurrentGCWorldTransform = Particle.GCToFollow->GetComponentTransform();
			FVector TargetWorldOffset = CurrentGCWorldTransform.TransformPosition(Particle.GCInitOffsetWorld);
			FVector TargetSimOffset = ConvertPositionWorldToSim(TargetWorldOffset);
			Particle.PredictedPosition = FMath::Lerp(Particle.PredictedPosition, TargetSimOffset, 0.3f);
			Particle.Velocity = (Particle.PredictedPosition - Particle.Position) / dt;
			Particle.Position = Particle.PredictedPosition;
		}
	}
}

void UPBDPhysicsComponent::UpdateGrabComponents()
{
	for (auto& GC : PBDGrabComponents)
	{
		if (!GC->bIsKinematic)
		{
			FVector pLocWorld = ConvertPositionSimToWorld(GC->ParticleToFollow->Position);
			GC->SetWorldLocation(pLocWorld);
		}
	}
}

void UPBDPhysicsComponent::UpdateDMC()
{
	FGeometryScriptVectorList PosList;
	PosList.Reset(SurfaceParticles.Num());
	for (auto p : SurfaceParticles)
	{
		FVector pOwner = ConvertPositionSimToOwner(p.first->Position);
		PosList.List->Add(pOwner);
	}
	UGeometryScriptLibrary_MeshBasicEditFunctions::SetAllMeshVertexPositions(DMC->GetDynamicMesh(), PosList);
}


void UPBDPhysicsComponent::TryGrab(USceneComponent* Grabber, UPBDGrabComponent* GC, bool bIsLeft)
{
	GC->Grab(Grabber);
	FVector GCLocSim = ConvertPositionWorldToSim(GC->GetComponentLocation());
	float RadiusSim = GC->Radius * SimulationScale;
	for (auto& Particle : PBDParticles)
	{
		float Dist = (Particle.Position - GCLocSim).Length();
		if (Dist <= RadiusSim)
		{
			Particle.Grab(GC, ConvertPositionSimToWorld(Particle.Position));
			if (bIsLeft)
			{
				ParticlesGrabbedLeft.Add(&Particle);
			}
			else
			{
				ParticlesGrabbedRight.Add(&Particle);
			}
		}
	}
}

void UPBDPhysicsComponent::TryRelease(USceneComponent* Grabber, UPBDGrabComponent* GC, bool bIsLeft)
{
	GC->Release();
	if (bIsLeft)
	{
		for (auto& particle : ParticlesGrabbedLeft)
		{
			particle->Release();
		}
		ParticlesGrabbedLeft.Empty();
	}
	else
	{
		for (auto& particle : ParticlesGrabbedRight)
		{
			particle->Release();
		}
		ParticlesGrabbedRight.Empty();
	}
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
		if (!Particle.bIsKinematic || true)
		{
			Particle.Velocity *= DampingFactor;
			Particle.Velocity += Gravity * dt;
			Particle.PredictedPosition = Particle.Position + Particle.Velocity * dt;
		}
	}

	//UE_LOG(LogTemp, Warning, TEXT("Num Pool: %d"), CollisionConstraintPool.Num());

	// Substep
	float step = ConstTimeStep / SolverIterations * dt;
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

		TArray<FPBDConstraintBase*> ConstraintsThisFrame;
		ConstraintsThisFrame.Append(PBDConstraints);
		ConstraintsThisFrame.Append(ActiveCollisionConstraints);

		//Algo::RandomShuffle(PBDConstraints);
		//Algo::RandomShuffle(ActiveCollisionConstraints);
		Algo::RandomShuffle(ConstraintsThisFrame);


		for (auto Constraint : ConstraintsThisFrame)
		{
			float AvgTemp = Constraint->CalculateAverageTemperature(PBDParticles);
			float EffectiveStiffnessFactor = GetEffectiveTemperatureFactorFromCurve(CachedStiffnessTemperatureFactorCurve, AvgTemp);
			float EffectiveStiffness = BaseStiffness * EffectiveStiffnessFactor;
			Constraint->Solve(PBDParticles, EffectiveStiffness, step);
		}

		/*
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
		*/
	}

	for (auto& Particle : PBDParticles)
	{
		if (!Particle.bIsKinematic || true)
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

	for (auto& Constraint : PBDConstraints)
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
	InitDMC();
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
				PBDParticles.Add(FPBDParticle(pos, 1.0, FIntVector(k, j, i)));
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
				int32 nIdxFront = Idx + GetOffset(1, 0, 0);
				int32 nIdxRight = Idx + GetOffset(0, 1, 0);
				int32 nIdxUp = Idx + GetOffset(0, 0, 1);
				int32 nIdxBack = Idx + GetOffset(-1, 0, 0);
				int32 nIdxLeft = Idx + GetOffset(0, -1, 0);
				int32 nIdxDown = Idx + GetOffset(0, 0, -1);

				if (k < Nx - 1)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdxFront]);
				}
				if (j < Ny - 1)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdxRight]);
				}
				if (i < Nz - 1)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdxUp]);
				}

				if (k > 0)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdxBack]);
				}
				if (j > 0)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdxLeft]);
				}
				if (i > 0)
				{
					PBDParticles[Idx].Neighbors.Add(&PBDParticles[nIdxDown]);
				}

				// Surface particles
				if (i == 0 || i == Nz - 1 || j == 0 || j == Ny - 1 || k == 0 || k == Nx - 1)
				{
					SurfaceParticles.Add({ &PBDParticles[Idx], Idx });
				}

				// Grab component
				if (i % GrabComponentInterval == 0 && j % GrabComponentInterval == 0 && k % GrabComponentInterval == 0)
				{
					UPBDGrabComponent* NewGC = Cast<UPBDGrabComponent>(GetOwner()->AddComponentByClass(UPBDGrabComponent::StaticClass(), false, FTransform::Identity, false));
					GetOwner()->AddInstanceComponent(NewGC);
					PBDGrabComponents.Add(NewGC);
					NewGC->ParticleToFollow = &(PBDParticles[Idx]);
					NewGC->Radius = GrabComponentRadius;
					NewGC->SetAbsolute(true, true, true);
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

		FIntVector pIdx = PBDParticles[i].nIdx;
		if (pIdx.X >= Nx - 1 || pIdx.Y >= Ny - 1 || pIdx.Z >= Nz - 1)continue;

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

void UPBDPhysicsComponent::InitDMC()
{
	DMC = Cast<UDynamicMeshComponent>(GetOwner()->GetComponentByClass(UDynamicMeshComponent::StaticClass()));

	if (DMC->IsValidLowLevel())
	{
		UDynamicMesh* DM = DMC->GetDynamicMesh();
		for (auto p : SurfaceParticles)
		{
			FVector pOwner = ConvertPositionSimToOwner(p.first->Position);
			UGeometryScriptLibrary_MeshBasicEditFunctions::AddVertexToMesh(DM, pOwner, p.first->DMIndex);

		}


		FGeometryScriptTriangleList TriangleList;
		TriangleList.Reset();

		for (auto p : SurfaceParticles)
		{
			FPBDParticle* particle = p.first;
			int32 IdxGlobal = p.second;
			int32 IdxSurface = particle->DMIndex;
			FIntVector nIdx = particle->nIdx;

			TArray<int32> ValidNeighbors;
			for (auto neighbor : particle->Neighbors)
			{
				if (IsSurfaceParticle(neighbor))
				{
					ValidNeighbors.Add(GetOffset(neighbor->nIdx));
				}
			}

			if (nIdx.X == 0  && nIdx.Y < Ny - 1 && nIdx.Z < Nz - 1)
			{
				int32 gIdxUp = IdxGlobal + GetOffset(0, 0, 1);
				int32 gIdxRight = IdxGlobal + GetOffset(0, 1, 0);
				int32 gIdxUpRight = IdxGlobal + GetOffset(0, 1, 1);

				int32 sIdx = IdxSurface;
				int32 sIdxUp = PBDParticles[gIdxUp].DMIndex;
				int32 sIdxRight = PBDParticles[gIdxRight].DMIndex;
				int32 sIdxUpRight = PBDParticles[gIdxUpRight].DMIndex;

				TriangleList.List->Add({sIdx, sIdxRight, sIdxUp});
				TriangleList.List->Add({ sIdxRight, sIdxUpRight, sIdxUp });
			}

			if (nIdx.X == Nx - 1 && nIdx.Y < Ny - 1 && nIdx.Z < Nz - 1)
			{
				int32 gIdxUp = IdxGlobal + GetOffset(0, 0, 1);
				int32 gIdxRight = IdxGlobal + GetOffset(0, 1, 0);
				int32 gIdxUpRight = IdxGlobal + GetOffset(0, 1, 1);

				int32 sIdx = IdxSurface;
				int32 sIdxUp = PBDParticles[gIdxUp].DMIndex;
				int32 sIdxRight = PBDParticles[gIdxRight].DMIndex;
				int32 sIdxUpRight = PBDParticles[gIdxUpRight].DMIndex;

				TriangleList.List->Add({ sIdx, sIdxUp, sIdxRight });
				TriangleList.List->Add({ sIdxRight, sIdxUp, sIdxUpRight });
			}

			if (nIdx.Y == 0 && nIdx.X < Nx - 1 && nIdx.Z < Nz - 1)
			{
				int32 gIdxUp = IdxGlobal + GetOffset(0, 0, 1);
				int32 gIdxFront = IdxGlobal + GetOffset(1, 0, 0);
				int32 gIdxUpFront = IdxGlobal + GetOffset(1, 0, 1);

				int32 sIdx = IdxSurface;
				int32 sIdxUp = PBDParticles[gIdxUp].DMIndex;
				int32 sIdxFront = PBDParticles[gIdxFront].DMIndex;
				int32 sIdxUpFront = PBDParticles[gIdxUpFront].DMIndex;

				TriangleList.List->Add({ sIdx, sIdxUp, sIdxUpFront });
				TriangleList.List->Add({ sIdx, sIdxUpFront, sIdxFront });
			}

			if (nIdx.Y == Ny - 1 && nIdx.X < Nx - 1 && nIdx.Z < Nz - 1)
			{
				int32 gIdxUp = IdxGlobal + GetOffset(0, 0, 1);
				int32 gIdxFront = IdxGlobal + GetOffset(1, 0, 0);
				int32 gIdxUpFront = IdxGlobal + GetOffset(1, 0, 1);

				int32 sIdx = IdxSurface;
				int32 sIdxUp = PBDParticles[gIdxUp].DMIndex;
				int32 sIdxFront = PBDParticles[gIdxFront].DMIndex;
				int32 sIdxUpFront = PBDParticles[gIdxUpFront].DMIndex;

				TriangleList.List->Add({ sIdx, sIdxUpFront, sIdxUp });
				TriangleList.List->Add({ sIdx, sIdxFront, sIdxUpFront });
			}

			if (nIdx.Z == 0 && nIdx.X < Nx - 1 && nIdx.Y < Ny - 1)
			{
				int32 gIdxRight = IdxGlobal + GetOffset(0, 1, 0);
				int32 gIdxFront = IdxGlobal + GetOffset(1, 0, 0);
				int32 gIdxRightFront = IdxGlobal + GetOffset(1, 1, 0);

				int32 sIdx = IdxSurface;
				int32 sIdxRight = PBDParticles[gIdxRight].DMIndex;
				int32 sIdxFront = PBDParticles[gIdxFront].DMIndex;
				int32 sIdxRightFront = PBDParticles[gIdxRightFront].DMIndex;

				TriangleList.List->Add({ sIdx, sIdxFront, sIdxRightFront });
				TriangleList.List->Add({ sIdx, sIdxRightFront, sIdxRight });
			}

			if (nIdx.Z == Nz - 1 && nIdx.X < Nx - 1 && nIdx.Y < Ny - 1)
			{
				int32 gIdxRight = IdxGlobal + GetOffset(0, 1, 0);
				int32 gIdxFront = IdxGlobal + GetOffset(1, 0, 0);
				int32 gIdxRightFront = IdxGlobal + GetOffset(1, 1, 0);

				int32 sIdx = IdxSurface;
				int32 sIdxRight = PBDParticles[gIdxRight].DMIndex;
				int32 sIdxFront = PBDParticles[gIdxFront].DMIndex;
				int32 sIdxRightFront = PBDParticles[gIdxRightFront].DMIndex;

				TriangleList.List->Add({ sIdx, sIdxRightFront, sIdxFront });
				TriangleList.List->Add({ sIdx, sIdxRight, sIdxRightFront });
			}



			//UE_LOG(LogTemp, Warning, TEXT("Num neighbors: %d"), particle->Neighbors.Num());
			/*
			if (particle->Neighbors.Num() >= 5 )
			{
				// Face surface
				for (int i = 0; i < ValidNeighbors.Num(); i++)
				{
					for (int j = i + 1; j < ValidNeighbors.Num(); j++)
					{
						int32 idx1 = ValidNeighbors[i];
						int32 idx2 = ValidNeighbors[j];

						if ((idx1 == IdxRight && idx2 == IdxLeft) || (idx2 == IdxRight && idx1 == IdxLeft))continue;
						if ((idx1 == IdxUp && idx2 == IdxDown) || (idx2 == IdxUp && idx1 == IdxDown))continue;

						

						int32 idxSurface0 = IdxSurface;
						int32 idxSurface1 = PBDParticles[idx1].DMIndex;
						int32 idxSurface2 = PBDParticles[idx2].DMIndex;

						TriangleList.List->Add(FIntVector(idxSurface0, idxSurface1, idxSurface2));

					}
				}
			}
			else if (ValidNeighbors.Num() >= 4)
			{
				// Edge surface
			}
			else if (ValidNeighbors.Num() >= 3)
			{
				// Corner surface
			}
			*/

		}

		FGeometryScriptIndexList NewIndicesList;
		UGeometryScriptLibrary_MeshBasicEditFunctions::AddTrianglesToMesh(DM, TriangleList, NewIndicesList);

		DMC->NotifyMeshVertexAttributesModified();
		DMC->NotifyMeshModified();
		DMC->NotifyMeshUpdated();

		int32 VCount = UGeometryScriptLibrary_MeshQueryFunctions::GetVertexCount(DM);
		int32 FCount = UGeometryScriptLibrary_MeshQueryFunctions::GetNumTriangleIDs(DM);
		UE_LOG(LogTemp, Warning, TEXT("Vertex Count: %d, Triangle Count: %d"), VCount, FCount);
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
		FLinearColor Color = FMath::Lerp(FLinearColor::Black, FLinearColor(0.8f, 0, 0), FMath::Clamp(particle.Temperature / 50.0, 0.0, 1.0));
		if (particle.bIsKinematic)Color = FLinearColor::Green;
		FVector pWorld = GetOwner()->GetActorTransform().TransformPosition(particle.Position / SimulationScale);
		UKismetSystemLibrary::DrawDebugPoint(this, pWorld, 5.0, Color);

		for (auto Neighbor : particle.Neighbors)
		{
			FVector nWorld = GetOwner()->GetActorTransform().TransformPosition(Neighbor->Position / SimulationScale);
			UKismetSystemLibrary::DrawDebugLine(this, pWorld, nWorld, Color);
		}

	}

}

