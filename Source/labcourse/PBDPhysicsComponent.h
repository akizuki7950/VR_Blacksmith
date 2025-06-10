// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/DynamicMeshComponent.h"
#include "PhysicsUtils.h"
#include "PBDGrabComponent.h"
#include "PBDPhysicsComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class LABCOURSE_API UPBDPhysicsComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UPBDPhysicsComponent();

	UFUNCTION(BlueprintCallable, DisplayName="Init PBD")
	void Init();

	UFUNCTION(BlueprintCallable, DisplayName = "Apply Impulse")
	void ApplyImpulse(FVector Position, FVector Velocity);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	void InitParticles();
	void InitConstraints();
	void InitCollisionPlanes();
	void InitDMC();
	void UpdateKinematics(float dt);
	void UpdateGrabComponents();
	void Simulate(float dt);
	void UpdateOwnerPos();
	void UpdatePlasticity();
	void UpdateDMC();
	static float GetEffectiveTemperatureFactorFromCurve(const UCurveFloat* Curve, const float Temperature);

	FVector ConvertVectorSimToWorld(const FVector& vec) const
	{
		FTransform TOwnerWorld = GetOwner()->GetActorTransform();
		return TOwnerWorld.TransformVectorNoScale(vec / SimulationScale);
	}
	FVector ConvertPositionSimToWorld(const FVector& pos) const
	{
		FTransform TOwnerWorld = GetOwner()->GetActorTransform();
		return TOwnerWorld.TransformPosition(pos / SimulationScale);
	}

	FVector ConvertPositionSimToOwner(const FVector& pos) const
	{
		return pos / SimulationScale;
	}

	FVector ConvertVectorWorldToSim(const FVector& vec) const
	{
		FTransform TOwnerWorld = GetOwner()->GetActorTransform();
		return TOwnerWorld.InverseTransformVectorNoScale(vec) * SimulationScale;
	}
	FVector ConvertPositionWorldToSim(const FVector& pos) const
	{
		FTransform TOwnerWorld = GetOwner()->GetActorTransform();
		return TOwnerWorld.InverseTransformPosition(pos) * SimulationScale;
	}

	int32 GetOffset(int32 OffsetX, int32 OffsetY, int32 OffsetZ) const
	{
		int32 OX = 1;
		int32 OY = Nx;
		int32 OZ = Nx * Ny;
		return OffsetX * OX + OffsetY * OY + OffsetZ * OZ;
	}

	int32 GetOffset(FIntVector Offset) const
	{
		int32 OX = 1;
		int32 OY = Nx;
		int32 OZ = Nx * Ny;
		return Offset.X * OX + Offset.Y * OY + Offset.Z * OZ;
	}

	bool IsSurfaceParticle(FPBDParticle* Particle) const
	{
		FIntVector nIdx = Particle->nIdx;

		if (nIdx.X == 0 || nIdx.X == Nx - 1)return true;
		if (nIdx.Y == 0 || nIdx.Y == Ny - 1)return true;
		if (nIdx.Z == 0 || nIdx.Z == Nz - 1)return true;
		return false;
	}

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	void DrawDebugShapes();

	UFUNCTION(BlueprintCallable)
	void TryGrab(USceneComponent* Grabber, UPBDGrabComponent* GC, bool bIsLeft);

	UFUNCTION(BlueprintCallable)
	void TryRelease(USceneComponent* Grabber, UPBDGrabComponent* GC, bool bIsLeft);

	//UFUNCTION(BlueprintCallable)
	//void ApplyImpulse(FVector Pos, FVector ImpactPulse);

	UPROPERTY(BlueprintReadOnly, EditAnywhere, DisplayName="Size", Category="Simulation Settings")
	FVector Dimension = FVector(100, 100, 100);

	UPROPERTY(BlueprintReadOnly, EditAnywhere, DisplayName="Simulation Resolution", Category = "Simulation Settings", Meta=(ClampMin="1.0"))
	float Res = 20.0;

	UPROPERTY(EditAnywhere, DisplayName = "Base Stiffness", Category = "Simulation Settings", Meta = (ClampMin = "0.001", ClampMax = "1.0"))
	float BaseStiffness = 0.3;

	UPROPERTY(EditAnywhere, DisplayName = "Yield Factor", Category = "Simulation Settings", Meta = (ClampMin = "0.001", ClampMax = "1.0"))
	float YieldFactor = 0.1;

	UPROPERTY(EditAnywhere, DisplayName = "Damping Factor", Category = "Simulation Settings", Meta = (ClampMin = "0.9", ClampMax = "1.0"))
	float DampingFactor = 0.995f;

	UPROPERTY(EditAnywhere, DisplayName = "Collision Constraint Pool Size", Category = "Simulation Settings", Meta = (ClampMin = "64"))
	int32 CollisionConstraintPoolSize = 768;

	UPROPERTY(EditAnywhere, DisplayName = "Solver Iterations", Category = "Simulation Settings", Meta = (ClampMin = "1", ClampMax = "64"))
	int32 SolverIterations = 8;

	UPROPERTY(EditAnywhere, DisplayName = "Simulation Scale", Category = "Simulation Settings", Meta = (ClampMin = "0.001", ClampMax = "1.0"))
	float SimulationScale = 0.01;

	UPROPERTY(EditAnywhere, Category = "Debug Settings", DisplayName="Draw Debug Shapes")
	bool bDrawDebugShapes = false;

	UPROPERTY(EditAnywhere, DisplayName="Collision Planes", Category = "Simulation Settings")
	TArray<FPlaneOpt> PlaneOpts;

	UPROPERTY(EditAnywhere, DisplayName = "Stiffness Temperature Factor Curve", Category = "Simulation Settings")
	TSoftObjectPtr<UCurveFloat> StiffnessTemperatureFactorCurve;

	UPROPERTY(EditAnywhere, DisplayName = "Yield Temperature Factor Curve", Category = "Simulation Settings")
	TSoftObjectPtr<UCurveFloat> YieldTemperatureFactorCurve;

	UPROPERTY()
	UCurveFloat* CachedStiffnessTemperatureFactorCurve;

	UPROPERTY()
	UCurveFloat* CachedYieldTemperatureFactorCurve;



	UPROPERTY(EditAnywhere, DisplayName = "Grab Component Interval", Category = "Grab Settings")
	int32 GrabComponentInterval = 2;

	UPROPERTY(EditAnywhere, DisplayName = "Grab Component Radius", Category = "Grab Settings")
	float GrabComponentRadius = 20.0f;

	UPROPERTY(BlueprintReadWrite)
	TObjectPtr<UDynamicMeshComponent> DMC= nullptr;

private:
	TArray<FPBDConstraintBase*> PBDConstraints;
	TArray<FPlaneCollisionConstraint*> ActiveCollisionConstraints;
	TArray<FPlaneCollisionConstraint*> CollisionConstraintPool;
	TArray<FPBDParticle> PBDParticles;
	TArray<std::pair<FPBDParticle*, int32>> SurfaceParticles;
	TArray<FPBDCollisionPlane> CollisionPlanes;
	TArray<TObjectPtr<UPBDGrabComponent>> PBDGrabComponents;
	TArray<FPBDParticle*> ParticlesGrabbedLeft;
	TArray<FPBDParticle*> ParticlesGrabbedRight;
	int Nx, Ny, Nz;
	float ConstTimeStep = 1.0f / 90.0f;

	
	
};
