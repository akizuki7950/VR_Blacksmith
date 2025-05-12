// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PhysicsUtils.h"
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

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	void InitParticles();
	void InitConstraints();
	void InitCollisionPlanes();
	void Simulate(float dt);
	void ApplyImpulse(FVector Pos, FVector ImpactPulse);

	int32 GetOffset(int32 OffsetX, int32 OffsetY, int32 OffsetZ)
	{
		int32 OX = 1;
		int32 OY = Nx;
		int32 OZ = Nx * Ny;
		return OffsetX * OX + OffsetY * OY + OffsetZ * OZ;
	}

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	void DrawDebugShapes();

	UPROPERTY(EditAnywhere, DisplayName="Size")
	FVector Dimension = FVector(100, 100, 100);

	UPROPERTY(EditAnywhere, DisplayName="Simulation Resolution", Meta=(ClampMin="1.0"))
	float Res = 20.0;

	UPROPERTY(EditAnywhere, DisplayName = "Stiffness", Meta = (ClampMin = "0.001", ClampMax = "1.0"))
	float Stiffness = 0.3;

	UPROPERTY(EditAnywhere, DisplayName = "Collision Constraint Pool Size", Meta = (ClampMin = "64"))
	int32 CollisionConstraintPoolSize = 512;

	UPROPERTY(EditAnywhere, DisplayName = "Solver Iterations", Meta = (ClampMin = "1", ClampMax = "64"))
	int32 SolverIterations = 8;

	UPROPERTY(EditAnywhere, DisplayName = "Simulation Scale", Meta = (ClampMin = "0.001", ClampMax = "1.0"))
	float SimulationScale = 0.01;

	UPROPERTY(EditAnywhere, DisplayName="Draw Debug Shapes")
	bool bDrawDebugShapes = false;

	UPROPERTY(EditAnywhere, DisplayName="Collision Planes")
	TArray<FPlaneOpt> PlaneOpts;

private:
	TArray<FPBDConstraintBase*> PBDConstraints;
	TArray<FPlaneCollisionConstraint*> ActiveCollisionConstraints;
	TArray<FPlaneCollisionConstraint*> CollisionConstraintPool;
	TArray<FPBDParticle> PBDParticles;
	TArray<FPBDCollisionPlane> CollisionPlanes;
	int Nx, Ny, Nz;

	
	
};
