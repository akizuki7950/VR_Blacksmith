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

	FTransform ActorTransform = GetOwner()->GetActorTransform();
	// ...
}

void UPBDPhysicsComponent::Init()
{
	InitParticles();
	InitConstraints();
}

void UPBDPhysicsComponent::InitParticles()
{
	FVector ND = Dimension / Res;
	Nx = ND.Y, Ny = ND.Y, Nz = ND.Z;

	float interval = Dimension.X / Nx;

	for (int i = 0; i < Nz; i++)
	{
		for (int j = 0; j < Ny; j++)
		{
			for (int k = 0; k < Nx; k++)
			{
				FVector pos(k * interval, j * interval, i * interval);
				PBDParticles.Add(FPBDParticle(pos, 1.0));
			}
		}
	}
	//UE_LOG(LogTemp, Warning, TEXT("ND: %d %d %d"), Nx, Ny, Nz);
}

void UPBDPhysicsComponent::InitConstraints()
{
	TSet<TTuple<int32, int32>> ProcessedEdges;
	
	for (int i = 0; i < PBDParticles.Num(); i++) {

		TArray<TTuple<int32, int32, int32, int32>> TetrahedronOffsetSets;
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(0, 1, 0),  GetOffset(0, 1, 1),  GetOffset(1, 1, 1) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(0, 1, 1),  GetOffset(0, 0, 1),  GetOffset(1, 1, 1) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(0, 0, 1),  GetOffset(1, 0, 1),  GetOffset(1, 1, 1) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(1, 0, 1),  GetOffset(1, 0, 0),  GetOffset(1, 1, 1) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(1, 0, 0),  GetOffset(1, 1, 0),  GetOffset(1, 1, 1) });
		TetrahedronOffsetSets.Add({ GetOffset(0,0,0), GetOffset(1, 1, 0),  GetOffset(0, 1, 0),  GetOffset(1, 1, 1) });

		
		for (auto TOffset : TetrahedronOffsetSets)
		{
			bool IsTetraValid = true;
			/*
			for (int j = 0; j < 4; j++)
			{
				int32 Idx = TOffset.Get<j>;
				if (Idx < 0 || Idx >= PBDParticles.Num())
				{
					IsTetraValid = false;
				}
			}
			*/

			if (IsTetraValid)
			{
				
			}

		}
		

		
	}

}

void UPBDPhysicsComponent::DrawDebugShapes()
{

	for (FPBDParticle& particle : PBDParticles)
	{
		FVector pWorld = UKismetMathLibrary::TransformLocation(GetOwner()->GetActorTransform(), particle.Position);
		UKismetSystemLibrary::DrawDebugPoint(this, pWorld, 10.0, FLinearColor::Red);
	}

}

