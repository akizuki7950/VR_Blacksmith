// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "PhysicsUtils.h"
#include "PBDGrabComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class LABCOURSE_API UPBDGrabComponent : public USceneComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UPBDGrabComponent();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void DrawDebugShapes();

	UFUNCTION(BlueprintCallable)
	void Grab(USceneComponent* Grabber)
	{
		SetKinematic(true);
		if (Grabber)
		{
			GrabberToFollow = Grabber;
			FTransform GrabberInitWorldTransform = Grabber->GetComponentTransform();
			GrabberInitialOffsetQuat = Grabber->GetComponentQuat().Inverse() * GetComponentQuat();
			GrabberInitialOffsetLocation = GrabberInitWorldTransform.InverseTransformPosition(GetComponentLocation());
		}
	}

	UFUNCTION(BlueprintCallable)
	void Release()
	{
		SetKinematic(false);
		GrabberToFollow = nullptr;
	}


	void SetKinematic(bool IsKinematic)
	{
		bIsKinematic = IsKinematic;
	}

	FPBDParticle* ParticleToFollow = nullptr;
	FVector GrabberInitialOffsetLocation;
	FQuat GrabberInitialOffsetQuat;
	TObjectPtr<USceneComponent> GrabberToFollow;
	float Radius = 10.0;
	bool bIsKinematic = false;
};
