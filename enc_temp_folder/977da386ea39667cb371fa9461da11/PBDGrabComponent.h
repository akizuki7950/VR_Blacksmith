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

	UFUNCTION(BlueprintCallable)
	void Grab(USceneComponent* Grabber)
	{
		SetKinematic(true);
		if (Grabber)
		{
			GrabberToFollow = Grabber;
			FTransform GrabberInitWorldTransform = Grabber->GetComponentTransform();
			testOffset = GrabberInitWorldTransform.InverseTransformPosition(GetComponentLocation());
			GrabberInitialRelativeTransform = GrabberInitWorldTransform.Inverse() * GetComponentTransform();
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
	FTransform GrabberInitialRelativeTransform;
	FVector testOffset;
	TObjectPtr<USceneComponent> GrabberToFollow;
	float Radius = 10.0;
	bool bIsKinematic = false;
};
