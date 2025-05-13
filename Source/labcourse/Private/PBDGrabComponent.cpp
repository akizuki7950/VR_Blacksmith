// Fill out your copyright notice in the Description page of Project Settings.


#include "PBDGrabComponent.h"
#include "Kismet/KismetSystemLibrary.h"

// Sets default values for this component's properties
UPBDGrabComponent::UPBDGrabComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UPBDGrabComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	
}


// Called every frame
void UPBDGrabComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	FLinearColor Color = bIsKinematic ? FLinearColor(0.3f, 0.5f, 0.0f) : FLinearColor(0.1f, 0.1f, 0.9f, 0.5f);
	if (bIsKinematic)
		UKismetSystemLibrary::DrawDebugCapsule(this, GetComponentLocation(), 0.0f, Radius, FRotator::ZeroRotator, Color);
}

