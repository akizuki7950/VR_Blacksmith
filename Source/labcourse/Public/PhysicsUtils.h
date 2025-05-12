// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Chaos/PBDParticles.h"
#include "PhysicsUtils.generated.h"

/**
 * 
 */

inline float GetTetVolume(const FVector& V1, const FVector& V2, const FVector& V3, const FVector& V4)
{
    const FVector v1v2 = V2 - V1;
    const FVector v1v3 = V3 - V1;
    const FVector v1v4 = V4 - V1;

    return FVector::DotProduct(FVector::CrossProduct(v1v2, v1v3), v1v4) / 6.0;
}

USTRUCT(Blueprintable, BlueprintType)
struct FPlaneOpt
{
    GENERATED_BODY()
public:

    FPlaneOpt() : Normal(FVector(0.0, 0.0, 1.0)), Point(FVector::ZeroVector), FrictionFactor(0.5) {};

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Normal;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Point;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float FrictionFactor;
};

class LABCOURSE_API FPBDParticle
{
public:
	FPBDParticle(FVector Pos, float mass, bool Kinematic = false) : Position(Pos), Mass(mass), bIsKinematic(Kinematic)
	{
        ensure(Mass > 0);
		if (Mass > 0 && !bIsKinematic)
        {
            InvMass = 1.0 / Mass;
        }
        else InvMass = 0;

        Velocity = FVector::ZeroVector;
        PredictedPosition = Pos;
        Temperature = 300.0;
	}
	~FPBDParticle() {}

	FVector Position;
    FVector PredictedPosition;
    FVector Velocity;
    float Mass;
    float InvMass;
    float Temperature;
    bool bIsKinematic;
    TArray<FPBDParticle*> Neighbors;
};

class LABCOURSE_API FPBDConstraintBase
{
public:
    virtual ~FPBDConstraintBase() {}

    virtual void Solve(TArray<FPBDParticle>& particles, float stiffness, float dt) { check(0 && "Must Override this"); }
    virtual float CalculateAverageTemperature(TArray<FPBDParticle>& particles) { check(0 && "Must Override this"); return 0; }
    virtual float CalculateCurrentStrain(TArray<FPBDParticle>& particles) { check(0 && "Must Override this"); return 0; }
    virtual void UpdateConstraintPlasticity(TArray<FPBDParticle>& particles, float YieldFactor) { check(0 && "Must Override this"); }

    float BaseYieldStrain;
};

class LABCOURSE_API FPBDCollisionPlane
{
public:

    FPBDCollisionPlane();
    FPBDCollisionPlane(FVector Pos, FVector Norm, float Friction) : Normal(Norm.GetSafeNormal()), Point(Pos), FrictionFactor(Friction) {}
    FPBDCollisionPlane(FVector Pos, FVector Norm) : Normal(Norm.GetSafeNormal()), Point(Pos), FrictionFactor(0.5) {}
    virtual ~FPBDCollisionPlane() {};

    float GetSignedDistance(const FVector& Pos) const
    {
        float SignedDistance = FVector::DotProduct(Pos - Point, Normal);
        return SignedDistance;
    }

    FVector Normal;
    FVector Point;
    float FrictionFactor;
};

class LABCOURSE_API FDistanceConstraint : public FPBDConstraintBase
{
public:
    int32 Index1;
    int32 Index2;
    float RestLength;
    float InitRestLength;
    float AccumulatedPlasticStrain;

    FDistanceConstraint(int32 idx1, int32 idx2, float restLen, float YieldFactor = 0.1)
        : Index1(idx1), Index2(idx2), RestLength(restLen) {
        BaseYieldStrain = restLen * YieldFactor;
        InitRestLength = RestLength;
        AccumulatedPlasticStrain = 0.0f;
    }
    virtual ~FDistanceConstraint() override{}

    virtual void Solve(TArray<FPBDParticle>& particles, float stiffness, float dt) override;
    virtual float CalculateAverageTemperature(TArray<FPBDParticle>& particles) override;
    virtual float CalculateCurrentStrain(TArray<FPBDParticle>& particles) override;
    virtual void UpdateConstraintPlasticity(TArray<FPBDParticle>& particles, float YieldFactor) override;
};

class LABCOURSE_API FVolumeConstraint : public FPBDConstraintBase
{
public:
    int32 Index1, Index2, Index3, Index4;
    float RestVolume;
    float CurrentVolume;
    // float Stiffness;

    FVolumeConstraint(int32 idx1, int32 idx2, int32 idx3, int32 idx4, float restVol)
        : Index1(idx1), Index2(idx2), Index3(idx3), Index4(idx4), RestVolume(restVol)
	{
        if (RestVolume < 0)
        {
            // Make sure the vertices are in correct order
            int32 tmp = Index4;
            Index4 = Index3;
            Index3 = tmp;
            RestVolume = -RestVolume;
            CurrentVolume = RestVolume;
        }

    }
    virtual ~FVolumeConstraint() override {}

    virtual void Solve(TArray<FPBDParticle>& particles, float stiffness, float dt) override;
    virtual float CalculateAverageTemperature(TArray<FPBDParticle>& particles) override;
    virtual float CalculateCurrentStrain(TArray<FPBDParticle>& particles) override;
    virtual void UpdateConstraintPlasticity(TArray<FPBDParticle>& particles, float YieldFactor) override {};
};

// ... 可能還有 FBendingConstraint 等

class LABCOURSE_API FPlaneCollisionConstraint : public FPBDConstraintBase
{
public:
    int32 Index1;
    FVector CollisionNormal;
    float PenetrationDepth;
    float DampingFactor;

    FPlaneCollisionConstraint() : Index1(0), CollisionNormal(FVector::ZeroVector), PenetrationDepth(0) {}
    virtual ~FPlaneCollisionConstraint() override{}

    void Set(const int32 idx1, const FVector& Normal, const float SD, const float Friction)
    {
        Index1 = idx1;
        CollisionNormal = Normal;
        PenetrationDepth = -SD;
        DampingFactor = FMath::Clamp(1.0 - Friction, 0.0, 1.0);
    }

    virtual void Solve(TArray<FPBDParticle>& particles, float stiffness, float dt) override;
    virtual float CalculateAverageTemperature(TArray<FPBDParticle>& particles) override { return 0; }
    virtual float CalculateCurrentStrain(TArray<FPBDParticle>& particles) override { return 0;  }
    virtual void UpdateConstraintPlasticity(TArray<FPBDParticle>& particles, float YieldFactor) override {};
    
};