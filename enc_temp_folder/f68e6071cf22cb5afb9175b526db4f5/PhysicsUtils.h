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

    FPlaneOpt() : Normal(FVector(0.0, 0.0, 1.0)), Point(FVector::ZeroVector) {};

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Normal;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Point;
};

class LABCOURSE_API FPBDParticle
{
public:
	FPBDParticle(FVector Pos, float mass) : Position(Pos), Mass(mass)
	{
        ensure(Mass > 0);
		if (Mass > 0)
        {
            InvMass = 1.0 / Mass;
        }
        else InvMass = 0;

        Velocity = FVector::ZeroVector;
        PredictedPosition = FVector::ZeroVector;
	}
	~FPBDParticle() {}

	FVector Position;
    FVector PredictedPosition;
    FVector Velocity;
    float Mass;
    float InvMass;
};

class LABCOURSE_API FPBDConstraintBase
{
public:
    virtual ~FPBDConstraintBase() {}

    virtual void Solve(TArray<FPBDParticle>& particles, float stiffness, float dt) { check(0 && "Must Override this"); }
};

class LABCOURSE_API FPBDCollisionPlane
{
public:

    FPBDCollisionPlane();
    FPBDCollisionPlane(FVector Pos, FVector Norm) : Normal(Norm.Normalize()), Point(Pos) {}
    virtual ~FPBDCollisionPlane() {};

    float GetSignedDistance(const FVector& Pos) const
    {
        float SignedDistance = FVector::DotProduct(Pos - Point, Normal);
        return SignedDistance;
    }

    FVector Normal;
    FVector Point;
    
};

class LABCOURSE_API FDistanceConstraint : public FPBDConstraintBase
{
public:
    int32 Index1; // 采ま
    int32 Index2;
    float RestLength;
    // float Stiffness; // ┪把计肚 Solve

    FDistanceConstraint(int32 idx1, int32 idx2, float restLen)
        : Index1(idx1), Index2(idx2), RestLength(restLen) {
    }
    virtual ~FDistanceConstraint() override{}

    virtual void Solve(TArray<FPBDParticle>& particles, float stiffness, float dt) override{}
};

class LABCOURSE_API FVolumeConstraint : public FPBDConstraintBase
{
public:
    int32 Index1, Index2, Index3, Index4; // 砰郴翴ま
    float RestVolume;
    // float Stiffness;

    FVolumeConstraint(int32 idx1, int32 idx2, int32 idx3, int32 idx4, float restVol)
        : Index1(idx1), Index2(idx2), Index3(idx3), Index4(idx4), RestVolume(restVol) {
    }
    virtual ~FVolumeConstraint() override {}

    virtual void Solve(TArray<FPBDParticle>& particles, float stiffness, float dt) override{}
};

// ... 临Τ FBendingConstraint 单

class LABCOURSE_API FPlaneCollisionConstraint : public FPBDConstraintBase
{
public:
    int32 Index1;
    FVector CollisionNormal;
    float PenetrationDepth;

    FPlaneCollisionConstraint() : Index1(0), CollisionNormal(FVector::ZeroVector), PenetrationDepth(0) {}
    virtual ~FPlaneCollisionConstraint() override{}

    void Set(const int32 idx1, const FVector& Normal, const float SD)
    {
        Index1 = idx1;
        CollisionNormal = Normal;
        PenetrationDepth = -SD;
    }

    virtual void Solve(TArray<FPBDParticle>& particles, float stiffness, float dt) override;
    
};