#pragma once

#include "PxPhysicsAPI.h"
#include "WO.h"

using namespace physx;
using namespace Aftr;

class PhysxPlane :public WO {
public:
	static PhysxPlane* New(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale);
	virtual ~PhysxPlane();
	virtual void updatePoseFromPhysx();
	virtual void physxSetPosition(float x, float y, float z);
	

protected:
	PhysxPlane();
	virtual void onCreate(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale);
	PxRigidActor* actor = nullptr;

};