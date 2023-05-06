#pragma once

#include "PxPhysicsAPI.h"
#include "WO.h"

using namespace physx;
using namespace Aftr;

class WOPhysxBox :public WO {
public:
	static WOPhysxBox* New(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale, float x, float y, float z);
	virtual ~WOPhysxBox();
	virtual void updatePoseFromPhysx();
	virtual void physxSetPosition(float x, float y, float z);
	virtual void addForces(float x, float y, float z);


protected:
	WOPhysxBox();
	virtual void onCreate(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale, float x, float y, float z);
	PxRigidDynamic* actor = nullptr;

};