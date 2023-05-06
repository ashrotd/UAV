#pragma once
#include "PxPhysicsAPI.h"
#include "vehicle/PxVehicleUtil.h"
#include "vehicle/PxVehicleWheels.h"
#include "vehicle/PxVehicleDrive4W.h"
#include "PxPhysics.h"
#include "extensions\PxDefaultErrorCallback.h"
#include "extensions\PxDefaultAllocator.h"
#include "GLView.h"
#include "WO.h"


using namespace physx;
using namespace Aftr;

class WODronePhysx :public WO {
public:
	static WODronePhysx* New(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale, float x);
	virtual ~WODronePhysx();
	virtual void updatePoseFromPhysx();
	virtual void physxSetPosition(float x, float y, float z);
	virtual void addForces(float x, float y, float z);
protected:
	WODronePhysx();
	virtual void onCreate(physx::PxPhysics* p, physx::PxScene* s, const std::string& path, const Vector& scale, float x);
	PxRigidDynamic* actor = nullptr;
};