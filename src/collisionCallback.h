#pragma once
#include <iostream>
#include <conio.h>
#include "PxPhysicsAPI.h"

using namespace physx;

class collisionCallback :public PxSimulationEventCallback {

public:
    void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override {
        if (pairHeader.actors[0] == m_object1 || pairHeader.actors[1] == m_object1)
        {
            for (PxU32 i = 0; i < nbPairs; i++)
            {
                if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
                {
                    std::cout << "Collision detected! HurrAYYYYYYY !!!!!!! CONGRATULATIONSSS" << std::endl;
                    return;
                }
            }
        }
    }

    void onConstraintBreak(PxConstraintInfo*, PxU32) override {}
    void onWake(PxActor**, PxU32) override {}
    void onSleep(PxActor**, PxU32) override {}

    void setObjects(PxActor* object1, PxActor* object2)
    {
        m_object1 = object1;
        m_object2 = object2;
    }

private:
    PxActor* m_object1;
    PxActor* m_object2;
};
