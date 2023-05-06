#pragma once

#include "GLView.h"
#include "PxPhysicsAPI.h"
#include "PhysicsEngine.h"
#include "PxPhysics.h"
#include "extensions\PxDefaultErrorCallback.h"
#include "extensions\PxDefaultAllocator.h"
#include "WOTerrainPhysx.h"
#include "WODronePhysx.h"
#include "CameraChaseActorStationary.h"
#include "WOCameraSink.h"
#include "WOPhysxBox.h"
#include "WOWayPointSpherical.h"

namespace Aftr
{
    class Camera;
    class WORay;

    class GLViewFinalProjectCode : public GLView
    {
    public:
        static GLViewFinalProjectCode* New(const std::vector< std::string >& outArgs);
        virtual ~GLViewFinalProjectCode();
        virtual void updateWorld(); ///< Called once per frame
        virtual void loadMap(); ///< Called once at startup to build this module's scene
        virtual void createFinalProjectCodeWayPoints();
        virtual void castRay(WO* v);
        void castRayUp(WO* u,auto time);

        virtual void castRayAngleBack(WO* u);
        virtual void castRayAngleFront(WO* u);
        void startEngine();
        void stopEngine();
        void shoot();
        void alertSound(bool flag);
        float calculateSpeed(float v, auto start);
        float calculateCloudSpeed(std::vector<Vector> cloudPos, std::vector<int> time);
        virtual float rayMarcher(Vector point, Vector direction);
        virtual float Getdist(Vector point);
        //double hit_sphere(const point3& center, double radius, const ray& r);
        virtual void onResizeWindow(GLsizei width, GLsizei height);
        virtual void onMouseDown(const SDL_MouseButtonEvent& e);
        virtual void onMouseUp(const SDL_MouseButtonEvent& e);
        virtual void onMouseMove(const SDL_MouseMotionEvent& e);
        virtual void onKeyDown(const SDL_KeyboardEvent& key);
        virtual void onKeyUp(const SDL_KeyboardEvent& key);
              
        void captureImage();
        void flipCamera();
        WO* drone = nullptr;
        WODronePhysx* d = nullptr;
        WOWayPointSpherical* wayPt = nullptr;
        Vector terrainCollision;
        Vector cloudCollision;
        std::vector<Vector>collisionPos;
        std::vector<int>times;
        WORay* ray1;
        WORay* ray2;
        Camera* otherCamera;
        WOCameraSink* sink = nullptr;
        Vector cloud_position;
        std::vector<WO*> targets;
        std::vector<WO*> hittable;
        //virtual void collisionDetector(const PxContactPairHeader pairHeader, const PxContactPair* pairs, PxU32 nbPairs);
        physx::PxScene* s = nullptr;

        WO* targetLight;
        Vector light_normal;
        Vector lightPos;

        //Vector of scattered WO's
        std::vector<WO*>scatters;
        WORay* ray = nullptr;
        float cloud_x = 500;
        
           std::vector<float> x1 = { 20 };
           std::vector<float> y1 = { 1000 };
          
        bool dyna_cloud = false;
        bool img_render = false;
        bool move_opposite = false;
        bool static_cam = false;
    protected:
        GLViewFinalProjectCode(const std::vector< std::string >& args);
        virtual void onCreate();
        WO* grid = nullptr;
        WO* target = nullptr;
        WO* target1 = nullptr;
        WO* target2 = nullptr;
        WO* target3 = nullptr;
        WO* target4 = nullptr;
        WO* target5 = nullptr;
        WO* box = nullptr;
        double time;

    private:
        bool up=false;
        bool down=false;
        bool right = false;
        bool display = false;
        bool left = false;
        Vector gravityDirection;
        bool start = false;
        bool gravity = false;
        float x = 20.0f;
        float y = -2.0f;
        float z = 170.0f;
        float theta = 0.0f;
        float theta1 = 0.0f;
        float theta2 = 0.0f;
        float timeT = 1;

        //physx variables
        physx::PxPhysics* p = nullptr;
        physx::PxCooking* mCooking = nullptr;
        physx::PxDefaultAllocator      a;
        physx::PxDefaultErrorCallback  e;
        physx::PxDefaultCpuDispatcher* mDispatcher = nullptr;
        physx::PxTolerancesScale       mToleranceScale;
        physx::PxFoundation* f = nullptr;
        physx::PxMaterial* mMaterial = nullptr;
        physx::PxRigidDynamic* actor;
        physx::PxPvd* mPvd = nullptr;
        Vector centerOfWorld;
        bool flip = false;



        /*Variables for the second part of project, Uncommenting enables these variables implementation of commented code of GLView subclass
        WORay* ray = nullptr;
   WORay* ray1 = nullptr;
   WORay* ray2 = nullptr;
   virtual void castRay();
   virtual float rayMarcher(Vector point, Vector direction);
   virtual float Getdist(WORay* ray,Vector p);
   Vector marchPoint=Vector(0,0,0);
   float signed_distance = 0;
   virtual void rayMarch();
   WO* wo;
   WO* moon;
   int x = 10;
   int y = 10;
   int z = 10;
   std::vector<WO*> target;
   std::vector<WO*>hittable;
   WO* targetLight;
   Vector light_normal;
   Vector lightPos;
        */
    };
}


