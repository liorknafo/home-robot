#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/GameModeBase.h"
#include "DriveCore.h"
#include "RobotGeometry.h"
#include "RobotPawn.generated.h"
class UBoxComponent;class USphereComponent;class UProceduralMeshComponent;class UPhysicsConstraintComponent;
class USceneCaptureComponent2D;class UTextureRenderTarget2D;class UMaterialInterface;class UPhysicalMaterial;class FSocket;class FInternetAddr;class ACameraActor;
UCLASS()
class HOMEROBOTSIM_API ARobotPawn : public APawn {
 GENERATED_BODY()
public:
 ARobotPawn();virtual void BeginPlay() override;virtual void Tick(float Dt) override;virtual void EndPlay(const EEndPlayReason::Type Reason) override;
 // Round chassis: root physics body. Its collision is one convex prism (bumper hard stop over the bumper span, rigid rear
 // shell behind it; RobotGeometry::body_prism) and, in LiDAR mode, a second convex element on the same body: the LiDAR column
 // (turret cylinder from the lid top to the robot top; RobotGeometry::lidar_column_prism). Nothing is welded into it.
 UPROPERTY() UProceduralMeshComponent* Body;
 UPROPERTY() UProceduralMeshComponent* LeftWheel;
 UPROPERTY() UProceduralMeshComponent* RightWheel;
 UPROPERTY() USphereComponent* Caster;   // the ONE rear ball caster (rear_caster_xy_m); there is no front caster
 UPROPERTY() UPhysicsConstraintComponent* LeftJoint;
 UPROPERTY() UPhysicsConstraintComponent* RightJoint;
 UPROPERTY() UPhysicsConstraintComponent* CasterJoint;
 UPROPERTY() UProceduralMeshComponent* Visual;
 UPROPERTY() USceneCaptureComponent2D* LeftCamera;
 UPROPERTY() USceneCaptureComponent2D* RightCamera;
 UPROPERTY() UTextureRenderTarget2D* LeftTarget;
 UPROPERTY() UTextureRenderTarget2D* RightTarget;
 UPROPERTY() UMaterialInterface* ColorMaterial;
 UPROPERTY() UPhysicalMaterial* TireMaterial;
 UPROPERTY() UPhysicalMaterial* CasterMaterial;
 UPROPERTY() ACameraActor* Overview;
 DriveCore Drive;
 // Geometry (cm, robot frame x fwd / y LEFT / z above floor, origin base_link = axle midpoint = shell centre). Read from
 // robot-parameters.json in BeginPlay; the RobotGeometry::Geometry defaults are only the fallback (logged when used).
 RobotGeometry::Geometry Geo;FString GeometrySource=TEXT("fallback");TSharedPtr<FJsonObject> GeometryJson; // echoed as sim_geometry
 void LoadParameters();void ApplyGeometry();void ApplyCenterOfMass(const TCHAR* When);void ConfigureDrive(); // ConfigureDrive: DriveCore track/wheel radius from Geo
 // Body inertia about the body COM from body_inertia_kg_m2 (robot frame, kg m2; scaled with -MassScale). Applied after every COM
 // nudge (ApplyCenterOfMass), because UpdateMassProperties recomputes the collider inertia. bBodyInertia false = file value
 // missing or not physical: the collider-derived inertia is kept (logged as a fallback).
 double BodyInertia[3][3]={{0,0,0},{0,0,0},{0,0,0}};bool bBodyInertia=false;
 void ApplyBodyInertia(const TCHAR* When);bool PhysicsBodyInertia(double Out[3][3]) const; // Out: robot frame, kg m2, about the physics COM
 FVector WheelOffset(bool Left) const;FVector CasterOffset() const; // Unreal component-space offsets from the body origin
 void ResetRobot(FVector FloorPosition=FVector::ZeroVector,double Heading=0);
 void NewStereoSession();void BuildRobotVisual();void PollCommands();void SendState();void CaptureStereo();void SetupJoints();
 bool TraceSensors();
 void SampleLidar(float Dt);
 void SetPreviewMode(int Mode);void UpdatePreview(float Dt);
 int PreviewMode=3;double ViewYaw=-145,ViewPitch=63,ViewDistance=1000;
 FSocket* Socket=nullptr;TSharedPtr<FInternetAddr> Peer;
 double Time=0,NextState=0,NextCapture=0,LeftTicks=0,RightTicks=0;
 double LeftRate=0,RightRate=0,LastKeyboard=0;
 TSharedPtr<FJsonObject> LidarScan;TArray<TSharedPtr<FJsonValue>> LidarRanges,LidarTimes;
 FRandomStream LidarRandom{7391};double LidarCarry=0,LidarStart=0;int LidarIndex=0,LidarFrame=0;bool LidarMode=true;
 double LidarBiasFraction=.5,LidarJitterFraction=.25,LidarDropout=.02;
 int Frame=0;bool CaptureEnabled=false,NoCaster=false,KeyboardWasDown=false,KeyboardControl=false;
 // Fallbacks only; BeginPlay overwrites them from robot-parameters.json (COM in robot frame, cm, z above the floor).
 double BodyMass=2.1,WheelMass=.035,CasterMass=.068,BodyComX=-2.0,BodyComY=0,BodyComZ=7.5;
 bool bComPending=true; // re-applied once on the first tick, after every welded shape is in the body
 double RollingTorque=.003,EncoderCPR=4741.44,CameraHz=2;
 bool Bump=false,BumpLeft=false,BumpRight=false;TArray<double> Cliff;FString FaultReason="startup",Scenario="house",DataDir;
 // Bumper skin zones (bit i = zone i between bump_zone_edges_deg[i] and [i+1], bit0 right .. bit4 left): this tick,
 // and OR-ed since the last state message. BumpColumns / BumpRows: probe columns and row heights (cm), RobotGeometry.
 uint8 BumpZones=0,BumpZonesLatched=0,LeftSwitchZones=0x1C,RightSwitchZones=0x07;
 std::vector<RobotGeometry::BumpColumn> BumpColumns;std::vector<double> BumpRows;FVector BumpBoxCenter=FVector::ZeroVector,BumpBoxExtent=FVector::ZeroVector;
 bool bBumperSparseColumns=false; // test-only negative control (-BumperSparseColumns): every 4th column, 16 deg apart
 // Per-tick bumper query cost (wall time), logged as SENSOR_COST.
 double BumperCostSum=0,BumperCostMax=0,BumperNearCostSum=0,BumperFullCostSum=0,BumperFullCostMax=0;
 int64 BumperTicks=0,BumperNearTicks=0,BumperFullTicks=0;int32 BumperFullRays=0;
 void LogSensorCost();
 FString LastFaultEvent="none",PreviousFault="startup";double LastFaultTime=-1;
 FVector PreviousVelocity=FVector::ZeroVector;FVector SpecificForce=FVector::ZeroVector;
};
UCLASS()
class HOMEROBOTSIM_API AHomeRobotGameMode : public AGameModeBase {
 GENERATED_BODY()
public:
 AHomeRobotGameMode();virtual void StartPlay() override;
};
