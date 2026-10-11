#include "RobotPawn.h"
#include "D200Model.h"
#include "HouseScene.h"
#include "Components/BoxComponent.h"
#include "Components/SphereComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "ProceduralMeshComponent.h"
#include "KismetProceduralMeshLibrary.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstance.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Common/UdpSocketBuilder.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "ImageUtils.h"
#include "InputCoreTypes.h"

static constexpr double BodyHeight=8.5; // physics body origin above the floor (cm); ground truth reports base_link at floor level
// ---- Round robot, front bumper skin about base_link ----
// Robot frame used below: x forward, y LEFT (REP-103), z up from the floor, origin = base_link = centre of the round shell =
// drive-axle midpoint (cm). Unreal component space has Y pointing RIGHT, so a robot point (x, y_left, z) is
// FVector(x, -y_left, z-BodyHeight) (ToUnreal). Azimuths are measured about base_link: az = atan2(y_left, x) in degrees,
// 0 = straight ahead, + = LEFT, - = RIGHT. The sensing skin (body_radius_m) covers -bumper_span_deg..+bumper_span_deg; the rigid
// rear shell behind it is unsensed. Geometry comes from robot-parameters.json (ARobotPawn::LoadParameters).
namespace {
// Probe row heights come from the loaded geometry (RobotGeometry::bump_rows: lowest row 0.2 mm above shell_bottom_z_m).
constexpr double BumpInsideCm=1.5,BumpOutsideCm=.2,BumpMaxNormalZ=.7;
constexpr double LegacyStereoHalfBaselineCm=3.1; // -StereoMode only (legacy 62 mm pair; not part of the round-robot CAD)
FVector ToUnreal(double X,double YLeft,double Z){return FVector(X,-YLeft,Z-BodyHeight);}
// Reads a JSON array of numbers; false (and Out empty) if the key is missing or any entry is not a number.
bool ReadNumbers(const TSharedPtr<FJsonObject>& O,const TCHAR* Key,TArray<double>& Out){
 const TArray<TSharedPtr<FJsonValue>>* A=nullptr;Out.Reset();
 if(!O.IsValid()||!O->TryGetArrayField(Key,A)||!A)return false;
 for(const TSharedPtr<FJsonValue>& V:*A){double D=0;if(!V.IsValid()||!V->TryGetNumber(D)||!FMath::IsFinite(D)){Out.Reset();return false;}Out.Add(D);}
 return true;
}
}
static TSharedPtr<FJsonObject> ReadObject(const FString& Path) {
 FString Text;TSharedPtr<FJsonObject> Obj;
 if(FFileHelper::LoadFileToString(Text,*Path))FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Obj);
 return Obj;
}
static void SaveObject(const TSharedPtr<FJsonObject>& Obj,const FString& Path) {
 FString Text;FJsonSerializer::Serialize(Obj.ToSharedRef(),TJsonWriterFactory<>::Create(&Text));FFileHelper::SaveStringToFile(Text,*Path);
}
static TArray<TSharedPtr<FJsonValue>> JVec(FVector V) {return {MakeShared<FJsonValueNumber>(V.X),MakeShared<FJsonValueNumber>(V.Y),MakeShared<FJsonValueNumber>(V.Z)};}
static void SetupBody(UPrimitiveComponent* P,float Mass,UPhysicalMaterial* Material) {
 P->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);P->SetCollisionObjectType(ECC_PhysicsBody);P->SetCollisionResponseToAllChannels(ECR_Block);
 P->SetPhysMaterialOverride(Material);P->BodyInstance.SetOverrideIterationCounts(true);P->BodyInstance.SetPositionSolverIterationCount(16);P->BodyInstance.SetVelocitySolverIterationCount(8);
 P->BodyInstance.SetMaxDepenetrationVelocity(100);P->SetLinearDamping(.02);P->SetAngularDamping(.02);P->SetSimulatePhysics(true);P->SetMassOverrideInKg(NAME_None,Mass,true);P->SetUseCCD(true);
}
FVector ARobotPawn::WheelOffset(bool Left) const {return ToUnreal(Geo.axle_x,Left?Geo.track_half:-Geo.track_half,Geo.wheel_radius);}
FVector ARobotPawn::CasterOffset() const {return ToUnreal(Geo.rear_caster_x,Geo.rear_caster_y,Geo.caster_radius+Geo.caster_lift);}
// DriveCore uses the loaded wheel geometry (wheel_track_m, wheel_radius_m) for the wheel setpoints and the pressed-reverse
// limit |w| <= 2|v|/track; its compiled .228/.04 are only the defaults. Called after LoadParameters and on every reset.
void ARobotPawn::ConfigureDrive(){Drive.track=Geo.track_half*2/100;Drive.wheel_radius=Geo.wheel_radius/100;}
ARobotPawn::ARobotPawn() {
 PrimaryActorTick.bCanEverTick=true;PrimaryActorTick.TickGroup=TG_PrePhysics;
 // Root physics body. Its only collision, a convex prism circumscribing the rigid shell, is built by ApplyGeometry at BeginPlay
 // (from robot-parameters.json); the constructor places parts with the RobotGeometry fallbacks.
 Body=CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("PhysicalChassis"));SetRootComponent(Body);Body->bUseComplexAsSimpleCollision=false;
 // The LiDAR has no component of its own: its collider is the LiDAR column, a second convex element of Body (ApplyGeometry).
 // (The compiled 9.6 x 6.0 x 3.9 cm LidarHousing box, welded as a 1 g proxy and reaching z 16.03 cm above the 15.5 cm robot
 // top, was removed in bay B10.)
 Visual=CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("CADAssembly"));Visual->SetupAttachment(Body);Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
 LeftWheel=CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("LeftWheel"));LeftWheel->SetupAttachment(Body);LeftWheel->SetRelativeLocation(WheelOffset(true));LeftWheel->bUseComplexAsSimpleCollision=false;
 RightWheel=CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("RightWheel"));RightWheel->SetupAttachment(Body);RightWheel->SetRelativeLocation(WheelOffset(false));RightWheel->bUseComplexAsSimpleCollision=false;
 Caster=CreateDefaultSubobject<USphereComponent>(TEXT("RearBallCaster"));Caster->SetupAttachment(Body);Caster->SetSphereRadius(Geo.caster_radius);Caster->SetRelativeLocation(CasterOffset());
 LeftJoint=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LeftAxle"));LeftJoint->SetupAttachment(Body);
 RightJoint=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RightAxle"));RightJoint->SetupAttachment(Body);
 CasterJoint=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("CasterBearing"));CasterJoint->SetupAttachment(Body);
 LeftCamera=CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("StereoLeft"));LeftCamera->SetupAttachment(Body);LeftCamera->SetRelativeLocation(FVector(12.272,-3.1,15-BodyHeight));
 RightCamera=CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("StereoRight"));RightCamera->SetupAttachment(Body);RightCamera->SetRelativeLocation(FVector(12.272,3.1,15-BodyHeight));
 for(auto* C:{LeftCamera,RightCamera}){C->FOVAngle=80;C->bCaptureEveryFrame=false;C->bAlwaysPersistRenderingState=true;C->bCaptureOnMovement=false;C->CaptureSource=ESceneCaptureSource::SCS_FinalColorLDR;C->bOverride_CustomNearClippingPlane=true;C->CustomNearClippingPlane=.5;C->PostProcessSettings.bOverride_MotionBlurAmount=true;C->PostProcessSettings.MotionBlurAmount=0;}
 AutoPossessPlayer=EAutoReceiveInput::Player0;
}
// Geometry, masses and COM from Content/RobotData/robot-parameters.json (written by Scripts/sync_robot_data.py from the CAD).
// Every key falls back to the compiled default when missing or malformed; the fallbacks are listed in the log.
void ARobotPawn::LoadParameters() {
 const FString Path=FPaths::ProjectContentDir()/TEXT("RobotData/robot-parameters.json");
 const TSharedPtr<FJsonObject> Params=ReadObject(Path);
 if(!Params.IsValid()){UE_LOG(LogTemp,Error,TEXT("ROBOT GEOMETRY: %s missing or unreadable; every geometry/mass value uses the compiled fallback"),*Path);GeometrySource=TEXT("fallback (no robot-parameters.json)");return;}
 TArray<FString> Fallback;
 // A file the sync has not rewritten still holds the box robot's masses and COM (old chassis frame): say so in the log,
 // in GeometrySource (ROBOT READY line) and in sim_geometry.source, so every consumer sees it.
 FString SyncStatus;const bool PreSync=Params->TryGetStringField(TEXT("sync_status"),SyncStatus);
 if(PreSync)UE_LOG(LogTemp,Error,TEXT("ROBOT GEOMETRY: robot-parameters.json is NOT synced from the round-robot CAD (sync_status: %s); masses and COM are stale. Run Scripts/sync_robot_data.py."),*SyncStatus);
 auto Num=[&Params,&Fallback](const TCHAR* Key,double& Out,double Scale,bool Required){double V=0;if(Params->TryGetNumberField(Key,V)&&FMath::IsFinite(V))Out=V*Scale;else if(Required)Fallback.Add(Key);};
 auto Vec=[&Params,&Fallback](const TCHAR* Key,int32 Count,TArray<double>& Out,bool Required){const bool Ok=ReadNumbers(Params,Key,Out)&&Out.Num()>=Count;if(!Ok&&Required)Fallback.Add(Key);return Ok;};
 // Masses and COM (robot frame; BodyComY is stored in Unreal sign, y right).
 Num(TEXT("body_mass_kg"),BodyMass,1,true);Num(TEXT("wheel_mass_kg"),WheelMass,1,true);Num(TEXT("caster_ball_mass_kg"),CasterMass,1,true);
 Num(TEXT("body_com_x_cm"),BodyComX,1,true);Num(TEXT("body_com_z_cm"),BodyComZ,1,true);Num(TEXT("encoder_cpr"),EncoderCPR,1,false);
 TArray<double> V;
 if(Vec(TEXT("estimated_body_com_mm"),3,V,true)){BodyComY=-V[1]/10;}
 // Geometry (metres in the file, cm here).
 Num(TEXT("drive_axle_x_m"),Geo.axle_x,100,true);
 double Track=Geo.track_half*2;Num(TEXT("wheel_track_m"),Track,100,true);Geo.track_half=Track/2;
 Num(TEXT("wheel_radius_m"),Geo.wheel_radius,100,true);
 Num(TEXT("body_radius_m"),Geo.skin_radius,100,true);Num(TEXT("rigid_radius_m"),Geo.rigid_radius,100,true);
 Num(TEXT("bumper_span_deg"),Geo.bumper_span_deg,1,true);Num(TEXT("bumper_travel_m"),Geo.bumper_travel,100,true);
 if(Vec(TEXT("bump_zone_edges_deg"),2,V,true))Geo.zone_edges_deg.assign(V.GetData(),V.GetData()+V.Num());
 Geo.zone_edges_deg=RobotGeometry::normalise_edges(Geo.zone_edges_deg);
 if(Vec(TEXT("rear_caster_xy_m"),2,V,true)){Geo.rear_caster_x=V[0]*100;Geo.rear_caster_y=V[1]*100;}
 else{double CX=Geo.rear_caster_x/100;if(Params->TryGetNumberField(TEXT("caster_x_m"),CX))Geo.rear_caster_x=CX*100;}
 Num(TEXT("rear_caster_z_m"),Geo.caster_lift,100,true);Num(TEXT("caster_ball_radius_m"),Geo.caster_radius,100,true);
 const TArray<TSharedPtr<FJsonValue>>* Cliffs=nullptr;bool CliffOk=Params->TryGetArrayField(TEXT("cliff_sensors_m"),Cliffs)&&Cliffs&&Cliffs->Num()>0;
 std::vector<std::array<double,2>> CliffXY;
 if(CliffOk)for(const TSharedPtr<FJsonValue>& Entry:*Cliffs){
  const TArray<TSharedPtr<FJsonValue>>* XY=nullptr;double CX=0,CY=0;
  if(Entry.IsValid()&&Entry->TryGetArray(XY)&&XY&&XY->Num()>=2&&(*XY)[0]->TryGetNumber(CX)&&(*XY)[1]->TryGetNumber(CY))CliffXY.push_back({CX*100,CY*100});else CliffOk=false;
 }
 if(CliffOk)Geo.cliff_xy=CliffXY;else Fallback.Add(TEXT("cliff_sensors_m"));
 Num(TEXT("cliff_z_m"),Geo.cliff_z,100,true);
 if(Vec(TEXT("camera_mount_m"),3,V,true))Geo.camera={V[0]*100,V[1]*100,V[2]*100};
 if(Vec(TEXT("lidar_mount_m"),3,V,true))Geo.lidar={V[0]*100,V[1]*100,V[2]*100};
 // LiDAR turret cylinder (the CAD's lidar_turret part): the LiDAR column collider (RobotGeometry::lidar_column_prism).
 if(Vec(TEXT("lidar_turret_xy_m"),2,V,true))Geo.turret_xy={V[0]*100,V[1]*100};
 Num(TEXT("lidar_turret_radius_m"),Geo.turret_radius,100,true);
 if(Vec(TEXT("lidar_turret_z_m"),2,V,true))Geo.turret_z={V[0]*100,V[1]*100};
 // Body inertia about the body COM (kg m2, robot frame, 3 x 3 nested array; cad/mass-properties.json via the sync). Applied by
 // ApplyBodyInertia; a missing or non-physical tensor keeps the collider-derived inertia and is listed as a fallback.
 bBodyInertia=false;
 const TArray<TSharedPtr<FJsonValue>>* InertiaRows=nullptr;
 if(Params->TryGetArrayField(TEXT("body_inertia_kg_m2"),InertiaRows)&&InertiaRows&&InertiaRows->Num()==3){
  bool Ok=true;
  for(int I=0;I<3&&Ok;I++){
   const TArray<TSharedPtr<FJsonValue>>* Row=nullptr;
   Ok=(*InertiaRows)[I].IsValid()&&(*InertiaRows)[I]->TryGetArray(Row)&&Row&&Row->Num()==3;
   for(int J=0;J<3&&Ok;J++){double D=0;Ok=(*Row)[J].IsValid()&&(*Row)[J]->TryGetNumber(D)&&FMath::IsFinite(D);BodyInertia[I][J]=D;}
  }
  double E[3],A[3][3];bBodyInertia=Ok&&RobotGeometry::principal_inertia(BodyInertia,E,A);
 }
 if(!bBodyInertia)Fallback.Add(TEXT("body_inertia_kg_m2 (collider inertia kept)"));
 Num(TEXT("shell_bottom_z_m"),Geo.shell_bottom_z,100,true);Num(TEXT("body_top_z_m"),Geo.body_top_z,100,true);Num(TEXT("bumper_top_z_m"),Geo.bumper_top_z,100,true);
 // base_link_height_m is the physics-body origin height (BodyHeight, compiled): the controller's "base_link + offset"
 // convention. It cannot change at run time, so a different file value is reported, not applied.
 double BaseLinkHeight=BodyHeight;Num(TEXT("base_link_height_m"),BaseLinkHeight,100,true);
 if(FMath::Abs(BaseLinkHeight-BodyHeight)>1e-4){
  UE_LOG(LogTemp,Error,TEXT("ROBOT GEOMETRY: base_link_height_m %.4f m differs from the compiled body origin height %.4f m (RobotPawn BodyHeight)"),BaseLinkHeight/100,BodyHeight/100);
  Fallback.Add(FString::Printf(TEXT("base_link_height_m (file %.4f m, compiled %.4f m used)"),BaseLinkHeight/100,BodyHeight/100));
 }
 // Contract: bump_zone_edges_deg is symmetric and its outer values are -/+bumper_span_deg (the half-span). A full span (180)
 // with +/-90 edges would wrap the columns round the back and shrink the whole collider to the bumper stop.
 if(!RobotGeometry::span_matches_edges(Geo.zone_edges_deg,Geo.bumper_span_deg)){
  const std::vector<double>& E=Geo.zone_edges_deg;
  if(E.size()>=2&&FMath::Abs(E.front()+E.back())<1e-6&&E.back()>0&&E.back()<=180){
   UE_LOG(LogTemp,Error,TEXT("ROBOT GEOMETRY: bumper_span_deg %.3f does not match bump_zone_edges_deg (+/-%.3f); the edges are used"),Geo.bumper_span_deg,E.back());
   Geo.bumper_span_deg=E.back();Fallback.Add(TEXT("bumper_span_deg (mismatch with bump_zone_edges_deg; edges used)"));
  }else{
   UE_LOG(LogTemp,Error,TEXT("ROBOT GEOMETRY: bump_zone_edges_deg is not symmetric about 0; the compiled +/-90 deg zones are used"));
   const RobotGeometry::Geometry Compiled;Geo.zone_edges_deg=Compiled.zone_edges_deg;Geo.bumper_span_deg=Compiled.bumper_span_deg;
   Fallback.Add(TEXT("bump_zone_edges_deg (not symmetric; compiled zones used)"));
  }
 }
 // Sanity: the skin must lie outside the rigid shell (else contacts are never sensed) and the probes must reach the shell.
 if(!(Geo.skin_radius>Geo.rigid_radius&&Geo.skin_radius-Geo.rigid_radius<BumpInsideCm&&Geo.bumper_travel>0&&Geo.bumper_travel<BumpInsideCm&&Geo.rigid_radius>Geo.track_half+1&&Geo.bumper_span_deg>0&&Geo.bumper_span_deg<=180&&Geo.body_top_z>Geo.shell_bottom_z&&Geo.bumper_top_z>Geo.shell_bottom_z+1))
  UE_LOG(LogTemp,Error,TEXT("ROBOT GEOMETRY inconsistent: skin %.2f rigid %.2f travel %.2f track/2 %.2f span %.1f bottom %.2f top %.2f bumper top %.2f cm"),Geo.skin_radius,Geo.rigid_radius,Geo.bumper_travel,Geo.track_half,Geo.bumper_span_deg,Geo.shell_bottom_z,Geo.body_top_z,Geo.bumper_top_z);
 GeometrySource=TEXT("robot-parameters.json");
 if(PreSync)GeometrySource+=FString::Printf(TEXT(" PRE-SYNC (%s)"),*SyncStatus);
 if(Fallback.Num())GeometrySource+=FString::Printf(TEXT(" with FALLBACK for: %s"),*FString::Join(Fallback,TEXT(", ")));
 if(Fallback.Num())UE_LOG(LogTemp,Warning,TEXT("ROBOT GEOMETRY: compiled fallback used for %s"),*FString::Join(Fallback,TEXT(", ")));
}
// Applies Geo to the components (before physics starts) and builds the body collider and bumper probe columns. The collider
// is one convex prism: under the bumper span its radius is the bumper's hard stop (skin - bumper_travel; the bumper is
// compliant over that travel and its switch trips first), behind it the rigid rear shell (rigid_radius_m). Its bottom is the
// shell bottom (1.8 cm), so 2-3 cm obstacles hit the shell as on the hardware; the lowest probe row is just above it.
void ARobotPawn::ApplyGeometry() {
 TArray<FVector> Prism;for(const std::array<double,3>& P:RobotGeometry::body_prism(Geo))Prism.Add(ToUnreal(P[0],P[1],P[2]));
 Body->ClearCollisionConvexMeshes();Body->AddCollisionConvexMesh(Prism);
 // LiDAR mode: the LiDAR column (RobotGeometry::lidar_column_prism: the turret cylinder, lidar_turret_*, from the lid top to the
 // robot top), a second convex element of Body. Collision only in effect: Body's mass is the BodyMass override and its COM and
 // inertia are set by ApplyCenterOfMass / ApplyBodyInertia, so the column's volume changes neither. -StereoMode has no column.
 if(LidarMode){
  TArray<FVector> Column;for(const std::array<double,3>& P:RobotGeometry::lidar_column_prism(Geo))Column.Add(ToUnreal(P[0],P[1],P[2]));
  if(Column.Num())Body->AddCollisionConvexMesh(Column);
  else UE_LOG(LogTemp,Error,TEXT("ROBOT GEOMETRY: no LiDAR column collider (turret top %.2f cm is not above the body top %.2f cm)"),Geo.turret_z[1],Geo.body_top_z);
 }
 LeftWheel->SetRelativeLocation(WheelOffset(true));RightWheel->SetRelativeLocation(WheelOffset(false));
 Caster->SetSphereRadius(Geo.caster_radius);Caster->SetRelativeLocation(CasterOffset());
 // LiDAR mode (default): the single D200-class camera at camera_mount_m. -StereoMode is a LEGACY mode (the round-robot CAD
 // has camera_baseline 0): its pair straddles camera_mount_m by the legacy 6.2 cm baseline (CaptureStereo reports .062).
 if(LidarMode){LeftCamera->SetRelativeLocation(ToUnreal(Geo.camera[0],Geo.camera[1],Geo.camera[2]));LeftCamera->FOVAngle=84;}
 else{LeftCamera->SetRelativeLocation(ToUnreal(Geo.camera[0],Geo.camera[1]+LegacyStereoHalfBaselineCm,Geo.camera[2]));RightCamera->SetRelativeLocation(ToUnreal(Geo.camera[0],Geo.camera[1]-LegacyStereoHalfBaselineCm,Geo.camera[2]));}
 BumpColumns=RobotGeometry::bump_columns(Geo,bBumperSparseColumns?4:1);BumpRows=RobotGeometry::bump_rows(Geo);
 if(BumpRows.front()>Geo.shell_bottom_z+.05)UE_LOG(LogTemp,Error,TEXT("ROBOT GEOMETRY: lowest bumper probe row %.2f cm is above the shell bottom %.2f cm (blind band)"),BumpRows.front(),Geo.shell_bottom_z);
 LeftSwitchZones=RobotGeometry::left_switch_mask(Geo.zone_edges_deg);RightSwitchZones=RobotGeometry::right_switch_mask(Geo.zone_edges_deg);
 const RobotGeometry::Box2 Box=RobotGeometry::probe_bounds(BumpColumns,BumpInsideCm,BumpOutsideCm);
 const double Z0=BumpRows.front(),Z1=BumpRows.back();
 BumpBoxCenter=ToUnreal(Box.cx,Box.cy,(Z0+Z1)/2);BumpBoxExtent=FVector(Box.ex,Box.ey,(Z1-Z0)/2+.1);
 // Geometry echo in every state message, so tests can check the pawn really used robot-parameters.json.
 GeometryJson=MakeShared<FJsonObject>();
 auto Arr=[](std::initializer_list<double> L){TArray<TSharedPtr<FJsonValue>> A;for(double D:L)A.Add(MakeShared<FJsonValueNumber>(D));return A;};
 GeometryJson->SetStringField(TEXT("source"),GeometrySource);GeometryJson->SetNumberField(TEXT("drive_axle_x_m"),Geo.axle_x/100);
 GeometryJson->SetNumberField(TEXT("body_radius_m"),Geo.skin_radius/100);GeometryJson->SetNumberField(TEXT("rigid_radius_m"),Geo.rigid_radius/100);GeometryJson->SetNumberField(TEXT("bumper_travel_m"),Geo.bumper_travel/100);
 GeometryJson->SetNumberField(TEXT("wheel_track_m"),Geo.track_half/50);GeometryJson->SetArrayField(TEXT("rear_caster_xy_m"),Arr({Geo.rear_caster_x/100,Geo.rear_caster_y/100}));
 GeometryJson->SetNumberField(TEXT("bump_columns"),(double)BumpColumns.size());GeometryJson->SetNumberField(TEXT("bumper_span_deg"),Geo.bumper_span_deg);
 TArray<TSharedPtr<FJsonValue>> Edges;for(double E:Geo.zone_edges_deg)Edges.Add(MakeShared<FJsonValueNumber>(E));GeometryJson->SetArrayField(TEXT("bump_zone_edges_deg"),Edges);
 TArray<TSharedPtr<FJsonValue>> CliffArr;for(const std::array<double,2>& P:Geo.cliff_xy)CliffArr.Add(MakeShared<FJsonValueArray>(Arr({P[0]/100,P[1]/100})));GeometryJson->SetArrayField(TEXT("cliff_sensors_m"),CliffArr);
 GeometryJson->SetArrayField(TEXT("camera_mount_m"),Arr({Geo.camera[0]/100,Geo.camera[1]/100,Geo.camera[2]/100}));GeometryJson->SetArrayField(TEXT("lidar_mount_m"),Arr({Geo.lidar[0]/100,Geo.lidar[1]/100,Geo.lidar[2]/100}));
 GeometryJson->SetBoolField(TEXT("sparse_columns"),bBumperSparseColumns);GeometryJson->SetBoolField(TEXT("caster"),!NoCaster);
 GeometryJson->SetNumberField(TEXT("wheel_radius_m"),Geo.wheel_radius/100);GeometryJson->SetNumberField(TEXT("cliff_z_m"),Geo.cliff_z/100);
 GeometryJson->SetNumberField(TEXT("rear_caster_z_m"),Geo.caster_lift/100);GeometryJson->SetNumberField(TEXT("caster_ball_radius_m"),Geo.caster_radius/100);
 GeometryJson->SetNumberField(TEXT("shell_bottom_z_m"),Geo.shell_bottom_z/100);GeometryJson->SetNumberField(TEXT("body_top_z_m"),Geo.body_top_z/100);GeometryJson->SetNumberField(TEXT("bumper_top_z_m"),Geo.bumper_top_z/100);
 GeometryJson->SetNumberField(TEXT("base_link_height_m"),BodyHeight/100); // compiled body origin height (not read back)
 GeometryJson->SetArrayField(TEXT("lidar_turret_xy_m"),Arr({Geo.turret_xy[0]/100,Geo.turret_xy[1]/100}));GeometryJson->SetNumberField(TEXT("lidar_turret_radius_m"),Geo.turret_radius/100);
 GeometryJson->SetArrayField(TEXT("lidar_turret_z_m"),Arr({Geo.turret_z[0]/100,Geo.turret_z[1]/100}));
 GeometryJson->SetBoolField(TEXT("lidar_column"),LidarMode&&!RobotGeometry::lidar_column_prism(Geo).empty());GeometryJson->SetArrayField(TEXT("lidar_column_z_m"),Arr({Geo.body_top_z/100,Geo.turret_z[1]/100}));
 GeometryJson->SetBoolField(TEXT("body_inertia_from_file"),bBodyInertia);
 TArray<TSharedPtr<FJsonValue>> Rows;for(double Z:BumpRows)Rows.Add(MakeShared<FJsonValueNumber>(Z));GeometryJson->SetArrayField(TEXT("bump_rows_cm"),Rows);
}
// Body's COM is set through SetCenterOfMass (COMNudge). UE 5.8 (FBodyInstance::UpdateMassProperties) computes the mass
// properties of Body's own shapes (the prism and the LiDAR column; mass override BodyMass, plus the nudge) and of each welded
// child separately and combines them BY MASS, so a nudge n moves the combined COM by n*BodyMass/M (M = combined physics mass;
// M = BodyMass now that nothing is welded in, but the general form is kept). The nudge is therefore (desired - zero-nudge
// combined COM) * M/BodyMass, followed by one correction from the measured residual (covers any welded shape). Called at the
// end of BeginPlay's physics setup (after every SetupBody and SetupJoints) and again on the first tick as a guard; both start
// from a zero nudge, so the two COM_FIX log lines must agree (a difference means Body's weld set changed after BeginPlay). The
// residual is logged; > 0.5 mm is an error. Every SetCenterOfMass recomputes the collider inertia, so the body inertia from the
// file is applied last (ApplyBodyInertia).
void ARobotPawn::ApplyCenterOfMass(const TCHAR* When) {
 if(!Body->IsSimulatingPhysics())return;
 const FTransform& X=Body->GetComponentTransform();
 auto Com=[this,&X]{return X.InverseTransformPosition(Body->BodyInstance.GetCOMPosition());};
 Body->SetCenterOfMass(FVector::ZeroVector);
 const FVector Centroid=Com();
 const double Own=Body->BodyInstance.GetMassOverride(),Total=Body->GetMass();
 const double K=(Own>0&&Total>0)?Total/Own:1.0;
 const FVector Desired(BodyComX,BodyComY,BodyComZ-BodyHeight);
 FVector Nudge=(Desired-Centroid)*K;Body->SetCenterOfMass(Nudge);
 FVector Got=Com();
 if(FVector::Dist(Got,Desired)>.001){Nudge+=(Desired-Got)*K;Body->SetCenterOfMass(Nudge);Got=Com();}
 const double ResidualMm=FVector::Dist(Got,Desired)*10;
 UE_LOG(LogTemp,Display,TEXT("COM_FIX %s: physics mass %.4f kg (Body override %.4f kg, welded extra %.4f kg), zero-nudge centroid (%.3f, %.3f, %.3f), nudge (%.3f, %.3f, %.3f), body COM (%.3f, %.3f, %.3f), desired (%.3f, %.3f, %.3f) cm, Unreal component space; residual %.3f mm"),
  When,Total,Own,Total-Own,Centroid.X,Centroid.Y,Centroid.Z,Nudge.X,Nudge.Y,Nudge.Z,Got.X,Got.Y,Got.Z,Desired.X,Desired.Y,Desired.Z,ResidualMm);
 if(ResidualMm>.5||Total-Own>.005)UE_LOG(LogTemp,Error,TEXT("COM_FIX %s: residual %.3f mm or welded extra mass %.4f kg exceeds the limit (0.5 mm / 5 g)"),When,ResidualMm,Total-Own);
 ApplyBodyInertia(When);
}
// Body inertia from body_inertia_kg_m2 (CAD, about the body COM, robot frame). UE 5.8's UpdateMassProperties (BodyUtils.cpp)
// takes the inertia from the collider (uniform density, scaled to the mass override) and then, for the COM nudge, adds
// M*nudge_i^2 per axis (p.ComNudgeAffectsInertia, on by default); BodyInstance.InertiaTensorScale would only be a geometric
// scale on that. So the file tensor's principal moments and axes are set directly, after the nudge, with the two calls
// UpdateMassProperties itself ends with (SetMassSpaceInertiaTensor, SetComLocalPose keeping the COM position). A later
// UpdateMassProperties (another SetCenterOfMass, a weld) restores the collider inertia: ApplyCenterOfMass calls this each time.
// The full tensor is read back (PhysicsBodyInertia) and compared with the file; > 1 % of the mean moment is an error.
void ARobotPawn::ApplyBodyInertia(const TCHAR* When) {
 if(!Body->IsSimulatingPhysics())return;
 const FVector Collider=Body->GetInertiaTensor(); // principal moments from the collider and the nudge (kg cm2)
 if(!bBodyInertia){UE_LOG(LogTemp,Warning,TEXT("INERTIA %s: body_inertia_kg_m2 missing or not physical; collider inertia kept, principal (%.2f, %.2f, %.2f) kg cm2"),When,Collider.X,Collider.Y,Collider.Z);return;}
 double U[3][3],E[3],A[3][3];RobotGeometry::inertia_to_unreal(BodyInertia,U);RobotGeometry::principal_inertia(U,E,A);
 const FQuat Axes(FMatrix(FVector(A[0][0],A[0][1],A[0][2]),FVector(A[1][0],A[1][1],A[1][2]),FVector(A[2][0],A[2][1],A[2][2]),FVector::ZeroVector)); // mass space -> component space
 const FVector Moments(E[0]*1e4,E[1]*1e4,E[2]*1e4); // kg m2 -> kg cm2 (Unreal units)
 FPhysicsCommand::ExecuteWrite(Body->BodyInstance.GetPhysicsActor(),[&](const FPhysicsActorHandle& Actor){
  const FTransform Com=FPhysicsInterface::GetComTransformLocal_AssumesLocked(Actor);
  FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(Actor,Moments);
  FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor,FTransform(Axes,Com.GetLocation()));
 });
 double Got[3][3];double Worst=1e9;
 if(PhysicsBodyInertia(Got)){
  Worst=0;const double Mean=(BodyInertia[0][0]+BodyInertia[1][1]+BodyInertia[2][2])/3;
  for(int I=0;I<3;I++)for(int J=0;J<3;J++)Worst=FMath::Max(Worst,FMath::Abs(Got[I][J]-BodyInertia[I][J])/Mean);
 }
 UE_LOG(LogTemp,Display,TEXT("INERTIA_FIX %s: collider principal (%.2f, %.2f, %.2f) kg cm2 -> body_inertia_kg_m2 principal (%.2f, %.2f, %.2f) kg cm2, largest-moment axis %.2f deg from z; read-back error %.3f %% of the mean moment"),
  When,Collider.X,Collider.Y,Collider.Z,Moments.X,Moments.Y,Moments.Z,FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FMath::Abs(A[2][2]),0.0,1.0))),Worst*100);
 if(Worst>.01)UE_LOG(LogTemp,Error,TEXT("INERTIA_FIX %s: read-back error %.3f %% of the mean moment exceeds 1 %%"),When,Worst*100);
}
// Body's inertia tensor about its physics COM in the robot frame (x fwd, y LEFT, z up), kg m2, from the physics state.
bool ARobotPawn::PhysicsBodyInertia(double Out[3][3]) const {
 if(!Body||!Body->IsSimulatingPhysics())return false;
 FVector D=FVector::ZeroVector;FQuat Q=FQuat::Identity;
 FPhysicsCommand::ExecuteRead(Body->BodyInstance.GetPhysicsActor(),[&](const FPhysicsActorHandle& Actor){
  D=FPhysicsInterface::GetLocalInertiaTensor_AssumesLocked(Actor);Q=FPhysicsInterface::GetComTransformLocal_AssumesLocked(Actor).GetRotation();
 });
 const FVector Axis[3]={Q.RotateVector(FVector::XAxisVector),Q.RotateVector(FVector::YAxisVector),Q.RotateVector(FVector::ZAxisVector)};
 const double S[3]={1,-1,1}; // Unreal y right -> robot y left
 for(int I=0;I<3;I++)for(int J=0;J<3;J++){double V=0;for(int K=0;K<3;K++)V+=D[K]*Axis[K][I]*Axis[K][J];Out[I][J]=V*S[I]*S[J]/1e4;}
 return true;
}
// Normals and tangents as UKismetProceduralMeshLibrary::CalculateTangentsForMesh produces them for this call (zero UVs: its
// singular UV matrix falls back to identity, so tangent X / Y are the triangles' unit edges P1-P0 / P2-P0; the normal is the sum
// of the unit normals (P1-P2) x (P0-P2) of every triangle touching a vertex at the same position), but with the same-position
// groups found through a hash map instead of its FindVertOverlaps scan of every vertex per triangle corner. That scan is O(n^2):
// with the bay board's ~0.5 M-vertex assembly (bay B10) it kept BeginPlay busy for minutes (the old 0.32 M-vertex assembly took
// about 70 s). The grouping is close to, not the same as, UE's: the hash map puts positions in cells of a 1e-4 cm grid, while
// FindVertOverlaps pairs any two vertices within FVector::Equals' 1e-4 tolerance per axis. Two vertices less than 1e-4 apart
// that straddle a cell edge stay apart here, and UE's pairing is not transitive. The CAD meshes repeat shared positions exactly,
// so both group them the same way: a numpy port of both on 9,668 assembly vertices differed by at most 2.2e-16 (bay B10).
static void SmoothNormals(const TArray<FVector>& V,const TArray<int32>& T,TArray<FVector>& Normals,TArray<FProcMeshTangent>& Tangents) {
 const int32 NV=V.Num(),NT=T.Num()/3;Normals.Reset();Tangents.Reset();if(!NV)return;
 TArray<int32> Group;Group.SetNumUninitialized(NV);TMap<FIntVector,int32> Index;Index.Reserve(NV);
 for(int32 I=0;I<NV;I++){const FIntVector Key(FMath::RoundToInt32(V[I].X*1e4),FMath::RoundToInt32(V[I].Y*1e4),FMath::RoundToInt32(V[I].Z*1e4));const int32 Next=Index.Num();Group[I]=Index.FindOrAdd(Key,Next);}
 TArray<FVector3f> SumZ,SumX,SumY;SumZ.SetNumZeroed(Index.Num());SumX.SetNumZeroed(NV);SumY.SetNumZeroed(NV);
 for(int32 K=0;K<NT;K++){
  int32 C[3];FVector3f P[3];for(int32 J=0;J<3;J++){C[J]=FMath::Clamp(T[3*K+J],0,NV-1);P[J]=(FVector3f)V[C[J]];}
  const FVector3f Z=((P[1]-P[2])^(P[0]-P[2])).GetSafeNormal(),X=(P[1]-P[0]).GetSafeNormal(),Y=(P[2]-P[0]).GetSafeNormal();
  for(int32 J=0;J<3;J++){if((J==1&&C[1]==C[0])||(J==2&&(C[2]==C[0]||C[2]==C[1])))continue;SumZ[Group[C[J]]]+=Z;SumX[C[J]]+=X;SumY[C[J]]+=Y;}
 }
 Normals.SetNumUninitialized(NV);Tangents.SetNumUninitialized(NV);
 for(int32 I=0;I<NV;I++){
  FVector3f Z=SumZ[Group[I]],X=SumX[I];X.Normalize();Z.Normalize();X-=Z*(Z|X);X.Normalize();
  Normals[I]=(FVector)Z;Tangents[I]=FProcMeshTangent((FVector)X,((Z^X)|SumY[I])<0.f);
 }
}
void ARobotPawn::BuildRobotVisual() {
 ColorMaterial=LoadObject<UMaterialInterface>(nullptr,TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
 FString Text;FFileHelper::LoadFileToString(Text,*(FPaths::ProjectContentDir()/TEXT("RobotData/assembly-meshes.json")));
 TArray<TSharedPtr<FJsonValue>> Parts;FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Parts);
 struct MeshData {TArray<FVector> V;TArray<int32> T;TArray<FLinearColor> C;};MeshData M[3];FBox WheelBounds[3]={FBox(ForceInit),FBox(ForceInit),FBox(ForceInit)};
 for(auto& Value:Parts){auto P=Value->AsObject();const FString Id=P->GetStringField(TEXT("id"));if(RobotGeometry::hidden_visual_part(TCHAR_TO_UTF8(*Id)))continue;
  // Part-id based: tyre and wheel meshes ride on the wheel bodies, re-centred at the axle (drive_axle_x_m, +/-track/2, wheel radius).
  int G=0;if(Id==TEXT("tire_left")||Id==TEXT("wheel_left"))G=1;else if(Id==TEXT("tire_right")||Id==TEXT("wheel_right"))G=2;
  FVector Offset(0,0,BodyHeight);if(G==1)Offset=WheelOffset(true)+FVector(0,0,BodyHeight);if(G==2)Offset=WheelOffset(false)+FVector(0,0,BodyHeight);
  auto& D=M[G];const int Base=D.V.Num();auto& V=P->GetArrayField(TEXT("vertices"));const FLinearColor Col=FLinearColor::FromSRGBColor(FColor::FromHex(P->GetStringField(TEXT("color"))));
  for(int I=0;I<V.Num();I+=3){D.V.Add(FVector(V[I]->AsNumber()/10,-V[I+1]->AsNumber()/10,V[I+2]->AsNumber()/10)-Offset);D.C.Add(Col);WheelBounds[G]+=D.V.Last();}
  auto& T=P->GetArrayField(TEXT("faces"));for(int I=0;I<T.Num();I+=3){D.T.Add(Base+T[I]->AsNumber());D.T.Add(Base+T[I+2]->AsNumber());D.T.Add(Base+T[I+1]->AsNumber());}
 }
 // The CAD wheel must sit on the physics axle; a mismatch means parameters.json and the meshes disagree.
 for(int G=1;G<3;G++)if(WheelBounds[G].IsValid&&WheelBounds[G].GetCenter().Size()>.3)
  UE_LOG(LogTemp,Warning,TEXT("ROBOT GEOMETRY: %s wheel mesh centre is (%.2f, %.2f, %.2f) cm from the physics axle"),G==1?TEXT("left"):TEXT("right"),WheelBounds[G].GetCenter().X,WheelBounds[G].GetCenter().Y,WheelBounds[G].GetCenter().Z);
 UProceduralMeshComponent* Targets[]={Visual,LeftWheel,RightWheel};
 const double TangentStart=FPlatformTime::Seconds();int32 VisualVerts=0;
 for(int I=0;I<3;I++){TArray<FVector> N;TArray<FProcMeshTangent> Tan;TArray<FVector2D> UV;UV.Init(FVector2D::ZeroVector,M[I].V.Num());SmoothNormals(M[I].V,M[I].T,N,Tan);VisualVerts+=M[I].V.Num();Targets[I]->CreateMeshSection_LinearColor(0,M[I].V,M[I].T,N,UV,M[I].C,Tan,false);Targets[I]->SetMaterial(0,ColorMaterial);}
 UE_LOG(LogTemp,Display,TEXT("ROBOT VISUAL: %d vertices, normals in %.2f s"),VisualVerts,FPlatformTime::Seconds()-TangentStart);
 TArray<FVector> Hull;const double R=Geo.wheel_radius;for(int I=0;I<48;I++){const double A=I*2*PI/48;for(double Y:{-.5,.5})Hull.Add(FVector(R*FMath::Cos(A),Y,R*FMath::Sin(A)));}
 LeftWheel->AddCollisionConvexMesh(Hull);RightWheel->AddCollisionConvexMesh(Hull);
}
void ARobotPawn::SetupJoints() {
 auto Joint=[&](UPhysicsConstraintComponent* J,UPrimitiveComponent* P,FVector Location,bool Ball){
  J->SetWorldLocation(Location);J->SetWorldRotation(Body->GetComponentQuat()*FRotator(0,90,0).Quaternion());
  J->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked,0);J->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked,0);J->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked,0);
  J->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free,0);J->SetAngularSwing1Limit(Ball?EAngularConstraintMotion::ACM_Free:EAngularConstraintMotion::ACM_Locked,0);J->SetAngularSwing2Limit(Ball?EAngularConstraintMotion::ACM_Free:EAngularConstraintMotion::ACM_Locked,0);
  J->SetDisableCollision(true);J->SetConstrainedComponents(Body,NAME_None,P,NAME_None);
 };
 Joint(LeftJoint,LeftWheel,LeftWheel->GetComponentLocation(),false);Joint(RightJoint,RightWheel,RightWheel->GetComponentLocation(),false);
 if(!NoCaster)Joint(CasterJoint,Caster,Caster->GetComponentLocation(),true);
}
void ARobotPawn::BeginPlay() {
 Super::BeginPlay();NoCaster=FParse::Param(FCommandLine::Get(),TEXT("NoCaster"));FParse::Value(FCommandLine::Get(),TEXT("Scenario="),Scenario);
 LoadParameters();ConfigureDrive();
 FParse::Value(FCommandLine::Get(),TEXT("ComX="),BodyComX);FParse::Value(FCommandLine::Get(),TEXT("ComZ="),BodyComZ);
 double MassScale=1;FParse::Value(FCommandLine::Get(),TEXT("MassScale="),MassScale);BodyMass*=FMath::Clamp(MassScale,.5,2.0);
 for(auto& Row:BodyInertia)for(double& D:Row)D*=FMath::Clamp(MassScale,.5,2.0); // same mass distribution, heavier
 LidarMode=!FParse::Param(FCommandLine::Get(),TEXT("StereoMode"));
 bBumperSparseColumns=FParse::Param(FCommandLine::Get(),TEXT("BumperSparseColumns")); // test-only negative control
 FParse::Value(FCommandLine::Get(),TEXT("LidarBiasFraction="),LidarBiasFraction);
 FParse::Value(FCommandLine::Get(),TEXT("LidarJitterFraction="),LidarJitterFraction);
 FParse::Value(FCommandLine::Get(),TEXT("LidarDropout="),LidarDropout);
 LidarBiasFraction=FMath::Clamp(LidarBiasFraction,-1.,1.);LidarJitterFraction=FMath::Clamp(LidarJitterFraction,0.,2.);LidarDropout=FMath::Clamp(LidarDropout,0.,1.);
 ApplyGeometry();
 BuildRobotVisual();
 TireMaterial=NewObject<UPhysicalMaterial>(this);TireMaterial->Friction=.8;TireMaterial->StaticFriction=.9;TireMaterial->Restitution=0;TireMaterial->bOverrideFrictionCombineMode=true;TireMaterial->FrictionCombineMode=EFrictionCombineMode::Min;
 CasterMaterial=NewObject<UPhysicalMaterial>(this);CasterMaterial->Friction=.25;CasterMaterial->Restitution=0;
 SetupBody(Body,BodyMass,CasterMaterial);
 SetupBody(LeftWheel,WheelMass,TireMaterial);SetupBody(RightWheel,WheelMass,TireMaterial);
 for(auto* Wheel:{LeftWheel,RightWheel}){const FVector I=Wheel->GetInertiaTensor();Wheel->BodyInstance.InertiaTensorScale=FVector(FMath::Sqrt(FMath::Max(1.0,5.0/I.Y)));Wheel->BodyInstance.UpdateMassProperties();}
 if(!NoCaster)SetupBody(Caster,CasterMass,CasterMaterial);else{Caster->SetCollisionEnabled(ECollisionEnabled::NoCollision);Caster->SetVisibility(false);}
 SetupJoints();
 // After every SetupBody: the wheels (procedural meshes, bAutoWeld off) never weld into Body, but the caster sphere
 // (UShapeComponent, bAutoWeld on) can weld transiently when its collision turns on and unweld when it starts simulating,
 // which recomputes Body's mass properties. Measuring here sees the final weld set (Body's prism and LiDAR column, nothing welded).
 ApplyCenterOfMass(TEXT("BeginPlay"));
 Socket=FUdpSocketBuilder(TEXT("RobotBridgeLocalhost")).AsNonBlocking().AsReusable().BoundToAddress(FIPv4Address(127,0,0,1)).BoundToPort(8767).WithReceiveBufferSize(65536);
 Peer=ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();bool Valid=false;Peer->SetIp(TEXT("127.0.0.1"),Valid);Peer->SetPort(8768);
 FParse::Value(FCommandLine::Get(),TEXT("CameraHz="),CameraHz);CameraHz=FMath::Clamp(CameraHz,1.0,10.0);
 if(!Socket)UE_LOG(LogTemp,Error,TEXT("Robot UDP port 8767 unavailable"));
 if(!FParse::Param(FCommandLine::Get(),TEXT("nullrhi")))NewStereoSession();
 if(!FParse::Param(FCommandLine::Get(),TEXT("nullrhi"))){
  LeftTarget=NewObject<UTextureRenderTarget2D>(this);RightTarget=NewObject<UTextureRenderTarget2D>(this);
  for(auto* T:{LeftTarget,RightTarget}){T->RenderTargetFormat=RTF_RGBA8;T->InitAutoFormat(640,360);T->UpdateResourceImmediate(true);}
  LeftCamera->TextureTarget=LeftTarget;RightCamera->TextureTarget=RightTarget;
  LeftCamera->HideActorComponents(this);RightCamera->HideActorComponents(this);
 }
 Overview=GetWorld()->SpawnActor<ACameraActor>();Overview->GetCameraComponent()->SetFieldOfView(60);
 if(auto* PC=GetWorld()->GetFirstPlayerController()){PC->Possess(this);PC->SetViewTarget(Overview);PC->bShowMouseCursor=true;FInputModeGameOnly Input;Input.SetConsumeCaptureMouseDown(false);PC->SetInputMode(Input);}
 SetPreviewMode(3);
 UE_LOG(LogTemp,Display,TEXT("ROBOT READY: physics body %.3f kg (welded, measured %.4f kg); COM %.2f / %.2f cm; localhost UDP 8767; scenario %s; round skin r=%.2f cm (bumper stop %.2f, rigid rear %.2f), bumper %d columns x %d rows (z %.2f-%.2f cm) over +/-%.0f deg; axle x=%.2f cm; rear caster (%.2f, %.2f) cm%s; geometry from %s"),
  BodyMass,Body->IsSimulatingPhysics()?Body->GetMass():0.f,BodyComX,BodyComZ,*Scenario,Geo.skin_radius,RobotGeometry::front_stop_radius(Geo),Geo.rigid_radius,(int)BumpColumns.size(),(int)BumpRows.size(),BumpRows.front(),BumpRows.back(),Geo.bumper_span_deg,Geo.axle_x,Geo.rear_caster_x,Geo.rear_caster_y,NoCaster?TEXT(" REMOVED (-NoCaster)"):TEXT(""),*GeometrySource);
}
void ARobotPawn::NewStereoSession() {
 Frame=0;DataDir=FPaths::ProjectSavedDir()/TEXT("Stereo")/FString::Printf(TEXT("run_%s_%llu"),*FDateTime::UtcNow().ToString(TEXT("%Y%m%dT%H%M%S")),FPlatformTime::Cycles64());IFileManager::Get().MakeDirectory(*DataDir,true);
}
void ARobotPawn::ResetRobot(FVector P,double Heading) {
 CaptureEnabled=false;LidarScan.Reset();LidarRanges.Empty();LidarTimes.Empty();LidarCarry=0;LidarIndex=LidarFrame=0;LidarRandom.Initialize(7391);if(LeftTarget)NewStereoSession();
 BumpZonesLatched=0;
 const int BumpEvents=Drive.bump_events;Drive=DriveCore();ConfigureDrive();Drive.bump_events=BumpEvents;Time=0;NextCapture=0;LeftTicks=RightTicks=0;LeftRate=RightRate=0;NextState=0;PreviousVelocity=FVector::ZeroVector;
 FQuat Q=FRotator(0,-Heading*180/PI,0).Quaternion();P.Z+=BodyHeight+.3;
 Body->SetWorldLocationAndRotation(P,Q,false,nullptr,ETeleportType::TeleportPhysics);
 auto ResetPart=[&](UPrimitiveComponent* C,FVector Offset){C->SetWorldLocationAndRotation(P+Q.RotateVector(Offset),Q,false,nullptr,ETeleportType::TeleportPhysics);C->SetPhysicsLinearVelocity(FVector::ZeroVector);C->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);C->WakeAllRigidBodies();};
 // Wheels on the axle through base_link, the single rear caster at rear_caster_xy_m (same offsets as the constructor/ApplyGeometry).
 ResetPart(Body,FVector::ZeroVector);ResetPart(LeftWheel,WheelOffset(true));ResetPart(RightWheel,WheelOffset(false));if(!NoCaster)ResetPart(Caster,CasterOffset());SetupJoints();FaultReason=TEXT("disarmed");
}
void ARobotPawn::PollCommands() {
 if(!Socket)return;uint32 Pending=0;
 while(Socket->HasPendingData(Pending)){
  TArray<uint8> Bytes;Bytes.SetNumUninitialized(FMath::Min(Pending,65535u)+1);int32 Read=0;if(!Socket->Recv(Bytes.GetData(),Bytes.Num()-1,Read))break;Bytes[Read]=0;
  FString Text=UTF8_TO_TCHAR(reinterpret_cast<const char*>(Bytes.GetData()));TSharedPtr<FJsonObject> O;if(!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),O)||!O.IsValid())continue;
  KeyboardControl=false;bool Flag=false;if(O->TryGetBoolField(TEXT("reset"),Flag)&&Flag){double X=0,Y=0,Z=0,H=0;O->TryGetNumberField(TEXT("x_m"),X);O->TryGetNumberField(TEXT("y_m"),Y);O->TryGetNumberField(TEXT("z_m"),Z);O->TryGetNumberField(TEXT("yaw_rad"),H);ResetRobot(FVector(X*100,-Y*100,Z*100),H);}
  if((O->TryGetBoolField(TEXT("motor_power_on"),Flag)||O->TryGetBoolField(TEXT("release_estop"),Flag))&&Flag)Drive.clear_stop();
  if(O->TryGetBoolField(TEXT("capture"),Flag))CaptureEnabled=Flag;
  double Rate=0;if(O->TryGetNumberField(TEXT("camera_hz"),Rate))CameraHz=FMath::Clamp(Rate,1.0,10.0);
  if(O->TryGetBoolField(TEXT("quit"),Flag)&&Flag)FGenericPlatformMisc::RequestExit(false);
  double V=0,W=0;bool Arm=false,Stop=false;O->TryGetNumberField(TEXT("v_m_s"),V);O->TryGetNumberField(TEXT("w_rad_s"),W);O->TryGetBoolField(TEXT("arm"),Arm);if(!O->TryGetBoolField(TEXT("motor_power_off"),Stop))O->TryGetBoolField(TEXT("estop"),Stop);
  Drive.command(Time,V,W,Arm,Stop);
 }
}
// Returns the hard hazards (cliff/invalid range, tilt); the front bumper is reported separately in Bump.
// Fault text priority: tilt > cliff_or_invalid > bumper, so a reported 'bumper' always means bumper-only.
bool ARobotPawn::TraceSensors() {
 FCollisionQueryParams Q;Q.AddIgnoredActor(this);Cliff.Empty();bool Hazard=false;FaultReason=TEXT("none");const FTransform& X=Body->GetComponentTransform();
 // Bumper skin: probe columns on the skin circle (body_radius_m) about base_link, evenly spaced <= 4 deg over
 // -bumper_span_deg..+bumper_span_deg (46 columns for +/-90), x BumpRows (shell bottom + 0.2 mm, then 2.8-12.5 cm). Each ray runs along the outward radial
 // from 1.5 cm inside the skin to 0.2 cm outside it, so a held contact stays pressed and anything touching the rigid shell
 // (rigid_radius_m, inside the skin) is inside the reach; hits with |n_z|>0.7 (floor, threshold or rug tops seen when the
 // body pitches) are ignored. Zone = bump_zone_edges_deg interval of the column; left switch = centre zone and the zones to its
 // left, right switch = centre zone and the zones to its right.
 const double Start=FPlatformTime::Seconds();BumpZones=0;
 // Broadphase: one oriented box that contains every probe segment. Nothing inside it -> no probe can hit; skip the grid.
 const bool Near=!BumpColumns.empty()&&GetWorld()->OverlapAnyTestByChannel(X.TransformPosition(BumpBoxCenter),X.GetRotation(),ECC_Visibility,FCollisionShape::MakeBox(BumpBoxExtent),Q);
 int Rays=0;
 if(Near)for(const RobotGeometry::BumpColumn& C:BumpColumns){
  const uint8 Bit=static_cast<uint8>(1u<<C.Zone);if(BumpZones&Bit)continue; // a zone already pressed needs no more rays
  for(double Z:BumpRows){
   const FVector S=X.TransformPosition(ToUnreal(C.X-BumpInsideCm*C.NX,C.Y-BumpInsideCm*C.NY,Z));
   const FVector E=X.TransformPosition(ToUnreal(C.X+BumpOutsideCm*C.NX,C.Y+BumpOutsideCm*C.NY,Z));
   FHitResult H;Rays++;if(GetWorld()->LineTraceSingleByChannel(H,S,E,ECC_Visibility,Q)&&FMath::Abs(H.ImpactNormal.Z)<=BumpMaxNormalZ){BumpZones|=Bit;break;}
  }
 }
 BumpZonesLatched|=BumpZones;BumpRight=(BumpZones&RightSwitchZones)!=0;BumpLeft=(BumpZones&LeftSwitchZones)!=0;
 Bump=BumpLeft||BumpRight;if(Bump)FaultReason=TEXT("bumper");
 const double Cost=(FPlatformTime::Seconds()-Start)*1e6;BumperTicks++;BumperCostSum+=Cost;BumperCostMax=FMath::Max(BumperCostMax,Cost);
 if(Near){BumperNearTicks++;BumperNearCostSum+=Cost;}
 if(Near&&!BumpZones){BumperFullTicks++;BumperFullRays=Rays;BumperFullCostSum+=Cost;BumperFullCostMax=FMath::Max(BumperFullCostMax,Cost);}
 if(BumperTicks%1800==0)LogSensorCost();
 // Cliff ToF sensors (cliff_sensors_m, order FL, FR, RL, RR) looking straight down from cliff_z_m.
 for(const std::array<double,2>& P:Geo.cliff_xy){
  const FVector S=X.TransformPosition(ToUnreal(P[0],P[1],Geo.cliff_z));FHitResult H;const FVector End=S-Body->GetUpVector()*30;
  bool Hit=GetWorld()->LineTraceSingleByChannel(H,S,End,ECC_Visibility,Q);double D=Hit?H.Distance/100:-1;Cliff.Add(D);if(D<.02||D>.08){Hazard=true;FaultReason=TEXT("cliff_or_invalid");}
 }
 const FRotator R=Body->GetComponentRotation();if(FMath::Abs(R.Roll)>20||FMath::Abs(R.Pitch)>20){Hazard=true;FaultReason=TEXT("tilt");}
 return Hazard;
}
void ARobotPawn::LogSensorCost() {
 UE_LOG(LogTemp,Display,TEXT("SENSOR_COST bumper_ticks=%lld bumper_avg_us=%.2f bumper_max_us=%.2f near_ticks=%lld near_avg_us=%.2f full_grid_ticks=%lld full_grid_rays=%d full_grid_avg_us=%.2f full_grid_max_us=%.2f"),
  BumperTicks,BumperCostSum/FMath::Max<int64>(1,BumperTicks),BumperCostMax,BumperNearTicks,BumperNearCostSum/FMath::Max<int64>(1,BumperNearTicks),
  BumperFullTicks,BumperFullRays,BumperFullCostSum/FMath::Max<int64>(1,BumperFullTicks),BumperFullCostMax);
}
void ARobotPawn::Tick(float Dt) {
 Super::Tick(Dt);if(bComPending){bComPending=false;ApplyCenterOfMass(TEXT("first tick"));}
 Time+=Dt;PollCommands();const FVector Axis=Body->GetRightVector();
 LeftRate=FVector::DotProduct(LeftWheel->GetPhysicsAngularVelocityInRadians()-Body->GetPhysicsAngularVelocityInRadians(),Axis);RightRate=FVector::DotProduct(RightWheel->GetPhysicsAngularVelocityInRadians()-Body->GetPhysicsAngularVelocityInRadians(),Axis);
 LeftTicks+=LeftRate*Dt/(2*PI)*EncoderCPR;RightTicks+=RightRate*Dt/(2*PI)*EncoderCPR;
 const bool Hazard=TraceSensors();
 if(auto* PC=GetWorld()->GetFirstPlayerController()){
  const bool Keys=PC->IsInputKeyDown(EKeys::W)||PC->IsInputKeyDown(EKeys::S)||PC->IsInputKeyDown(EKeys::A)||PC->IsInputKeyDown(EKeys::D);
  if(PC->WasInputKeyJustPressed(EKeys::SpaceBar)){Drive.estop=true;Drive.disarm();}
  if(PC->WasInputKeyJustPressed(EKeys::P)){Drive.clear_stop();}
  if(PC->WasInputKeyJustPressed(EKeys::R)){KeyboardControl=true;Drive.command(Time,0,0,true,false);}
  if(PC->WasInputKeyJustPressed(EKeys::C))CaptureEnabled=!CaptureEnabled;
  if(Keys||KeyboardControl){KeyboardControl=true;double V=(PC->IsInputKeyDown(EKeys::W)? .12:0)-(PC->IsInputKeyDown(EKeys::S)? .12:0),W=(PC->IsInputKeyDown(EKeys::A)? .7:0)-(PC->IsInputKeyDown(EKeys::D)? .7:0);Drive.command(Time,V,W,false,false);}
  else if(KeyboardWasDown)Drive.command(Time,0,0,false,false);KeyboardWasDown=Keys;
 }
 // Which switch is pressed (+1 left only, -1 right only, 0 both): DriveCore allows a pressed spin only toward that side.
 const int BumpSide=BumpLeft==BumpRight?0:(BumpLeft?1:-1);
 double TL=0,TR=0;Drive.update(Time,FMath::Min(Dt,.05f),LeftRate,RightRate,Hazard,TL,TR,Bump,BumpSide);
 if(Drive.estop)FaultReason=TEXT("motor_power_off");else if(Time-Drive.command_time>Drive.timeout&&Drive.have_command)FaultReason=TEXT("watchdog");else if(!Drive.armed&&FaultReason==TEXT("none"))FaultReason=TEXT("disarmed");
 // State goes out at 20 Hz but faults are per tick: latch the newest fault onset so a 1-2 tick event is never lost.
 if(FaultReason!=TEXT("none")&&FaultReason!=TEXT("disarmed")&&FaultReason!=PreviousFault){LastFaultEvent=FaultReason;LastFaultTime=Time;}PreviousFault=FaultReason;
 auto Torque=[&](UPrimitiveComponent* W,double Rate,double T){if(!Drive.armed&&!Drive.estop)T=FMath::Clamp(-Rate*.010,-Drive.max_torque,Drive.max_torque);const double Drag=FMath::Clamp(Rate*.004,-RollingTorque,RollingTorque);const FVector TorqueVector=Axis*((T-Drag)*10000);W->AddTorqueInRadians(TorqueVector);Body->AddTorqueInRadians(-TorqueVector);};Torque(LeftWheel,LeftRate,TL);Torque(RightWheel,RightRate,TR);
 const FVector Velocity=Body->GetPhysicsLinearVelocity()/100;SpecificForce=Body->GetComponentQuat().UnrotateVector((Velocity-PreviousVelocity)/FMath::Max(Dt,.001f)+FVector(0,0,9.8));PreviousVelocity=Velocity;
 UpdatePreview(Dt);
 if(LidarMode)SampleLidar(Dt);
 if(Time>=NextState){SendState();NextState=Time+.05;}
 if(CaptureEnabled&&LeftTarget&&Time>=NextCapture){CaptureStereo();NextCapture=Time+1/CameraHz;}
 if(GEngine)GEngine->AddOnScreenDebugMessage(770,0,FColor::White,FString::Printf(TEXT("HOME ROBOT R1 | %s | %s | bumps %d | zones 0x%02x\nW/S move  A/D turn  P motor ON  R arm  SPACE motor OFF  C camera\nVIEW: 1 follow  2 robot eye  3 house  4 top | drag / arrows rotate | scroll / +/- zoom\nPitch %.1f  Roll %.1f | Camera %s | UDP 127.0.0.1:8767"),Drive.armed?TEXT("ARMED"):TEXT("DISARMED"),*FaultReason,Drive.bump_events,BumpZones,Body->GetComponentRotation().Pitch,Body->GetComponentRotation().Roll,CaptureEnabled?TEXT("ON"):TEXT("OFF")));
}
void ARobotPawn::SetPreviewMode(int Mode){
 PreviewMode=FMath::Clamp(Mode,1,4);
 if(auto* PC=GetWorld()->GetFirstPlayerController())for(TActorIterator<AActor> I(GetWorld());I;++I)if(I->ActorHasTag(TEXT("InspectionRoof"))){if(PreviewMode==2)PC->HiddenActors.Remove(*I);else PC->HiddenActors.AddUnique(*I);}
 if(PreviewMode==1){ViewYaw=140;ViewPitch=35;ViewDistance=180;}
 if(PreviewMode==3){ViewYaw=-145;ViewPitch=63;ViewDistance=1300;}
 if(PreviewMode==4){ViewYaw=-90;ViewPitch=89.9;ViewDistance=1200;}
}
void ARobotPawn::UpdatePreview(float Dt){
 if(!Overview)return;
 if(auto* PC=GetWorld()->GetFirstPlayerController()){
  if(PC->WasInputKeyJustPressed(EKeys::One))SetPreviewMode(1);
  if(PC->WasInputKeyJustPressed(EKeys::Two))SetPreviewMode(2);
  if(PC->WasInputKeyJustPressed(EKeys::Three))SetPreviewMode(3);
  if(PC->WasInputKeyJustPressed(EKeys::Four))SetPreviewMode(4);
  if(PreviewMode!=2){
   float MX=0,MY=0;PC->GetInputMouseDelta(MX,MY);
   const bool Drag=(PC->IsInputKeyDown(EKeys::LeftMouseButton)||PC->IsInputKeyDown(EKeys::RightMouseButton))&&!PC->WasInputKeyJustPressed(EKeys::LeftMouseButton)&&!PC->WasInputKeyJustPressed(EKeys::RightMouseButton);
   const double YawStep=(PC->IsInputKeyDown(EKeys::Right)?1:0)-(PC->IsInputKeyDown(EKeys::Left)?1:0);
   const double PitchStep=(PC->IsInputKeyDown(EKeys::Up)?1:0)-(PC->IsInputKeyDown(EKeys::Down)?1:0);
   ViewYaw+=(Drag?-MX*.35:0)+YawStep*60*Dt;ViewPitch=FMath::Clamp(ViewPitch+(Drag?-MY*.35:0)+PitchStep*45*Dt,8.0,89.9);
   if(PC->WasInputKeyJustPressed(EKeys::MouseScrollUp)||PC->WasInputKeyJustPressed(EKeys::Equals))ViewDistance*=.9;
   if(PC->WasInputKeyJustPressed(EKeys::MouseScrollDown)||PC->WasInputKeyJustPressed(EKeys::Hyphen))ViewDistance/= .9;
   ViewDistance=FMath::Clamp(ViewDistance,35.0,1800.0);
  }
 }
 if(PreviewMode==2){Overview->GetCameraComponent()->SetFieldOfView(80);Overview->SetActorLocation(LeftCamera->GetComponentLocation()+Body->GetForwardVector()*2.0);Overview->SetActorRotation(Body->GetComponentRotation());}
 else{Overview->GetCameraComponent()->SetFieldOfView(60);const FVector Focus=PreviewMode==1?Body->GetComponentLocation()+FVector(0,0,5):FVector(100,0,20);const double Y=FMath::DegreesToRadians(ViewYaw),P=FMath::DegreesToRadians(ViewPitch);const FVector Offset(FMath::Cos(P)*FMath::Cos(Y),FMath::Cos(P)*FMath::Sin(Y),FMath::Sin(P));Overview->SetActorLocation(Focus+Offset*ViewDistance);Overview->SetActorRotation((Focus-Overview->GetActorLocation()).Rotation());}
}
void ARobotPawn::SendState() {
 if(!Socket)return;auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("type"),TEXT("robot_state"));O->SetNumberField(TEXT("sim_time_s"),Time);O->SetBoolField(TEXT("armed"),Drive.armed);O->SetStringField(TEXT("fault"),FaultReason);O->SetStringField(TEXT("scenario"),Scenario);
 auto S=MakeShared<FJsonObject>();S->SetNumberField(TEXT("left_ticks"),FMath::RoundToDouble(LeftTicks));S->SetNumberField(TEXT("right_ticks"),FMath::RoundToDouble(RightTicks));S->SetNumberField(TEXT("encoder_cpr"),EncoderCPR);S->SetBoolField(TEXT("bumper"),Bump);
 S->SetBoolField(TEXT("bumper_left"),BumpLeft);S->SetBoolField(TEXT("bumper_right"),BumpRight);S->SetNumberField(TEXT("bumper_events"),Drive.bump_events);S->SetStringField(TEXT("last_fault_event"),LastFaultEvent);S->SetNumberField(TEXT("last_fault_event_time_s"),LastFaultTime);
 // bump_zones: OR of every tick since the previous message (a 1-tick touch is never lost); bumper/_left/_right are this tick.
 S->SetNumberField(TEXT("bump_zones"),BumpZonesLatched);BumpZonesLatched=0;
 TArray<TSharedPtr<FJsonValue>> Ranges;for(double D:Cliff)Ranges.Add(MakeShared<FJsonValueNumber>(D));S->SetArrayField(TEXT("cliff_m"),Ranges);
 const FVector Omega=Body->GetComponentQuat().UnrotateVector(Body->GetPhysicsAngularVelocityInRadians());S->SetArrayField(TEXT("gyro_rad_s"),JVec(FVector(-Omega.X,Omega.Y,-Omega.Z)));S->SetArrayField(TEXT("specific_force_m_s2"),JVec(FVector(SpecificForce.X,-SpecificForce.Y,SpecificForce.Z)));if(LidarScan.IsValid())S->SetObjectField(TEXT("lidar"),LidarScan);O->SetObjectField(TEXT("sensors"),S);
 auto T=MakeShared<FJsonObject>();const FVector P=Body->GetComponentLocation(),V=Body->GetPhysicsLinearVelocity();const FRotator R=Body->GetComponentRotation();T->SetArrayField(TEXT("position_m"),JVec(FVector(P.X,-P.Y,P.Z-BodyHeight)/100));T->SetNumberField(TEXT("yaw_rad"),-R.Yaw*PI/180);T->SetNumberField(TEXT("pitch_deg"),R.Pitch);T->SetNumberField(TEXT("roll_deg"),R.Roll);T->SetArrayField(TEXT("velocity_m_s"),JVec(FVector(V.X,-V.Y,V.Z)/100));T->SetArrayField(TEXT("com_m"),JVec(Body->GetCenterOfMass()*FVector(1,-1,1)/100));
 const FVector ComLocal=Body->GetComponentTransform().InverseTransformPosition(Body->GetCenterOfMass());T->SetArrayField(TEXT("com_base_m"),JVec(FVector(ComLocal.X,-ComLocal.Y,ComLocal.Z+BodyHeight)/100));
 // Physics masses and the whole-robot COM (welded Body + both wheels + caster ball, mass-weighted, in the body frame) for scoring.
 if(Body->IsSimulatingPhysics()){
  const FTransform& BX=Body->GetComponentTransform();double MassSum=0;FVector Moment=FVector::ZeroVector;
  const TArray<UPrimitiveComponent*> Parts={Body,LeftWheel,RightWheel,Caster};
  for(UPrimitiveComponent* Part:Parts)if(Part&&Part->IsSimulatingPhysics()){const double M=Part->GetMass();MassSum+=M;Moment+=BX.InverseTransformPosition(Part->GetCenterOfMass())*M;}
  T->SetNumberField(TEXT("physics_body_mass_kg"),Body->GetMass());T->SetNumberField(TEXT("physics_robot_mass_kg"),MassSum);
  double BI[3][3];if(PhysicsBodyInertia(BI)){TArray<TSharedPtr<FJsonValue>> Rows3;for(int I=0;I<3;I++)Rows3.Add(MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{MakeShared<FJsonValueNumber>(BI[I][0]),MakeShared<FJsonValueNumber>(BI[I][1]),MakeShared<FJsonValueNumber>(BI[I][2])}));T->SetArrayField(TEXT("physics_body_inertia_kg_m2"),Rows3);}
  if(MassSum>0){const FVector RC=Moment/MassSum;T->SetArrayField(TEXT("robot_com_base_m"),JVec(FVector(RC.X,-RC.Y,RC.Z+BodyHeight)/100));}
 }
 O->SetObjectField(TEXT("ground_truth_evaluation_only"),T);
 if(GeometryJson.IsValid())O->SetObjectField(TEXT("sim_geometry"),GeometryJson);
 O->SetArrayField(TEXT("wheel_inertia_kg_cm2"),JVec(LeftWheel->GetInertiaTensor()));O->SetArrayField(TEXT("wheel_rate_rad_s"),{MakeShared<FJsonValueNumber>(LeftRate),MakeShared<FJsonValueNumber>(RightRate)});O->SetNumberField(TEXT("stereo_frame"),Frame);O->SetBoolField(TEXT("stereo_available"),LeftTarget!=nullptr);
 auto View=MakeShared<FJsonObject>();View->SetNumberField(TEXT("mode"),PreviewMode);View->SetNumberField(TEXT("yaw_deg"),ViewYaw);View->SetNumberField(TEXT("pitch_deg"),ViewPitch);View->SetNumberField(TEXT("distance_cm"),ViewDistance);if(Overview)View->SetArrayField(TEXT("camera_position_cm"),JVec(Overview->GetActorLocation()));O->SetObjectField(TEXT("preview"),View);
 FString Text;FJsonSerializer::Serialize(O,TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Text));FTCHARToUTF8 Utf8(*Text);int32 Sent=0;Socket->SendTo(reinterpret_cast<const uint8*>(Utf8.Get()),Utf8.Length(),Sent,*Peer);
}
void ARobotPawn::CaptureStereo() {
 LeftCamera->CaptureScene();if(!LidarMode)RightCamera->CaptureScene();Frame++;
 const FString L=FString::Printf(TEXT("left_%06d.png"),Frame),R=FString::Printf(TEXT("right_%06d.png"),Frame);
 auto Save=[&](UTextureRenderTarget2D* RT,const FString& Name){TArray<FColor> Pixels;RT->GameThread_GetRenderTargetResource()->ReadPixels(Pixels);TArray64<uint8> PNG;FImageUtils::PNGCompressImageArray(640,360,Pixels,PNG);return FFileHelper::SaveArrayToFile(PNG,*(DataDir/Name));};
 if(!Save(LeftTarget,L)||(!LidarMode&&!Save(RightTarget,R)))return;
 auto O=MakeShared<FJsonObject>();O->SetNumberField(TEXT("frame"),Frame);O->SetNumberField(TEXT("sim_time_s"),Time);O->SetStringField(TEXT("left"),L);if(!LidarMode)O->SetStringField(TEXT("right"),R);O->SetStringField(TEXT("sensor_configuration"),LidarMode?TEXT("d200_single_camera"):TEXT("stereo"));O->SetNumberField(TEXT("width"),640);O->SetNumberField(TEXT("height"),360);O->SetNumberField(TEXT("fx"),320/FMath::Tan(FMath::DegreesToRadians(LidarMode?42.0:40.0)));O->SetNumberField(TEXT("fy"),320/FMath::Tan(FMath::DegreesToRadians(LidarMode?42.0:40.0)));O->SetNumberField(TEXT("cx"),319.5);O->SetNumberField(TEXT("cy"),179.5);O->SetNumberField(TEXT("baseline_m"),LidarMode?0:.062);O->SetStringField(TEXT("model"),TEXT("ideal_pinhole_zero_distortion_synthetic_FOV"));
 O->SetNumberField(TEXT("left_ticks"),FMath::RoundToDouble(LeftTicks));O->SetNumberField(TEXT("right_ticks"),FMath::RoundToDouble(RightTicks));
 const FVector CP=LeftCamera->GetComponentLocation();O->SetArrayField(TEXT("camera_position_evaluation_only_m"),JVec(CP*FVector(1,-1,1)/100));
 auto Evaluation=MakeShared<FJsonObject>();const FVector BP=Body->GetComponentLocation();Evaluation->SetArrayField(TEXT("position_m"),JVec(FVector(BP.X,-BP.Y,BP.Z-BodyHeight)/100));Evaluation->SetNumberField(TEXT("yaw_rad"),-Body->GetComponentRotation().Yaw*PI/180);O->SetObjectField(TEXT("ground_truth_evaluation_only"),Evaluation);
 SaveObject(O,DataDir/FString::Printf(TEXT("frame_%06d.json"),Frame));SaveObject(O,DataDir/TEXT("latest.json"));
 auto Session=MakeShared<FJsonObject>();Session->SetStringField(TEXT("directory"),FPaths::GetCleanFilename(DataDir));SaveObject(Session,FPaths::ProjectSavedDir()/TEXT("Stereo/latest-session.json"));
}
// D200-class scan: clockwise rolling sweep, 4000 samples/s, ~6 revolutions/s.
// Only polar readings and timestamps cross the sensor boundary. Noise/dropout
// are explicit engineering assumptions, not a calibrated optical model.
void ARobotPawn::SampleLidar(float Dt) {
 LidarCarry+=Dt*4000.;const int Count=FMath::Min(400,(int)LidarCarry);LidarCarry-=Count;
 FCollisionQueryParams Q;Q.AddIgnoredActor(this);
 const FVector Start=Body->GetComponentTransform().TransformPosition(ToUnreal(Geo.lidar[0],Geo.lidar[1],Geo.lidar[2]));
 for(int I=0;I<Count;I++){
  if(LidarIndex==0)LidarStart=Time;
  const double Angle=-2*PI*LidarIndex/667.;const FVector Dir=Body->GetComponentQuat().RotateVector(FVector(FMath::Cos(Angle),-FMath::Sin(Angle),0));
  FHitResult Hit;bool Found=GetWorld()->LineTraceSingleByChannel(Hit,Start,Start+Dir*800,ECC_Visibility,Q);
  double Range=Found?Hit.Distance/100:-1;
  // White paint is assigned the bright test class. All other surfaces use
  // the conservative dark class. These are simulated IR material assumptions;
  // RGB colour is deliberately NOT interpreted as measured IR reflectivity.
  const bool Dark=!(Hit.GetActor()&&Hit.GetActor()->ActorHasTag(TEXT("LidarBrightTarget")));
  const double Noise=FMath::Sqrt(-2*FMath::Loge(FMath::Max(.00001f,LidarRandom.FRand())))*FMath::Cos(2*PI*LidarRandom.FRand());
  Range=D200Model::measure(Range,Dark,Noise,LidarRandom.FRand(),LidarBiasFraction,LidarJitterFraction,LidarDropout);
  LidarRanges.Add(MakeShared<FJsonValueNumber>(Range<0?-1:FMath::RoundToDouble(Range*1000)));LidarTimes.Add(MakeShared<FJsonValueNumber>(FMath::RoundToDouble((Time-LidarStart)*1000)));LidarIndex++;
  if(LidarIndex==667){
   LidarScan=MakeShared<FJsonObject>();LidarScan->SetNumberField(TEXT("frame"),++LidarFrame);LidarScan->SetNumberField(TEXT("start_time_s"),LidarStart);LidarScan->SetNumberField(TEXT("sim_time_s"),Time);LidarScan->SetNumberField(TEXT("angle_min_rad"),0);LidarScan->SetNumberField(TEXT("angle_increment_rad"),-2*PI/667.);LidarScan->SetNumberField(TEXT("range_min_m"),.1);LidarScan->SetNumberField(TEXT("range_max_m"),8);LidarScan->SetArrayField(TEXT("ranges_mm"),LidarRanges);LidarScan->SetArrayField(TEXT("time_offsets_ms"),LidarTimes);LidarScan->SetArrayField(TEXT("sensor_in_base_m"),{MakeShared<FJsonValueNumber>(Geo.lidar[0]/100),MakeShared<FJsonValueNumber>(Geo.lidar[1]/100),MakeShared<FJsonValueNumber>(Geo.lidar[2]/100)});
   LidarRanges.Empty();LidarTimes.Empty();LidarIndex=0;
  }
 }
}
void ARobotPawn::EndPlay(const EEndPlayReason::Type Reason) {
 LogSensorCost();
 if(Socket){Socket->Close();ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);Socket=nullptr;}Super::EndPlay(Reason);
}

static void BoxInWorld(UWorld* W,FVector Center,FVector Size,FColor Color,const FString& Name) {
 AActor* A=W->SpawnActor<AActor>();
#if WITH_EDITOR
A->SetActorLabel(Name);
#endif
auto* Collision=NewObject<UBoxComponent>(A);A->SetRootComponent(Collision);Collision->SetBoxExtent(Size/2);Collision->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);Collision->SetCollisionObjectType(ECC_WorldStatic);Collision->SetCollisionResponseToAllChannels(ECR_Block);Collision->RegisterComponent();Collision->SetWorldLocation(Center);
 auto* M=NewObject<UProceduralMeshComponent>(A);M->SetupAttachment(Collision);M->RegisterComponent();M->SetWorldLocation(Center);
 TArray<FVector> V;for(float Z:{-1.f,1.f})for(float Y:{-1.f,1.f})for(float X:{-1.f,1.f})V.Add(FVector(X*Size.X/2,Y*Size.Y/2,Z*Size.Z/2));
 TArray<int32> T={0,2,1,1,2,3,4,5,6,5,7,6,0,1,4,1,5,4,2,6,3,3,6,7,0,4,2,2,4,6,1,3,5,3,7,5};
 TArray<FVector> N;TArray<FProcMeshTangent> Tan;TArray<FVector2D> UV;UV.Init(FVector2D::ZeroVector,V.Num());UKismetProceduralMeshLibrary::CalculateTangentsForMesh(V,T,UV,N,Tan);TArray<FLinearColor> C;C.Init(FLinearColor::FromSRGBColor(Color),V.Num());M->CreateMeshSection_LinearColor(0,V,T,N,UV,C,Tan,false);M->SetMaterial(0,LoadObject<UMaterialInterface>(nullptr,TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial")));M->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}
// Static vertical pole (exact capsule collider, cylinder from the floor to Height) for bumper coverage tests.
static void PoleInWorld(UWorld* W,FVector Base,double Radius,double Height,FColor Color,const FString& Name) {
 AActor* A=W->SpawnActor<AActor>();
#if WITH_EDITOR
 A->SetActorLabel(Name);
#endif
 auto* Collision=NewObject<UCapsuleComponent>(A);A->SetRootComponent(Collision);Collision->SetCapsuleSize(Radius,Height/2+Radius);Collision->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);Collision->SetCollisionObjectType(ECC_WorldStatic);Collision->SetCollisionResponseToAllChannels(ECR_Block);Collision->RegisterComponent();Collision->SetWorldLocation(Base+FVector(0,0,Height/2));
 auto* M=NewObject<UProceduralMeshComponent>(A);M->SetupAttachment(Collision);M->RegisterComponent();M->SetWorldLocation(Base+FVector(0,0,Height/2));
 TArray<FVector> V;TArray<int32> T;constexpr int Sides=16;
 for(int I=0;I<Sides;I++){const double Angle=2*PI*I/Sides;for(double Z:{-Height/2,Height/2})V.Add(FVector(Radius*FMath::Cos(Angle),Radius*FMath::Sin(Angle),Z));}
 for(int I=0;I<Sides;I++){const int J=(I+1)%Sides;T.Append({2*I,2*J,2*I+1,2*J,2*J+1,2*I+1});}
 TArray<FVector> N;TArray<FProcMeshTangent> Tan;TArray<FVector2D> UV;UV.Init(FVector2D::ZeroVector,V.Num());UKismetProceduralMeshLibrary::CalculateTangentsForMesh(V,T,UV,N,Tan);TArray<FLinearColor> C;C.Init(FLinearColor::FromSRGBColor(Color),V.Num());M->CreateMeshSection_LinearColor(0,V,T,N,UV,C,Tan,false);M->SetMaterial(0,LoadObject<UMaterialInterface>(nullptr,TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial")));M->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}
AHomeRobotGameMode::AHomeRobotGameMode(){DefaultPawnClass=nullptr;}
void AHomeRobotGameMode::StartPlay() {
 Super::StartPlay();UWorld* W=GetWorld();FString Scenario=TEXT("house");FParse::Value(FCommandLine::Get(),TEXT("Scenario="),Scenario);
 const FColor Floor(176,161,138),Wall(219,223,215),Accent(50,112,115);
 if(Scenario==TEXT("house"))BuildFurnishedHouse(W);
 else {
 if(Scenario==TEXT("cliff")){BoxInWorld(W,FVector(-110,0,-5),FVector(380,600,10),Floor,TEXT("Floor ending at x=80cm"));BoxInWorld(W,FVector(300,0,-80),FVector(400,600,10),FColor(80,90,90),TEXT("Lower catch floor"));}
 else BoxInWorld(W,FVector(100,0,-5),FVector(800,600,10),Floor,TEXT("House floor 8m x 6m"));
 BoxInWorld(W,FVector(-305,0,120),FVector(10,610,240),Wall,TEXT("Rear wall"));BoxInWorld(W,FVector(505,0,120),FVector(10,610,240),Wall,TEXT("Front wall"));
 for(float Y:{-305.f,305.f})BoxInWorld(W,FVector(100,Y,120),FVector(800,10,240),Wall,TEXT("Side wall"));
 if(Scenario==TEXT("calibration")){
  // Synthetic textured rug: varied patches make stereo correspondence testable.
  for(int I=0;I<22;I++)for(int J=0;J<24;J++){const int H=(I*731+J*419+I*J*37)%251;BoxInWorld(W,FVector(-200+I*30,-280+J*24,.004),FVector(11+H%14,8+H%11,.008),FColor(70+H%105,62+H%90,53+H%75),TEXT("Floor texture patch"));}
  for(float Y:{-185.f,185.f})BoxInWorld(W,FVector(180,Y,120),FVector(10,250,240),Wall,TEXT("Doorway partition"));
  BoxInWorld(W,FVector(180,0,.5),FVector(6,120,1),FColor(115,95,75),TEXT("10 mm threshold"));
  BoxInWorld(W,FVector(-150,200,30),FVector(170,65,60),Accent,TEXT("Sofa"));BoxInWorld(W,FVector(-150,227,55),FVector(170,12,50),Accent,TEXT("Sofa back"));
  BoxInWorld(W,FVector(350,150,38),FVector(120,75,8),FColor(151,103,57),TEXT("Dining table"));for(float X:{300.f,400.f})for(float Y:{122.f,178.f})BoxInWorld(W,FVector(X,Y,17),FVector(7,7,34),FColor(100,70,45),TEXT("Table leg"));
  BoxInWorld(W,FVector(450,-180,60),FVector(60,100,120),FColor(100,113,132),TEXT("Bookcase"));
  for(int I=0;I<40;I++){float X=-270+(I%10)*75,Y=-299+(I/10%2)*598,Z=25+(I/20)*60;BoxInWorld(W,FVector(X,Y,Z),FVector(25+(I%3)*8,1,18+(I%4)*6),FColor(45+I*13%170,70+I*7%130,85+I*11%150),TEXT("Wall feature"));}
 }
 }
 if(Scenario==TEXT("obstacle"))BoxInWorld(W,FVector(75,0,25),FVector(10,60,50),Accent,TEXT("Bumper test obstacle"));
 if(Scenario==TEXT("threshold"))BoxInWorld(W,FVector(80,0,.5),FVector(6,180,1),Accent,TEXT("10 mm step"));
 if(Scenario==TEXT("bumpbench")){
  // Bumper acceptance bench (tests/run_physics.py teleports the robot between stations). Test frame: x=X/100, y=-Y/100 m.
  BoxInWorld(W,FVector(-200,197.5,.4),FVector(6,195,.8),Accent,TEXT("8 mm threshold strip"));
  BoxInWorld(W,FVector(0,197.5,.5),FVector(6,195,1),Accent,TEXT("10 mm threshold strip"));
  BoxInWorld(W,FVector(230,197.5,.45),FVector(60,195,.9),FColor(120,80,60),TEXT("9 mm rug"));
  for(int I=0;I<3;I++){const double H=2.5+I;BoxInWorld(W,FVector(-150+150*I,-100,H/2),FVector(2,2,H),FColor(200,60,40),FString::Printf(TEXT("Low box %.1f cm"),H));}
  BoxInWorld(W,FVector(300,-100,.95),FVector(2,2,1.9),FColor(200,60,40),TEXT("Low box 1.9 cm (1 mm above the 1.8 cm shell bottom)"));
  PoleInWorld(W,FVector(-150,-200,0),1,60,FColor(60,60,200),TEXT("2 cm pole"));
 }
 auto* Sun=W->SpawnActor<ADirectionalLight>(FVector(0,0,300),FRotator(-60,-30,0));Sun->GetLightComponent()->SetIntensity(4.0f);
 auto* Sky=W->SpawnActor<ASkyLight>();Sky->GetLightComponent()->SetIntensity(1.f);Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);
 W->SpawnActor<ARobotPawn>(FVector(0,0,BodyHeight+.3),FRotator::ZeroRotator);
 if(auto* C=IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS")))C->Set(60.f);
}
