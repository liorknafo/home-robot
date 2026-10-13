#include "HouseScene.h"
#include "CoreMinimal.h"
#include "Engine/World.h"
#include "ProceduralMeshComponent.h"
#include "KismetProceduralMeshLibrary.h"
#include "PhysicsEngine/BodySetup.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/Material.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Engine/PointLight.h"
#include "Components/PointLightComponent.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#if WITH_EDITOR
#include "ProceduralMeshConversion.h"
#include "MeshDescription.h"
#include "StaticMeshCompiler.h"
#include "DistanceFieldAtlas.h"
#include "MeshCardBuild.h"
#include "StaticMeshResources.h"
#endif

void BuildFurnishedHouse(UWorld* World) {
 FString Text; TSharedPtr<FJsonObject> Scene,Library;
 const FString Folder=FPaths::ProjectContentDir()/TEXT("RobotData/");
 if(!FFileHelper::LoadFileToString(Text,*(Folder+TEXT("house-scene.json"))) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Scene))return;
 if(!FFileHelper::LoadFileToString(Text,*(Folder+TEXT("house-meshes.json"))) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Library)) {
  UE_LOG(LogTemp,Error,TEXT("Blender house mesh library is missing; house was not built."));return;
 }
 auto Assets=Library->GetObjectField(TEXT("assets"));
 auto* Default=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/RobotMaterials/M_Prop.M_Prop"));
 auto* FloorParent=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/RobotMaterials/M_Terrazzo.M_Terrazzo"));
 auto* Paint=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/RobotMaterials/M_WhitePaint.M_WhitePaint"));
 auto* Floor=FloorParent?UMaterialInstanceDynamic::Create(FloorParent,World):nullptr;
 FString Finish=TEXT("polished");FParse::Value(FCommandLine::Get(),TEXT("FloorFinish="),Finish);
 if(Floor)Floor->SetScalarParameterValue(TEXT("Roughness"),Finish==TEXT("matte")?.80f:.30f);
 TMap<FString,UMaterialInterface*> Materials;
 int32 Objects=0,Sections=0,Triangles=0;
 // Global illumination (task O, LIO-133): Lumen traces mesh distance fields, and procedural meshes have none. So each object
 // keeps its procedural mesh for collision (LiDAR, bumper, cliff and physics queries are unchanged) and is drawn by a static-mesh
 // copy of the same triangles and materials, built here with a distance field and Lumen cards (editor builds: the sim runs in
 // UnrealEditor -game). One copy per mesh asset and material set. -HouseGI=0: the old look, procedural meshes drawn, no GI.
 int32 GI=1;FParse::Value(FCommandLine::Get(),TEXT("HouseGI="),GI);
#if !WITH_EDITOR
 GI=0;
#endif
 TMap<FString,UStaticMesh*> Copies;int32 Drawn=0;const double BuildStart=FPlatformTime::Seconds();
 for(auto& Entry:Scene->GetArrayField(TEXT("objects"))) {
  auto O=Entry->AsObject();const FString Key=O->GetStringField(TEXT("mesh"));auto Asset=Assets->GetObjectField(Key);
  const auto& C=O->GetArrayField(TEXT("center"));const FVector Center(C[0]->AsNumber(),C[1]->AsNumber(),C[2]->AsNumber());
  auto* A=World->SpawnActor<AActor>();FString Surface;O->TryGetStringField(TEXT("material"),Surface);
  if(Surface==TEXT("white_paint"))A->Tags.Add(TEXT("LidarBrightTarget"));
  if(O->GetStringField(TEXT("group"))==TEXT("ceiling"))A->Tags.Add(TEXT("InspectionRoof"));
#if WITH_EDITOR
  A->SetActorLabel(O->GetStringField(TEXT("name")));
#endif
  auto* Mesh=NewObject<UProceduralMeshComponent>(A);A->SetRootComponent(Mesh);
  Mesh->SetMobility(EComponentMobility::Static);Mesh->bUseComplexAsSimpleCollision=true;Mesh->bUseAsyncCooking=false;
  const bool Collision=O->GetBoolField(TEXT("collision"));
  Mesh->SetCollisionEnabled(Collision?ECollisionEnabled::QueryAndPhysics:ECollisionEnabled::NoCollision);
  Mesh->SetCollisionObjectType(ECC_WorldStatic);Mesh->SetCollisionResponseToAllChannels(ECR_Block);
  A->SetActorLocationAndRotation(Center,FRotator(0,O->GetNumberField(TEXT("yaw")),0));Mesh->RegisterComponent();
  int32 Section=0;
  for(auto& Value:Asset->GetArrayField(TEXT("parts"))) {
   auto P=Value->AsObject();TArray<FVector> V,N;TArray<int32> T;TArray<FVector2D> UV;TArray<FProcMeshTangent> Tan;
   const auto& Vert=P->GetArrayField(TEXT("vertices"));const auto& Norm=P->GetArrayField(TEXT("normals"));const auto& Tex=P->GetArrayField(TEXT("uv"));const auto& Faces=P->GetArrayField(TEXT("triangles"));
   for(int32 I=0;I<Vert.Num();I+=3){V.Add(FVector(Vert[I]->AsNumber(),Vert[I+1]->AsNumber(),Vert[I+2]->AsNumber()));N.Add(FVector(Norm[I]->AsNumber(),Norm[I+1]->AsNumber(),Norm[I+2]->AsNumber()));}
   for(int32 I=0;I<Tex.Num();I+=2)UV.Add(FVector2D(Tex[I]->AsNumber(),Tex[I+1]->AsNumber()));
   // Blender exports counterclockwise outside faces. Unreal uses clockwise.
   for(int32 I=0;I<Faces.Num();I+=3){T.Add(Faces[I]->AsNumber());T.Add(Faces[I+2]->AsNumber());T.Add(Faces[I+1]->AsNumber());}
   TArray<FVector> CalculatedNormals;UKismetProceduralMeshLibrary::CalculateTangentsForMesh(V,T,UV,CalculatedNormals,Tan);
   TArray<FLinearColor> Colors;Colors.Init(FLinearColor::FromSRGBColor(FColor::FromHex(P->GetStringField(TEXT("color")))),V.Num());
   Mesh->CreateMeshSection_LinearColor(Section,V,T,N,UV,Colors,Tan,Collision);
   const FString Kind=P->GetStringField(TEXT("surface"));UMaterialInterface* Material=nullptr;
   if(Kind==TEXT("terrazzo"))Material=Floor;
   else if(Kind==TEXT("paint")&&Surface==TEXT("white_paint"))Material=Paint;
   else {
    if(!Materials.Contains(Kind)) {
     const FString Path=TEXT("/Game/HouseMaterials/M_")+Kind+TEXT(".M_")+Kind;
     Materials.Add(Kind,LoadObject<UMaterialInterface>(nullptr,*Path));
    }
    Material=Materials[Kind];
   }
   Mesh->SetMaterial(Section,Material?Material:Default);Triangles+=T.Num()/3;Section++;Sections++;
  }
  if(Mesh->GetBodySetup())Mesh->GetBodySetup()->bDoubleSidedGeometry=true;
#if WITH_EDITOR
  if(GI) {
   FString CopyKey=Key;for(int32 S=0;S<Mesh->GetNumSections();S++)CopyKey+=TEXT("|")+GetNameSafe(Mesh->GetMaterial(S));
   UStaticMesh*& Copy=Copies.FindOrAdd(CopyKey);
   if(!Copy) {
    // As the engine's "Create StaticMesh" for procedural meshes: polygon groups and material slots both follow the sections'
    // unique materials, in section order. Normals and tangents are kept as built above.
    Copy=NewObject<UStaticMesh>(GetTransientPackage(),NAME_None,RF_Transient);
    FStaticMeshSourceModel& Source=Copy->AddSourceModel();
    Source.BuildSettings.bRecomputeNormals=false;Source.BuildSettings.bRecomputeTangents=false;Source.BuildSettings.bRemoveDegenerates=false;Source.BuildSettings.bGenerateLightmapUVs=false;
    Copy->CreateMeshDescription(0,BuildMeshDescription(Mesh));Copy->CommitMeshDescription(0);
    TSet<UMaterialInterface*> Unique;
    for(int32 S=0;S<Mesh->GetNumSections();S++){UMaterialInterface* M=Mesh->GetMaterial(S);if(!M)M=UMaterial::GetDefaultMaterial(MD_Surface);if(!Unique.Contains(M)){Unique.Add(M);Copy->GetStaticMaterials().Add(FStaticMaterial(M));}}
    // Section i draws material i. Without these entries the distance-field and Lumen-card builders skip every triangle (they
    // take each section's material and flags from this map), and Lumen then has nothing to bounce light from.
    for(int32 M=0;M<Copy->GetStaticMaterials().Num();M++){FMeshSectionInfo Info(M);Info.bEnableCollision=false;Info.bCastShadow=true;Copy->GetSectionInfoMap().Set(0,M,Info);}
    Copy->SetImportVersion(EImportStaticMeshVersion::LastVersion);Copy->Build(true);
   }
   auto* Render=NewObject<UStaticMeshComponent>(A);Render->SetMobility(EComponentMobility::Static);Render->SetCollisionEnabled(ECollisionEnabled::NoCollision);
   Render->SetStaticMesh(Copy);Render->SetupAttachment(Mesh);Render->RegisterComponent();
   Mesh->SetVisibility(false);Drawn++;   // collision only from here on: not drawn, casts no shadow
  }
#endif
  Objects++;
 }
 if(FParse::Param(FCommandLine::Get(),TEXT("ValidateHouseAssets"))) {
  struct Probe { const TCHAR* Name; FVector Start,End; float Minimum,Maximum; int Axis; };
  TArray<Probe> Probes={
   {TEXT("structural_floor"),FVector(0,0,30),FVector(0,0,-20),-.05f,.05f,2},
   {TEXT("slipper_open_heel"),FVector(-139,18,15),FVector(-139,18,-2),.7f,1.3f,2},
   {TEXT("slipper_closed_upper"),FVector(-124,18,15),FVector(-124,18,-2),4.f,6.3f,2},
   {TEXT("bookcase_open_gap"),FVector(260,134.2,22),FVector(312,134.2,22),307.f,310.f,0},
   {TEXT("chair_leg_surface"),FVector(357,-102.6,20),FVector(370,-102.6,20),362.f,365.f,0}
  };
  auto Report=MakeShared<FJsonObject>();TArray<TSharedPtr<FJsonValue>> Results;bool Passed=true;
  for(const auto& P:Probes){FHitResult Hit;FCollisionQueryParams Params(SCENE_QUERY_STAT(HouseAssets),false);
   const bool Found=World->LineTraceSingleByChannel(Hit,P.Start,P.End,ECC_Visibility,Params);
   const float Value=Hit.ImpactPoint[P.Axis];const bool OK=Found&&Value>=P.Minimum&&Value<=P.Maximum;Passed&=OK;
   auto Row=MakeShared<FJsonObject>();Row->SetStringField(TEXT("probe"),P.Name);Row->SetBoolField(TEXT("passed"),OK);Row->SetBoolField(TEXT("hit"),Found);Row->SetNumberField(TEXT("surface_coordinate_cm"),Value);Row->SetNumberField(TEXT("expected_min_cm"),P.Minimum);Row->SetNumberField(TEXT("expected_max_cm"),P.Maximum);Results.Add(MakeShared<FJsonValueObject>(Row));
  }
  Report->SetBoolField(TEXT("passed"),Passed);Report->SetStringField(TEXT("scope"),TEXT("Static geometry query checks only; not an autonomous mapping or robot contact test"));Report->SetArrayField(TEXT("probes"),Results);
  FString Output;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Output));FFileHelper::SaveStringToFile(Output,*(FPaths::ProjectDir()/TEXT("reports/house-native-collision.json")));
 }
 UE_LOG(LogTemp,Display,TEXT("BLENDER_HOUSE_READY objects=%d sections=%d triangles=%d collision=visible_static_surfaces"),Objects,Sections,Triangles);
#if WITH_EDITOR
 // Wait for the copies, their distance fields and Lumen cards before the robot starts (cached in the DDC after the first run).
 int32 WithField=0,WithCards=0;SIZE_T FieldBytes=0;
 if(GI){FStaticMeshCompilingManager::Get().FinishAllCompilation();if(GDistanceFieldAsyncQueue)GDistanceFieldAsyncQueue->BlockUntilAllBuildsComplete();if(GCardRepresentationAsyncQueue)GCardRepresentationAsyncQueue->BlockUntilAllBuildsComplete();
  for(auto& Pair:Copies){const FStaticMeshRenderData* Data=Pair.Value->GetRenderData();if(!Data||!Data->LODResources.Num())continue;const FStaticMeshLODResources& Lod=Data->LODResources[0];
   if(Lod.DistanceFieldData&&Lod.DistanceFieldData->Mips[0].IndirectionDimensions.GetMax()>0){WithField++;FieldBytes+=Lod.DistanceFieldData->GetResourceSizeBytes();}
   if(Lod.CardRepresentationData&&Lod.CardRepresentationData->MeshCardsBuildData.CardBuildData.Num()>0)WithCards++;}
  UE_LOG(LogTemp,Display,TEXT("HOUSE_GI copies with a distance field %d / %d (%.1f MB), with Lumen cards %d"),WithField,Copies.Num(),FieldBytes/1048576.0,WithCards);}
#endif
 if(auto* Method=IConsoleManager::Get().FindConsoleVariable(TEXT("r.DynamicGlobalIlluminationMethod")))Method->Set(GI?1:0,ECVF_SetByCode);
 UE_LOG(LogTemp,Display,TEXT("HOUSE_GI %s: %d objects drawn by %d static copies (distance fields, Lumen cards) in %.1f s; r.DynamicGlobalIlluminationMethod=%d"),GI?TEXT("lumen"):TEXT("off"),Drawn,Copies.Num(),FPlatformTime::Seconds()-BuildStart,GI?1:0);
 for(FVector Position:{FVector(-95,-90,223),FVector(270,-90,223),FVector(-120,205,223),FVector(208,183,223),FVector(410,170,223)}) {
  auto* Lamp=World->SpawnActor<APointLight>(Position,FRotator::ZeroRotator);
  auto* Light=Cast<UPointLightComponent>(Lamp->GetLightComponent());Light->SetMobility(EComponentMobility::Movable);Light->SetUseInverseSquaredFalloff(false);Light->SetLightFalloffExponent(2.f);Light->SetIntensity(6.f);Light->SetAttenuationRadius(470.f);Light->SetLightColor(FLinearColor(1.f,.96f,.90f));Light->SetSourceRadius(18.f);Light->SetSoftSourceRadius(25.f);
 }
}
