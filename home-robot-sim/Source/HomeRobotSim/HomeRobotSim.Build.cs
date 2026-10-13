using UnrealBuildTool;
public class HomeRobotSim : ModuleRules {
 public HomeRobotSim(ReadOnlyTargetRules Target) : base(Target) {
 PCHUsage=PCHUsageMode.UseExplicitOrSharedPCHs;
 PublicDependencyModuleNames.AddRange(new string[]{"Core","CoreUObject","Engine","InputCore","PhysicsCore","ProceduralMeshComponent","Sockets","Networking","Json","JsonUtilities","RenderCore","RHI","ImageWrapper","MeshDescription","StaticMeshDescription"});

 }
}
