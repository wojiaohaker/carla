// Copyright Epic Games, Inc. All Rights Reserved.
using System;
using System.IO;
using UnrealBuildTool;

public class MuJoCoUE : ModuleRules
{
	public MuJoCoUE(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);


		PrivateIncludePaths.AddRange(
			new string[] {
				Path.Combine(ModuleDirectory, "Proto"),
			}
			);


		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core", "ProceduralMeshComponent",
				// ... add other public dependencies that you statically link with here ...
			}
			);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"Projects",
				"Sockets",
				"Networking",
				"Json",
				"JsonUtilities",
				"Protobuf",
			}
			);


		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);


		string MUJOCO_ROOT = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../Source/mujoco/"));
		string MUJOCO_INCLUDE_PATH = Path.Combine(MUJOCO_ROOT, "include/");
		PublicIncludePaths.Add(MUJOCO_INCLUDE_PATH);

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			string MujocoLibDir = Path.Combine(MUJOCO_ROOT, "lib", "Win64");
			string UE4BinDirectory = Path.Combine(PluginDirectory, "Binaries", "Win64");

			// Link against the import library
			PublicAdditionalLibraries.Add(Path.Combine(MujocoLibDir, "mujoco.lib"));

			// Copy mujoco.dll to plugin Binaries/Win64 if not present
			string DLLSourcePath = Path.Combine(MujocoLibDir, "mujoco.dll");
			string DLLTargetPath = Path.Combine(UE4BinDirectory, "mujoco.dll");
			if (!File.Exists(DLLTargetPath))
			{
				Directory.CreateDirectory(UE4BinDirectory);
				Console.WriteLine("[MuJoCoUE] Copy " + DLLSourcePath + " -> " + DLLTargetPath);
				File.Copy(DLLSourcePath, DLLTargetPath, false);
			}

			RuntimeDependencies.Add(DLLTargetPath);
			PublicDelayLoadDLLs.Add("mujoco.DLL");
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			string MujocoLibDir = Path.Combine(MUJOCO_ROOT, "lib", "Linux");
			string UE4BinDirectory = Path.Combine(PluginDirectory, "Binaries", "Linux");

			// Link against the shared library
			PublicAdditionalLibraries.Add(Path.Combine(MujocoLibDir, "libmujoco.so"));

			// Protobuf 3.12.4 静态库 (与 mc_ctrl 通信)
			string ProtobufLinuxDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../Protobuf/Source/ThirdParty/Linux/lib"));
			PublicAdditionalLibraries.Add(Path.Combine(ProtobufLinuxDir, "libprotobuf.a"));

			// Copy libmujoco.so (all versions) to plugin Binaries/Linux
			string SoVersioned = Path.Combine(MujocoLibDir, "libmujoco.so.3.3.0");
			string SoUnversioned = Path.Combine(MujocoLibDir, "libmujoco.so");
			string TargetVersioned = Path.Combine(UE4BinDirectory, "libmujoco.so.3.3.0");
			string TargetUnversioned = Path.Combine(UE4BinDirectory, "libmujoco.so");
			if (!File.Exists(TargetVersioned))
			{
				Directory.CreateDirectory(UE4BinDirectory);
				Console.WriteLine("[MuJoCoUE] Copy " + SoVersioned + " -> " + TargetVersioned);
				File.Copy(SoVersioned, TargetVersioned, false);
			}
			if (!File.Exists(TargetUnversioned))
			{
				File.Copy(SoUnversioned, TargetUnversioned, false);
			}

			RuntimeDependencies.Add(TargetVersioned);
			RuntimeDependencies.Add(TargetUnversioned);
		}
	}
}
