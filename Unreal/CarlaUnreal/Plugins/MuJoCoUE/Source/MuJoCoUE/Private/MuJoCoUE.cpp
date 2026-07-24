// Copyright Epic Games, Inc. All Rights Reserved.

#include "MuJoCoUE.h"
#include "Interfaces/IPluginManager.h"

#define LOCTEXT_NAMESPACE "FMuJoCoUEModule"

void FMuJoCoUEModule::StartupModule()
{
	FString PluginDir = IPluginManager::Get().FindPlugin("MuJoCoUE")->GetBaseDir();

#if PLATFORM_WINDOWS
	FString LibName = TEXT("mujoco.dll");
#elif PLATFORM_LINUX
	FString LibName = TEXT("libmujoco.so");
#else
	FString LibName = TEXT("libmujoco.dylib");
#endif

	FString DLLPath = FPaths::Combine(PluginDir, TEXT("Binaries"), FPlatformProcess::GetBinariesSubdirectory(), LibName);

	if (FPaths::FileExists(DLLPath))
	{
		DLLHandle = FPlatformProcess::GetDllHandle(*DLLPath);
		if (!DLLHandle)
		{
			UE_LOG(LogTemp, Error, TEXT("MuJoCoUE: Failed to load library: %s"), *DLLPath);
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("MuJoCoUE: Library not found at path: %s"), *DLLPath);
	}
}

void FMuJoCoUEModule::ShutdownModule()
{
	if (DLLHandle)
	{
		FPlatformProcess::FreeDllHandle(DLLHandle);
		DLLHandle = nullptr;
	}
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FMuJoCoUEModule, MuJoCoUE)