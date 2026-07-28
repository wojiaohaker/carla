// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class Protobuf : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    private string ThirdPartyPath
    {
        get { return Path.Combine(ModulePath, "ThirdParty/"); }
    }

    public Protobuf(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[] { "Core" }
        );

        // RTTI 设置 (protobuf 需要 RTTI)
        if (Target.bForceEnableRTTI)
        {
            bUseRTTI = true;
            PublicDefinitions.Add("GOOGLE_PROTOBUF_NO_RTTI=0");
        }
        else
        {
            bUseRTTI = false;
            PublicDefinitions.Add("GOOGLE_PROTOBUF_NO_RTTI=1");
        }

        bEnableExceptions = true;
        UndefinedIdentifierWarningLevel = WarningLevel.Off;

        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            // Linux: 使用本地构建的 protobuf 3.12.4 静态库
            string LinuxPath = Path.GetFullPath(Path.Combine(ModulePath, "../ThirdParty/Linux"));
            PublicSystemIncludePaths.Add(Path.Combine(LinuxPath, "include"));
            PublicAdditionalLibraries.Add(Path.Combine(LinuxPath, "lib/libprotobuf.a"));
            PublicDefinitions.Add("HAVE_PTHREAD");
        }
        else if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // Windows: 使用插件自带的 protobuf 3.15.4 源码
            string PluginRoot = Path.GetFullPath(Path.Combine(ModulePath, "../../.."));
            PublicSystemIncludePaths.Add(Path.Combine(PluginRoot, "ThirdParty_Win"));
            PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS");
        }
        else
        {
            // 其他平台: 也尝试系统 protobuf
            PublicSystemIncludePaths.Add("/usr/include");
            PublicAdditionalLibraries.Add("protobuf");
            PublicDefinitions.Add("HAVE_PTHREAD");
        }
    }
}
