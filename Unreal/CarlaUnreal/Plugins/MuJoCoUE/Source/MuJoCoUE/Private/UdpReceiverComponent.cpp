#include "UdpReceiverComponent.h"
#include "Async/Async.h"
#include "SocketSubsystem.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Common/UdpSocketBuilder.h"
#include "Common/UdpSocketReceiver.h"
#include "robot_sdk.pb.h"

DEFINE_LOG_CATEGORY_STATIC(LogUdpReceiver, Log, All);

UUdpReceiverComponent::UUdpReceiverComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	bWantsInitializeComponent = false;
	bAutoActivate = true;
}

void UUdpReceiverComponent::SetParseMode(EUdpParseMode Mode)
{
	ParseMode = Mode;
	// Adjust default port if it was still the old default
	if (Mode == EUdpParseMode::ParseRobotState && ListenPort == 25002)
	{
		ListenPort = 25001;
	}
	else if (Mode == EUdpParseMode::ParseRobotCmd && ListenPort == 25001)
	{
		ListenPort = 25002;
	}
}

void UUdpReceiverComponent::BeginPlay()
{
	Super::BeginPlay();

	if (bAutoStart)
	{
		StartListening();
	}
}

void UUdpReceiverComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopListening();
	Super::EndPlay(EndPlayReason);
}

bool UUdpReceiverComponent::StartListening()
{
	if (bIsListening)
	{
		UE_LOG(LogUdpReceiver, Warning,
			TEXT("UdpReceiver: Already listening on port %d."), ListenPort);
		return false;
	}

	FIPv4Address Addr;
	if (!FIPv4Address::Parse(ListenIP, Addr))
	{
		UE_LOG(LogUdpReceiver, Error,
			TEXT("UdpReceiver: Invalid listen IP '%s'"), *ListenIP);
		return false;
	}

	FIPv4Endpoint Endpoint(Addr, ListenPort);

	FString SocketName = (ParseMode == EUdpParseMode::ParseRobotState)
		? TEXT("MujocoSimStateReceiver")
		: TEXT("McCtrlCmdReceiver");

	ReceiverSocket = FUdpSocketBuilder(*SocketName)
		.AsNonBlocking()
		.AsReusable()
		.BoundToEndpoint(Endpoint)
		.WithReceiveBufferSize(65536);

	if (!ReceiverSocket)
	{
		UE_LOG(LogUdpReceiver, Error,
			TEXT("UdpReceiver: Failed to create socket on %s:%d"), *ListenIP, ListenPort);
		return false;
	}

	FTimespan ThreadWaitTime = FTimespan::FromMilliseconds(1);
	FString ThreadName = FString::Printf(TEXT("UDPRecv-%d"), ListenPort);
	UdpReceiver = new FUdpSocketReceiver(ReceiverSocket, ThreadWaitTime, *ThreadName);

	UdpReceiver->OnDataReceived().BindLambda([this](const FArrayReaderPtr& DataPtr, const FIPv4Endpoint& Ep)
	{
		if (!DataPtr.IsValid() || DataPtr->TotalSize() == 0)
		{
			return;
		}

		TArray<uint8> RawData;
		RawData.AddUninitialized(DataPtr->TotalSize());
		DataPtr->Serialize(RawData.GetData(), DataPtr->TotalSize());

		FString SenderIP = Ep.Address.ToString();
		int32 SenderPort = Ep.Port;

		HandleDataReceived(RawData, SenderIP, SenderPort);
	});
	UdpReceiver->Start();

	bIsListening = true;

	const TCHAR* ModeStr = (ParseMode == EUdpParseMode::ParseRobotState) ? TEXT("RobotState") : TEXT("RobotCmd");
	UE_LOG(LogUdpReceiver, Log,
		TEXT("UdpReceiver: Listening on port %d for Protobuf %s."), ListenPort, ModeStr);

	return true;
}

void UUdpReceiverComponent::StopListening()
{
	if (!bIsListening)
	{
		return;
	}

	if (UdpReceiver)
	{
		UdpReceiver->Stop();
		delete UdpReceiver;
		UdpReceiver = nullptr;
	}

	if (ReceiverSocket)
	{
		ReceiverSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ReceiverSocket);
		ReceiverSocket = nullptr;
	}

	bIsListening = false;
	UE_LOG(LogUdpReceiver, Log, TEXT("UdpReceiver: Stopped."));
}

void UUdpReceiverComponent::HandleDataReceived(const TArray<uint8>& Data, const FString& SenderIP, int32 SenderPort)
{
	TotalPacketsReceived++;

	if (ParseMode == EUdpParseMode::ParseRobotState)
	{
		// ---- RobotState mode: parse on receive thread, protect with mutex ----
		FUdpStateData StateData;
		if (ParseProtobufState(Data, StateData))
		{
			StateData.ReceiveTime = FPlatformTime::Seconds();
			StateData.bValid = true;

			{
				FScopeLock Lock(&StateMutex);
				LastState = StateData;
			}
			TotalStatesParsed++;
			OnStateReceived.Broadcast(StateData);
		}
	}
	else
	{
		// ---- RobotCmd mode: optionally marshal to game thread ----
		if (bProcessOnGameThread)
		{
			AsyncTask(ENamedThreads::GameThread, [this, Data, SenderPort]()
			{
				FUdpCommandData CmdData;
				if (ParseProtobufCommand(Data, CmdData))
				{
					CmdData.ReceiveTime = FPlatformTime::Seconds();
					CmdData.bValid = true;
					LastCommand = CmdData;
					TotalCommandsParsed++;
					OnCommandReceived.Broadcast(CmdData);
				}
			});
		}
		else
		{
			FUdpCommandData CmdData;
			if (ParseProtobufCommand(Data, CmdData))
			{
				CmdData.ReceiveTime = FPlatformTime::Seconds();
				CmdData.bValid = true;
				LastCommand = CmdData;
				TotalCommandsParsed++;
				OnCommandReceived.Broadcast(CmdData);
			}
		}
	}
}

FUdpStateData UUdpReceiverComponent::GetLatestState()
{
	FScopeLock Lock(&StateMutex);
	return LastState;
}

// ==================== RobotCmd Parsing ====================

bool UUdpReceiverComponent::ParseProtobufCommand(const TArray<uint8>& Data, FUdpCommandData& OutData)
{
	// Deserialize protobuf RobotCmd
	robot_sdk::pb::RobotCmd Cmd;
	if (!Cmd.ParseFromArray(Data.GetData(), Data.Num()))
	{
		UE_LOG(LogUdpReceiver, Verbose,
			TEXT("UdpReceiver: Protobuf RobotCmd parse failed (%d bytes)."), Data.Num());
		return false;
	}

	// Validate minimum field presence (at least q_des should have 4 elements)
	if (Cmd.q_des_abad_size() < 4 || Cmd.q_des_hip_size() < 4 || Cmd.q_des_knee_size() < 4)
	{
		UE_LOG(LogUdpReceiver, Verbose,
			TEXT("UdpReceiver: Incomplete RobotCmd (abad=%d, hip=%d, knee=%d)."),
			Cmd.q_des_abad_size(), Cmd.q_des_hip_size(), Cmd.q_des_knee_size());
		return false;
	}

	// Parse joint targets: [abad×4, hip×4, knee×4] = 12 joints
	OutData.JointTargets.SetNum(12);
	OutData.JointVelocities.SetNum(12);
	OutData.KpValues.SetNum(12);
	OutData.KdValues.SetNum(12);
	OutData.TauFF.SetNum(12);

	// ABAD joints (indices 0-3)
	for (int32 i = 0; i < 4; i++)
	{
		OutData.JointTargets[i] = Cmd.q_des_abad(i);
		OutData.JointVelocities[i] = (Cmd.qd_des_abad_size() > i) ? Cmd.qd_des_abad(i) : 0.0f;
		OutData.KpValues[i] = (Cmd.kp_abad_size() > i) ? Cmd.kp_abad(i) : 0.0f;
		OutData.KdValues[i] = (Cmd.kd_abad_size() > i) ? Cmd.kd_abad(i) : 0.0f;
		OutData.TauFF[i] = (Cmd.tau_abad_ff_size() > i) ? Cmd.tau_abad_ff(i) : 0.0f;
	}
	// HIP joints (indices 4-7)
	for (int32 i = 0; i < 4; i++)
	{
		OutData.JointTargets[4 + i] = Cmd.q_des_hip(i);
		OutData.JointVelocities[4 + i] = (Cmd.qd_des_hip_size() > i) ? Cmd.qd_des_hip(i) : 0.0f;
		OutData.KpValues[4 + i] = (Cmd.kp_hip_size() > i) ? Cmd.kp_hip(i) : 0.0f;
		OutData.KdValues[4 + i] = (Cmd.kd_hip_size() > i) ? Cmd.kd_hip(i) : 0.0f;
		OutData.TauFF[4 + i] = (Cmd.tau_hip_ff_size() > i) ? Cmd.tau_hip_ff(i) : 0.0f;
	}
	// KNEE joints (indices 8-11)
	for (int32 i = 0; i < 4; i++)
	{
		OutData.JointTargets[8 + i] = Cmd.q_des_knee(i);
		OutData.JointVelocities[8 + i] = (Cmd.qd_des_knee_size() > i) ? Cmd.qd_des_knee(i) : 0.0f;
		OutData.KpValues[8 + i] = (Cmd.kp_knee_size() > i) ? Cmd.kp_knee(i) : 0.0f;
		OutData.KdValues[8 + i] = (Cmd.kd_knee_size() > i) ? Cmd.kd_knee(i) : 0.0f;
		OutData.TauFF[8 + i] = (Cmd.tau_knee_ff_size() > i) ? Cmd.tau_knee_ff(i) : 0.0f;
	}

	return true;
}

// ==================== RobotState Parsing ====================

bool UUdpReceiverComponent::ParseProtobufState(const TArray<uint8>& Data, FUdpStateData& OutData)
{
	// Deserialize protobuf RobotState
	robot_sdk::pb::RobotState StateMsg;
	if (!StateMsg.ParseFromArray(Data.GetData(), Data.Num()))
	{
		UE_LOG(LogUdpReceiver, Verbose,
			TEXT("UdpReceiver: Protobuf RobotState parse failed (%d bytes)."), Data.Num());
		return false;
	}

	// Validate minimum field presence
	if (StateMsg.q_abad_size() < 4 || StateMsg.q_hip_size() < 4 || StateMsg.q_knee_size() < 4)
	{
		UE_LOG(LogUdpReceiver, Verbose,
			TEXT("UdpReceiver: Incomplete RobotState (abad=%d, hip=%d, knee=%d)."),
			StateMsg.q_abad_size(), StateMsg.q_hip_size(), StateMsg.q_knee_size());
		return false;
	}

	// Joint positions
	OutData.QAbad.SetNum(4);
	OutData.QHip.SetNum(4);
	OutData.QKnee.SetNum(4);
	for (int32 i = 0; i < 4; i++)
	{
		OutData.QAbad[i] = StateMsg.q_abad(i);
		OutData.QHip[i] = StateMsg.q_hip(i);
		OutData.QKnee[i] = StateMsg.q_knee(i);
	}

	// Joint velocities
	OutData.QdAbad.SetNum(4);
	OutData.QdHip.SetNum(4);
	OutData.QdKnee.SetNum(4);
	for (int32 i = 0; i < 4; i++)
	{
		OutData.QdAbad[i] = (StateMsg.qd_abad_size() > i) ? StateMsg.qd_abad(i) : 0.0f;
		OutData.QdHip[i] = (StateMsg.qd_hip_size() > i) ? StateMsg.qd_hip(i) : 0.0f;
		OutData.QdKnee[i] = (StateMsg.qd_knee_size() > i) ? StateMsg.qd_knee(i) : 0.0f;
	}

	// Quaternion [w, x, y, z]
	OutData.Quat.SetNum(4);
	for (int32 i = 0; i < 4; i++)
	{
		OutData.Quat[i] = (StateMsg.quat_size() > i) ? StateMsg.quat(i) : (i == 0 ? 1.0f : 0.0f);
	}

	// Position [x, y, z]
	OutData.Position.SetNum(3);
	for (int32 i = 0; i < 3; i++)
	{
		OutData.Position[i] = (StateMsg.position_size() > i) ? StateMsg.position(i) : 0.0f;
	}

	// Gyroscope
	OutData.Gyro.SetNum(3);
	for (int32 i = 0; i < 3; i++)
	{
		OutData.Gyro[i] = (StateMsg.gyro_size() > i) ? StateMsg.gyro(i) : 0.0f;
	}

	// Accelerometer
	OutData.Acc.SetNum(3);
	for (int32 i = 0; i < 3; i++)
	{
		OutData.Acc[i] = (StateMsg.acc_size() > i) ? StateMsg.acc(i) : 0.0f;
	}

	return true;
}
