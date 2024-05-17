#pragma once

#include <stdint.h>

enum RTCEngineErrorType {
    kNoError                    = 0,
    
    // Base error
    kErrorFatal                 = -1,
    kErrorOutOfMemory           = -2,
    kErrorInvalidParam          = -3,
    kErrorSDKFail               = -4,
    kErrorNotSupport            = -5,
    kErrorInvalidState          = -6,
    kErrorLackOfResource        = -7,
    kErrorTransmitPendding      = -8,
    
    // Session error
    kErrorInvalidAppKey         = -100,
    kErrorInvalidServerURI      = -101,
    kErrorSessionNotFound       = -102,
    kErrorSessionCreateFail     = -103,
    kErrorSessionDisconnect     = -104,
    kErrorInvalidVideoProfile   = -105,
    kErrorInvalidAECMode        = -106,
    kErrorMediaThreadCreateFail = -107,
    kErrorMediaModuleCreateFail = -108,
    kErrorMediaFactoryCreateFail        = -109,
    kErrorWorkThreadStartFail   = -110,
    kErrorInvalidProxyType      = -111,
    kErrorInvalidProxyAddress   = -112,
    
    // Room error
    kErrorInvalidRoomID         = -200,
    kErrorInvalidUserID         = -201,
    kErrorConnectionNotFound    = -202,
    kErrorConnectionCreateFail  = -203,
    kErrorConnectionBecomeFail  = -204,
    kErrorStreamNotFound        = -205,
    kErrorStreamCreateFail      = -206,
    kErrorTrackNotFound         = -207,
    kErrorTrackCreateFail       = -208,
    kErrorJoinRefuse            = -209,
    kErrorJoinedAnotherRoom     = -210,
    kErrorNotJoinedRoom         = -211,
    kErrorRepeatedlyLeaveRoom   = -212,
    kErrorUserNotFound          = -213,
    kErrorQueryMediaServerFail  = -214,
    kErrorRequestJoinRoomFail   = -215,
    kErrorSourceNotFound        = -216,
    kErrorUserMediaNotStart     = -217,
    
    // Device error
    kErrorDeviceNotFound        = -500,
    KErrorInvalidDeviceType     = -501,
    kErrorInvalidDeviceIndex    = -502,
    kErrorDeviceNotifierRegisterFail    = -503,
    kErrorDeviceNotifierUnregisterFail  = -504,
    kErrorCreateDeviceSourceFail        = -505,
    kErrorInvalidDeviceSourceID = -506,
    kErrorDevicePreviewAlreadyStarted   = -507,
    
    // Render error
    kErrorInvalidRender         = -600,
    kErrorRenderCreateFail      = -601,
    
    // Dump error
    kErrorInvalidDumpFile       = -700,
    kErrorCreateDumpFileFail    = -701,
    kErrorCloseDumpFileFail     = -702,
    kErrorStartAudioDumpFail    = -703,
    kErrorStopAudioDumpFail     = -704,
    
    // External capturer error
    kErrorInvalidExternalCapturer       = -800,
    kErrorInvalidExternalCapturerFormat = -801,
};

typedef int32_t RTCResult;
