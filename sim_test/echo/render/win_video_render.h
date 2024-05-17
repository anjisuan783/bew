#pragma once

#include <mutex>
#include <winsock2.h>
#include <d3d9.h>

#include "video_render_impl.h"

class RTCWinVideoRender : public RTCVideoInternalRenderImpl {
public:
    enum {
        MAX_VIDEO_WIDTH = 1920,
        MAX_VIDEO_HEIGHT = 1920,
    };

    RTCWinVideoRender(void * window, uint32_t option);
    virtual ~RTCWinVideoRender();

    RTCResult setVideoScalingMode(RTCVideoScalingMode mode) override;
    void      OnFrame(const webrtc::VideoFrame & frame) override;

protected:
    bool                Initialize(int textureWidth, int textureHeight);
    void                Uninitialize();
    void                UpdateVertex(RTCVideoRotation degree,
                                        RTCVideoScalingMode mode,
                                        int nWindowWidth, int nWindowHeight, 
                                        unsigned long ulWidth, 
                                        unsigned long ulHeight, 
                                        RECT& srcRect);
    void                SafeRendBkgnd();
    void                D3dRender(unsigned char* pRGBData, int nWidth, int nHeight, int nbdp, RTCVideoRotation degree, RTCVideoScalingMode mode);
    void                GdiRender(unsigned char* pRGBData, int nWidth, int nHeight, int nbdp, RTCVideoScalingMode mode);
    static DWORD WINAPI RenderThread(LPVOID lpParam);
    void                OnRenderThread();

private:
    std::mutex m_Lock;
    HANDLE m_hEvent;
    bool m_bRending;
    HANDLE m_hRender;
    int m_nTextureWidth;
    int m_nTextureHeight;
    RTCVideoScalingMode m_scalingMode;
    int m_nBkColorR;
    int m_nBkColorG;
    int m_nBkColorB;
    bool m_mirror;
    bool m_bInit;
    IDirect3D9* m_pD3D9;
    IDirect3DDevice9* m_pD3DDevice9;
    IDirect3DTexture9* m_pD3DTexture9;
    IDirect3DVertexBuffer9* m_pD3DVertexBuffer9;
    RECT m_srcRect;
    int m_nLastWindowWidth;
    int m_nLastWindowHeight;
    int m_nLastVideoWidth;
    int m_nLastVideoHeight;
    RTCVideoRotation m_lastRotation;
    RTCVideoScalingMode m_lastScalingMode;
    std::unique_ptr<webrtc::VideoFrame>   m_VideoFrame;
    uint32_t m_use_gdi_render;
};
