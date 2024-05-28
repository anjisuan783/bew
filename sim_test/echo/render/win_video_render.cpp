
#include "win_video_render.h"

#include <algorithm>
#include "libyuv.h"


#define MAX_VIDEO_WIDTH 1920
#define MAX_VIDEO_HEIGHT 1920
#define D3DFVF_CUSTOMVERTEX (D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX1)

#undef max
#undef min

typedef struct {
    FLOAT x, y, z;
    FLOAT rhw;
    D3DCOLOR diffuse;
    FLOAT tu, tv;
} CUSTOMVERTEX;

RTCWinVideoRender::RTCWinVideoRender(void * windows, uint32_t option) 
    : RTCVideoInternalRenderImpl(windows, option) {
    m_hEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    m_bRending = true;
    DWORD dwThreadId = 0;
    m_hRender = CreateThread(nullptr, 0, RenderThread, this, 0, &dwThreadId);
    m_nBkColorR = 0;
    m_nBkColorG = 0;
    m_nBkColorB = 0;
    m_mirror = false;
    m_bInit = false;
    m_scalingMode = kVideoScaleFit;
    m_lastRotation = kVideoRotation_0;
    m_lastScalingMode = kVideoScaleFit;
    m_pD3D9 = nullptr;
    m_pD3DDevice9 = nullptr;
    m_pD3DTexture9 = nullptr;
    m_pD3DVertexBuffer9 = nullptr;
    m_nTextureWidth = 0;
    m_nTextureHeight = 0;
    m_nLastVideoWidth = 0;
    m_nLastVideoHeight = 0;
    m_nLastWindowWidth = 0;
    m_nLastWindowHeight = 0;
}

RTCWinVideoRender::~RTCWinVideoRender() {
    if (m_hRender)
    {
        m_bRending = false;
        SetEvent(m_hEvent);
        WaitForSingleObject(m_hRender, INFINITE);
        CloseHandle(m_hRender);
    }
    if (m_hEvent) {
        CloseHandle(m_hEvent);
    }
}

RTCResult RTCWinVideoRender::setVideoScalingMode(RTCVideoScalingMode mode) {
    m_scalingMode = mode;
    return kNoError;
}

void RTCWinVideoRender::OnFrame(const webrtc::VideoFrame& frame) {
    HWND hWnd = (HWND)m_window;
    if (nullptr == hWnd || !::IsWindow(hWnd) || !::IsWindowVisible(hWnd))
        return;
    RECT wndRt;
    GetClientRect(hWnd, &wndRt);
    int nWindowWidth = wndRt.right - wndRt.left;
    int nWindowHeight = wndRt.bottom - wndRt.top;
    if (nWindowWidth <= 0 || nWindowHeight <= 0)
        return;

    if (frame.width() > MAX_VIDEO_WIDTH || frame.height() > MAX_VIDEO_HEIGHT)
        return;
    {
        std::lock_guard<std::mutex> lock_guard(m_Lock);
        m_VideoFrame.reset(new webrtc::VideoFrame(frame));
    }
    SetEvent(m_hEvent);
}

bool RTCWinVideoRender::Initialize(int textureWidth, int textureHeight)
{
    if (m_gdi_render) {
        return false;
    }

    LOG_T(LS_INFO) << "RTCWinVideoRender::Initialize: textureWidth =" << textureWidth << ", textureHeight = " << textureHeight;
    if (nullptr == m_window) {
        return false;
    }
    HWND hWnd = (HWND)m_window;
    m_nTextureWidth = textureWidth;
    m_nTextureHeight = textureHeight;
    m_pD3D9 = Direct3DCreate9(D3D_SDK_VERSION);
    if (nullptr == m_pD3D9) {
        return false;
    }
    D3DDISPLAYMODE displayMode;
    HRESULT hr = m_pD3D9->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &displayMode);
    if (SUCCEEDED(hr))
    {
    }
    int behaviorFlag = D3DCREATE_SOFTWARE_VERTEXPROCESSING | D3DCREATE_MULTITHREADED;
    D3DCAPS9 d3dCaps;
    hr = m_pD3D9->GetDeviceCaps(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, &d3dCaps);
    if (SUCCEEDED(hr))
    {
        if (d3dCaps.DevCaps & D3DDEVCAPS_HWTRANSFORMANDLIGHT)
            behaviorFlag = D3DCREATE_HARDWARE_VERTEXPROCESSING;
    }
    D3DPRESENT_PARAMETERS presentParam = {};
    presentParam.BackBufferCount = 1;
    presentParam.BackBufferFormat = D3DFMT_X8R8G8B8;
    presentParam.BackBufferWidth = m_nTextureWidth;
    presentParam.BackBufferHeight = m_nTextureHeight;
    presentParam.SwapEffect = D3DSWAPEFFECT_COPY;
    presentParam.hDeviceWindow = hWnd;
    presentParam.Windowed = TRUE;
    presentParam.Flags = D3DPRESENTFLAG_VIDEO;
    hr = m_pD3D9->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, nullptr, behaviorFlag, &presentParam, &m_pD3DDevice9);
    if (FAILED(hr))
    {
        return false;
    }
    hr = m_pD3DDevice9->CreateTexture(m_nTextureWidth, m_nTextureHeight, 1, 0, D3DFMT_X8R8G8B8, D3DPOOL_MANAGED, &m_pD3DTexture9, nullptr);
    if (FAILED(hr))
    {
        return false;
    }
    hr = m_pD3DDevice9->CreateVertexBuffer(4 * sizeof(CUSTOMVERTEX), 0, D3DFVF_CUSTOMVERTEX, D3DPOOL_MANAGED, &m_pD3DVertexBuffer9, nullptr);
    if (FAILED(hr))
    {
        return false;
    }
    RECT rt;
    GetClientRect(hWnd, &rt);
    UpdateVertex(kVideoRotation_0, m_scalingMode, rt.right - rt.left, rt.bottom - rt.top, m_nTextureWidth, m_nTextureHeight, m_srcRect);
    return true;
}

void RTCWinVideoRender::Uninitialize() {
    if (m_pD3DVertexBuffer9)
    {
        m_pD3DVertexBuffer9->Release();
        m_pD3DVertexBuffer9 = nullptr;
    }
    if (m_pD3DTexture9)
    {
        m_pD3DTexture9->Release();
        m_pD3DTexture9 = nullptr;
    }
    if (m_pD3DDevice9)
    {
        m_pD3DDevice9->Release();
        m_pD3DDevice9 = nullptr;
    }
    if (m_pD3D9)
    {
        m_pD3D9->Release();
        m_pD3D9 = nullptr;
    }
}

void RTCWinVideoRender::UpdateVertex(RTCVideoRotation degree, RTCVideoScalingMode mode, int nWindowWidth, int nWindowHeight, unsigned long ulWidth, unsigned long ulHeight, RECT& srcRect)
{
    srcRect.left = 0;
    srcRect.top = 0;
    srcRect.right = ulWidth;
    srcRect.bottom = ulHeight;
    int nWndWidth = nWindowWidth;
    int nWndHeight = nWindowHeight;
    int nVideoWidth = ulWidth;
    int nVideoHeight = ulHeight;
    if ((kVideoRotation_90 == degree || kVideoRotation_270 == degree) && kVideoScaleFullFill != mode)
    {
        nVideoWidth = ulHeight;
        nVideoHeight = ulWidth;
    }
    float u = (float)ulWidth / m_nTextureWidth;
    float v = (float)ulHeight / m_nTextureHeight;
    float fVertexX[4] = { -0.5f, (float)nVideoWidth, (float)nVideoWidth, -0.5f };
    float fVertexY[4] = { -0.5f, -0.5f, (float)nVideoHeight, (float)nVideoHeight };
    bool bAdjust = false;
    int nVertexWidth = 0;
    int nVertexHeight = 0;
    if (kVideoScaleFit == mode)
    {
        if (nWndWidth * nVideoHeight != nWndHeight * nVideoWidth)
        {
            nVertexWidth = std::min(nWndWidth, m_nTextureWidth);
            nVertexHeight = std::min(nWndHeight, m_nTextureHeight);
            bAdjust = true;
        }
        if (bAdjust)
        {
            float fRatioW = (float)nWndWidth / nVideoWidth;
            float fRatioH = (float)nWndHeight / nVideoHeight;
            float fRatio = std::min(fRatioW, fRatioH);
            int nScaleWidth = static_cast<int>(nVideoWidth * fRatio);
            int nScaleHeight = static_cast<int>(nVideoHeight * fRatio);

            int widthOffset = (nWndWidth - nScaleWidth) * nVertexWidth / nWndWidth;
            if (widthOffset % 2 == 0) {
                widthOffset -= 1;
            }
            fVertexX[0] = (widthOffset * 1.0f) / 2;
            fVertexX[1] = nVertexWidth - fVertexX[0];
            fVertexX[2] = fVertexX[1];
            fVertexX[3] = fVertexX[0];

            int heightOffset = (nWndHeight - nScaleHeight) * nVertexHeight / nWndHeight;
            if (heightOffset % 2 == 0) {
                heightOffset -= 1;
            }
            fVertexY[0] = (heightOffset * 1.0f) / 2;
            fVertexY[1] = fVertexY[0];
            fVertexY[2] = nVertexHeight - fVertexY[1];
            fVertexY[3] = fVertexY[2];

            srcRect.right = nVertexWidth;
            srcRect.bottom = nVertexHeight;
        }
    }
    else if (kVideoScaleCropFill == mode)
    {
        if (nWndWidth * nVideoHeight != nWndHeight * nVideoWidth)
        {
            float fRatioW = (float)nVideoWidth / nWndWidth;
            float fRatioH = (float)nVideoHeight / nWndHeight;
            float fRatio = std::min(fRatioW, fRatioH);
            nVertexWidth = static_cast<int>(nWndWidth * fRatio);
            nVertexHeight = static_cast<int>(nWndHeight * fRatio);

            srcRect.left = (nVideoWidth - nVertexWidth) / 2;
            srcRect.top = (nVideoHeight - nVertexHeight) / 2;
            srcRect.right = srcRect.left + nVertexWidth;
            srcRect.bottom = srcRect.top + nVertexHeight;

            if (kVideoRotation_90 == degree || kVideoRotation_270 == degree) {
                fVertexX[1] = (float)nVertexHeight;
                fVertexX[2] = fVertexX[1];
                fVertexY[2] = (float)nVertexWidth;
                fVertexY[3] = fVertexY[2];

                srcRect.left = srcRect.left * nVertexHeight / nVideoWidth;
                srcRect.top = srcRect.top * nVertexWidth / nVideoHeight;
                srcRect.right = srcRect.right * nVertexHeight / nVideoWidth;
                srcRect.bottom = srcRect.bottom * nVertexWidth / nVideoHeight;
            }
        }
    }
    CUSTOMVERTEX* pVertex = nullptr;
    HRESULT hr = m_pD3DVertexBuffer9->Lock(0, 4 * sizeof(CUSTOMVERTEX), (void**)&pVertex, 0);
    if (SUCCEEDED(hr) && pVertex)
    {
        if (kVideoRotation_90 == degree)
        {
            CUSTOMVERTEX vertexes[] = {
                { fVertexX[0],	fVertexY[0],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	0.0f,	v },
                { fVertexX[1],	fVertexY[1],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	0.0f,	0.0f },
                { fVertexX[2],	fVertexY[2],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	u,		0.0f },
                { fVertexX[3],	fVertexY[3],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	u,		v },
            };
            memcpy(pVertex, vertexes, sizeof(vertexes));
        }
        else if (kVideoRotation_180 == degree)
        {
            CUSTOMVERTEX vertexes[] = {
                { fVertexX[0],	fVertexY[0],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	u,		v },
                { fVertexX[1],	fVertexY[1],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	0.0f,	v },
                { fVertexX[2],	fVertexY[2],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	0.0f,	0.0f },
                { fVertexX[3],	fVertexY[3],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	u,		0.0f },
            };
            memcpy(pVertex, vertexes, sizeof(vertexes));
        }
        else if (kVideoRotation_270 == degree)
        {
            CUSTOMVERTEX vertexes[] = {
                { fVertexX[0],	fVertexY[0],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	u,		0.0f },
                { fVertexX[1],	fVertexY[1],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	u,		v },
                { fVertexX[2],	fVertexY[2],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	0.0f,	v },
                { fVertexX[3],	fVertexY[3],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	0.0f,	0.0f },
            };
            memcpy(pVertex, vertexes, sizeof(vertexes));
        }
        else
        {
            CUSTOMVERTEX vertexes[] = {
                { fVertexX[0],	fVertexY[0],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	0.0f,	0.0f },
                { fVertexX[1],	fVertexY[1],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	u,		0.0f },
                { fVertexX[2],	fVertexY[2],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	u,		v },
                { fVertexX[3],	fVertexY[3],	0.0f,	1.0f,	D3DCOLOR_XRGB(255,255,255),	0.0f,	v },
            };
            memcpy(pVertex, vertexes, sizeof(vertexes));
        }
        m_pD3DVertexBuffer9->Unlock();
    }
}

void RTCWinVideoRender::D3dRender(unsigned char* pRGBData, int nWidth, int nHeight, int nbdp, RTCVideoRotation degree, RTCVideoScalingMode mode)
{
    RECT wndRt;
    HWND hWnd = (HWND)m_window;
    GetClientRect(hWnd, &wndRt);
    int nWindowWidth = wndRt.right - wndRt.left;
    int nWindowHeight = wndRt.bottom - wndRt.top;
    if (m_nLastWindowWidth != nWindowWidth || m_nLastWindowHeight != nWindowHeight
        || m_nLastVideoWidth != nWidth || m_nLastVideoHeight != nHeight
        || m_lastRotation != degree || m_lastScalingMode != mode) {
        UpdateVertex(degree, mode, nWindowWidth, nWindowHeight, nWidth, nHeight, m_srcRect);
        m_nLastWindowWidth = nWindowWidth;
        m_nLastWindowHeight = nWindowHeight;
        m_nLastVideoWidth = nWidth;
        m_nLastVideoHeight = nHeight;
        m_lastRotation = degree;
        m_lastScalingMode = mode;
    }
    HRESULT hr /*= m_pD3DDevice9->Clear(0, nullptr, D3DCLEAR_TARGET, D3DCOLOR_XRGB(m_nBkColorR, m_nBkColorG, m_nBkColorB), 1.0f, 0)*/;
    //copy data
    D3DLOCKED_RECT rect;
    hr = m_pD3DTexture9->LockRect(0, &rect, 0, 0);
    if (SUCCEEDED(hr)) {
        int nWidthStride = nWidth * nbdp >> 3;
        unsigned char* pSrcData = pRGBData;
        unsigned char* pDstBuff = (unsigned char*)rect.pBits;
        memcpy(pDstBuff, pSrcData, nWidthStride * nHeight);
        /*
        for (int i = 0; i < nHeight; i++) {
            memcpy(pDstBuff, pSrcData, nWidthStride);
            pSrcData += nWidthStride;
            pDstBuff += rect.Pitch;
        }*/
        m_pD3DTexture9->UnlockRect(0);
    } else {
        LOG_T(LS_ERROR) << "RTCWinVideoRender::D3dRender: LockRect failed, hr=" << hr;
        m_bInit = false;
        return;
    }
    //draw
    hr = m_pD3DDevice9->BeginScene();
    if (SUCCEEDED(hr)) {
        hr = m_pD3DDevice9->SetTexture(0, m_pD3DTexture9);
        hr = m_pD3DDevice9->SetStreamSource(0, m_pD3DVertexBuffer9, 0, sizeof(CUSTOMVERTEX));
        hr = m_pD3DDevice9->SetFVF(D3DFVF_CUSTOMVERTEX);
        hr = m_pD3DDevice9->DrawPrimitive(D3DPT_TRIANGLEFAN, 0, 2);
        m_pD3DDevice9->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);
        m_pD3DDevice9->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
        m_pD3DDevice9->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
        m_pD3DDevice9->SetTextureStageState(0, D3DTSS_TEXCOORDINDEX, 0);
        m_pD3DDevice9->SetSamplerState(0, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR);
        m_pD3DDevice9->SetSamplerState(0, D3DSAMP_MINFILTER, D3DTEXF_LINEAR);
        m_pD3DDevice9->EndScene();
        hr = m_pD3DDevice9->Present(&m_srcRect, nullptr, nullptr, nullptr);
        if (FAILED(hr)) {
            LOG_T(LS_ERROR) << "RTCWinVideoRender::D3dRender: Present failed,hr=" << hr;
            D3DPRESENT_PARAMETERS presentParam = {};
            presentParam.BackBufferCount = 1;
            presentParam.BackBufferFormat = D3DFMT_X8R8G8B8;
            presentParam.BackBufferWidth = m_nTextureWidth;
            presentParam.BackBufferHeight = m_nTextureHeight;
            presentParam.SwapEffect = D3DSWAPEFFECT_COPY;
            presentParam.hDeviceWindow = hWnd;
            presentParam.Windowed = TRUE;
            presentParam.Flags = D3DPRESENTFLAG_VIDEO;
            hr = m_pD3DDevice9->Reset(&presentParam);
            if (FAILED(hr)) {
                LOG_T(LS_ERROR) << "RTCWinVideoRender::D3dRender: Reset failed,hr=" << hr;
                m_bInit = false;
            }
        }
    } else {
        LOG_T(LS_ERROR) << "RTCWinVideoRender::D3dRender: BeginScene failed,hr=" << hr;
        m_bInit = false;
    }
}

void RTCWinVideoRender::GdiRender(unsigned char* pRGBData, int nWidth, int nHeight, int nbdp, RTCVideoScalingMode mode)
{
    BITMAPINFO bmi;
    HWND hWnd = (HWND)m_window;
    memset(&bmi, 0, sizeof(BITMAPINFO));
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;
    bmi.bmiHeader.biWidth = nWidth;
    bmi.bmiHeader.biHeight = -nHeight;
    bmi.bmiHeader.biSizeImage = bmi.bmiHeader.biWidth * bmi.bmiHeader.biHeight * (bmi.bmiHeader.biBitCount >> 3);
    RECT rect;
    GetClientRect(hWnd, &rect);
    HDC hDC = ::GetDC(hWnd);
    HDC dc_mem = ::CreateCompatibleDC(hDC);
    ::SetStretchBltMode(dc_mem, HALFTONE);
    // Set the map mode so that the ratio will be maintained for us.
    HDC all_dc[] = { hDC, dc_mem };
    for (int i = 0; i < sizeof(all_dc) / sizeof(HDC); ++i) {
        SetMapMode(all_dc[i], MM_ISOTROPIC);
        SetWindowExtEx(all_dc[i], nWidth, nHeight, NULL);
        SetViewportExtEx(all_dc[i], rect.right, rect.bottom, NULL);
    }
    HBITMAP bmp_mem = ::CreateCompatibleBitmap(hDC, rect.right, rect.bottom);
    HGDIOBJ bmp_old = ::SelectObject(dc_mem, bmp_mem);
    POINT logical_area = { rect.right, rect.bottom };
    DPtoLP(hDC, &logical_area, 1);
    HBRUSH brush = ::CreateSolidBrush(RGB(m_nBkColorR, m_nBkColorG, m_nBkColorB));
    RECT logical_rect = { 0, 0, logical_area.x, logical_area.y };
    ::FillRect(dc_mem, &logical_rect, brush);
    ::DeleteObject(brush);
    int x = 0;
    int y = 0;
    int destWidth = nWidth;
    int destHeight = nHeight;
    if (kVideoScaleFit == mode)
    {
        x = (logical_area.x >> 1) - (nWidth >> 1);
        y = (logical_area.y >> 1) - (nHeight >> 1);
        destWidth = nWidth;
        destHeight = nHeight;
    }
    else if (kVideoScaleFullFill == mode)
    {
        x = 0;
        y = 0;
        destWidth = logical_area.x;
        destHeight = logical_area.y;
    }
    else if (kVideoScaleCropFill == mode)
    {
        float fRatioW = (float)nWidth / logical_area.x;
        float fRatioH = (float)nHeight / logical_area.y;
        if (fRatioW < fRatioH)
        {
            x = 0;
            destWidth = logical_area.x;
            destHeight = logical_area.y * logical_area.x / nWidth;
            y = (nHeight - destHeight) / 2;
        }
        else
        {
            y = 0;
            destHeight = logical_area.y;
            destWidth = logical_area.x * logical_area.y / nHeight;
            x = (nWidth - destWidth) / 2;
        }
    }
    StretchDIBits(dc_mem, x, y, destWidth, destHeight,
        0, 0, nWidth, nHeight, pRGBData, &bmi, DIB_RGB_COLORS, SRCCOPY);
    BitBlt(hDC, 0, 0, logical_area.x, logical_area.y,
        dc_mem, 0, 0, SRCCOPY);
    // Cleanup.
    ::SelectObject(dc_mem, bmp_old);
    ::DeleteObject(bmp_mem);
    ::DeleteDC(dc_mem);
    ::ReleaseDC(hWnd, hDC);
}

DWORD WINAPI RTCWinVideoRender::RenderThread(LPVOID lpParam)
{
    RTCWinVideoRender* pRender = (RTCWinVideoRender*)lpParam;
    pRender->OnRenderThread();
    return 0;
}

void RTCWinVideoRender::OnRenderThread() {
    HWND hWnd = (HWND)m_window;
    std::unique_ptr<webrtc::VideoFrame> frame;
    unsigned char* pVideoData = nullptr;
    int nRGBBufferSize = MAX_VIDEO_WIDTH * MAX_VIDEO_HEIGHT << 2;
    std::unique_ptr<unsigned char> rgbData(new unsigned char[nRGBBufferSize]);
    std::unique_ptr<unsigned char> rgbRotationData(new unsigned char[nRGBBufferSize]);
    bool bD3DInitialed = false;
    while (m_bRending) {
        /*DWORD dwRes = */WaitForSingleObject(m_hEvent, 1500);
        {
            std::lock_guard<std::mutex> lock_guard(m_Lock);
            frame = std::move(m_VideoFrame);
        }
        if (nullptr == frame) {
            SafeRendBkgnd();
            continue;
        }
        
        int nWidth = frame->width();
        int nHeight = frame->height();
        if (nWidth <= 0 || nHeight <= 0 )
            continue;

        if (!m_bInit || nWidth != m_nTextureWidth || nHeight != m_nTextureHeight) {
            Uninitialize();
            bD3DInitialed = Initialize(nWidth, nHeight);
            m_bInit = true;
        }

        rtc::scoped_refptr<webrtc::VideoFrameBuffer> frameBuffer;
        if (frame->video_frame_buffer()->native_handle() == nullptr) {
            frameBuffer = frame->video_frame_buffer();
        } else {
            frameBuffer = frame->video_frame_buffer()->NativeToI420Buffer();
        }

        if (!m_yuv) {
            pVideoData = (unsigned char*)frameBuffer->DataY();
        } else {
            const uint8_t * y = frameBuffer->DataY();
            const uint8_t * u = frameBuffer->DataU();
            const uint8_t * v = frameBuffer->DataV();
            
            libyuv::I420ToARGB(y, frameBuffer->StrideY(), u, frameBuffer->StrideU(), v, frameBuffer->StrideV(), rgbData.get(), nWidth << 2, nWidth, nHeight);
            pVideoData = rgbData.get();
			if (!bD3DInitialed && kVideoRotation_0 != (RTCVideoRotation)frame->rotation()) {
				bool bSwap = false;
				int nDstStride = nWidth << 2;
				libyuv::RotationMode mode = libyuv::kRotate0;
				if (kVideoRotation_90 == (RTCVideoRotation)frame->rotation()) {
					mode = libyuv::kRotate90;
					nDstStride = nHeight << 2;
					bSwap = true;
				}
				else if (kVideoRotation_180 == (RTCVideoRotation)frame->rotation()) {
					mode = libyuv::kRotate180;
					nDstStride = nWidth << 2;
				}
				else {
					mode = libyuv::kRotate270;
					nDstStride = nHeight << 2;
					bSwap = true;
				}
				ARGBRotate(rgbData.get(), nWidth << 2, rgbRotationData.get(), nDstStride, nWidth, nHeight, mode);
				pVideoData = rgbRotationData.get();
				if (bSwap) {
					std::swap(nWidth, nHeight);
				}
				if (m_mirror) {
					libyuv::ARGBMirror(pVideoData, nDstStride, rgbData.get(), nDstStride, nWidth, nHeight);
					pVideoData = rgbData.get();
				}
			}
			else if (m_mirror) {
				int nDstStride = nWidth << 2;
				libyuv::ARGBMirror(pVideoData, nDstStride, rgbRotationData.get(), nDstStride, nWidth, nHeight);
				pVideoData = rgbRotationData.get();
			}
        }
        
        if (bD3DInitialed)
            D3dRender(pVideoData, nWidth, nHeight, 32, (RTCVideoRotation)frame->rotation(), m_scalingMode);
        else
            GdiRender(pVideoData, nWidth, nHeight, 32, m_scalingMode);
    }
    Uninitialize();
}

void RTCWinVideoRender::SafeRendBkgnd()
{
    RECT rect;
    HWND hWnd = (HWND)m_window;
   
    GetClientRect(hWnd, &rect);
   /* int nWindowWidth = rect.right - rect.left;
    int nWindowHeight = rect.bottom - rect.top;
    if (nWindowWidth <= 0 || nWindowHeight <= 0)
        return;
    */
    HDC hDC = GetDC(hWnd);
    POINT logical_area = { rect.right, rect.bottom };
    DPtoLP(hDC, &logical_area, 1);
    HBRUSH brush = ::CreateSolidBrush(RGB(m_nBkColorR, m_nBkColorG, m_nBkColorB));
    RECT logical_rect = { 0, 0, logical_area.x, logical_area.y };
    ::FillRect(hDC, &logical_rect, brush);
    ::DeleteObject(brush);
    ::DeleteDC(hDC);
}
