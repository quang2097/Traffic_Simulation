#include "traffic/Simulation.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>
#ifdef _WIN32
#include <windows.h>
#include <wincodec.h>
#include <wrl/client.h>
#else
#include <png.h>
#endif

namespace traffic {
Map Map::fromPng(const std::string& path,double metersPerPixel,MapPalette palette) {
    unsigned width=0,height=0;
    std::vector<unsigned char> pixels;
#ifdef _WIN32
    struct ComScope {
        HRESULT status=CoInitializeEx(nullptr,COINIT_MULTITHREADED);
        ~ComScope() { if (SUCCEEDED(status)) CoUninitialize(); }
    } scope;
    auto check=[](HRESULT hr) { if (FAILED(hr)) throw std::runtime_error("Cannot decode PNG map"); };
    if (FAILED(scope.status) && scope.status != RPC_E_CHANGED_MODE) check(scope.status);
    using Microsoft::WRL::ComPtr;
    ComPtr<IWICImagingFactory> factory;
    check(CoCreateInstance(CLSID_WICImagingFactory,nullptr,CLSCTX_INPROC_SERVER,IID_PPV_ARGS(factory.GetAddressOf())));
    int length=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path.c_str(),-1,nullptr,0);
    if (!length) throw std::runtime_error("Invalid UTF-8 map path");
    std::wstring filename(length,L'\0');
    MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path.c_str(),-1,filename.data(),length);
    ComPtr<IWICBitmapDecoder> decoder;
    check(factory->CreateDecoderFromFilename(filename.c_str(),nullptr,GENERIC_READ,WICDecodeMetadataCacheOnLoad,decoder.GetAddressOf()));
    GUID format;
    check(decoder->GetContainerFormat(&format));
    if (!IsEqualGUID(format,GUID_ContainerFormatPng)) throw std::runtime_error("Map must be PNG");
    ComPtr<IWICBitmapFrameDecode> frame;
    check(decoder->GetFrame(0,frame.GetAddressOf()));
    check(frame->GetSize(&width,&height));
    if (!width || !height || static_cast<std::uint64_t>(width)*height > 16000000)
        throw std::runtime_error("Map exceeds 16 million cells");
    ComPtr<IWICFormatConverter> converter;
    check(factory->CreateFormatConverter(converter.GetAddressOf()));
    check(converter->Initialize(frame.Get(),GUID_WICPixelFormat32bppRGBA,WICBitmapDitherTypeNone,nullptr,0,WICBitmapPaletteTypeCustom));
    pixels.resize(static_cast<std::size_t>(width)*height*4);
    check(converter->CopyPixels(nullptr,width*4,static_cast<UINT>(pixels.size()),pixels.data()));
#else
    png_image image{}; image.version=PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_file(&image,path.c_str())) throw std::runtime_error("Cannot open PNG map");
    struct Cleanup { png_image* image; ~Cleanup() { png_image_free(image); } } cleanup{&image};
    width=image.width; height=image.height;
    if (!width || !height || static_cast<std::uint64_t>(width)*height > 16000000)
        throw std::runtime_error("Map exceeds 16 million cells");
    image.format=PNG_FORMAT_RGBA; pixels.resize(PNG_IMAGE_SIZE(image));
    if (!png_image_finish_read(&image,nullptr,pixels.data(),0,nullptr)) throw std::runtime_error("Cannot decode PNG map");
#endif
    std::vector<Tile> tiles;
    for (std::size_t i=0; i<pixels.size(); i+=4) {
        int r=pixels[i],g=pixels[i+1],b=pixels[i+2],a=pixels[i+3];
        if(a<128) tiles.push_back(Tile::Wall);
        else if(palette==MapPalette::Legacy) {
            if(g>=128 && g>r*1.5 && g>b*1.5) tiles.push_back(Tile::Spawn);
            else if(r>=128 && r>g*1.5 && r>b*1.5) tiles.push_back(Tile::Destination);
            else tiles.push_back((299*r+587*g+114*b)>=128000?Tile::Road:Tile::Wall);
        } else {
            // Includes magenta, hot pink and pale pink; classify before red.
            if(r>=128 && b>=128 && b>g+8 && r>g+20) tiles.push_back(Tile::Divider);
            else if(r>=128 && g>=128 && b<std::min(r,g)*.65) tiles.push_back(Tile::Spawn);
            else if(g>=100 && g>r*1.4 && g>b*1.4) tiles.push_back(Tile::Destination);
            else if(b>=100 && b>r*1.4 && b>g*1.4) tiles.push_back(Tile::Signal);
            else if(r>=100 && r>g*1.4 && r>b*1.4) tiles.push_back(Tile::Road);
            else tiles.push_back(Tile::Wall);
        }
    }
    Map result(static_cast<int>(width),static_cast<int>(height),std::move(tiles),metersPerPixel);
    result.rebuildFeatures(); return result;
}
void Map::savePng(const std::string& path) const {
    std::vector<unsigned char> pixels; pixels.reserve(tiles.size()*3);
    for(Tile tile:tiles) {
        unsigned char r=0,g=0,b=0;
        switch(tile) {
        case Tile::Road:r=255;break;
        case Tile::Spawn:r=g=255;break;
        case Tile::Destination:g=255;break;
        case Tile::Signal:b=255;break;
        case Tile::Divider:r=b=255;break;
        default:break;
        }
#ifdef _WIN32
        pixels.insert(pixels.end(),{b,g,r});
#else
        pixels.insert(pixels.end(),{r,g,b});
#endif
    }
#ifdef _WIN32
    struct ComScope {
        HRESULT status=CoInitializeEx(nullptr,COINIT_MULTITHREADED);
        ~ComScope() { if(SUCCEEDED(status)) CoUninitialize(); }
    } scope;
    auto check=[](HRESULT hr) { if(FAILED(hr)) throw std::runtime_error("Cannot save PNG map"); };
    if(FAILED(scope.status) && scope.status!=RPC_E_CHANGED_MODE) check(scope.status);
    using Microsoft::WRL::ComPtr;
    ComPtr<IWICImagingFactory> factory;
    check(CoCreateInstance(CLSID_WICImagingFactory,nullptr,CLSCTX_INPROC_SERVER,IID_PPV_ARGS(factory.GetAddressOf())));
    int length=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path.c_str(),-1,nullptr,0);
    if(!length) throw std::runtime_error("Invalid UTF-8 map path");
    std::wstring filename(length,L'\0');
    MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path.c_str(),-1,filename.data(),length);
    ComPtr<IWICStream> stream; check(factory->CreateStream(stream.GetAddressOf()));
    check(stream->InitializeFromFilename(filename.c_str(),GENERIC_WRITE));
    ComPtr<IWICBitmapEncoder> encoder;
    check(factory->CreateEncoder(GUID_ContainerFormatPng,nullptr,encoder.GetAddressOf()));
    check(encoder->Initialize(stream.Get(),WICBitmapEncoderNoCache));
    ComPtr<IWICBitmapFrameEncode> frame; ComPtr<IPropertyBag2> properties;
    check(encoder->CreateNewFrame(frame.GetAddressOf(),properties.GetAddressOf()));
    check(frame->Initialize(properties.Get()));
    check(frame->SetSize(static_cast<UINT>(width),static_cast<UINT>(height)));
    WICPixelFormatGUID format=GUID_WICPixelFormat24bppBGR; check(frame->SetPixelFormat(&format));
    if(!IsEqualGUID(format,GUID_WICPixelFormat24bppBGR)) throw std::runtime_error("Unsupported PNG encoder format");
    check(frame->WritePixels(static_cast<UINT>(height),static_cast<UINT>(width*3),static_cast<UINT>(pixels.size()),pixels.data()));
    check(frame->Commit()); check(encoder->Commit());
#else
    png_image output{}; output.version=PNG_IMAGE_VERSION;
    output.width=width; output.height=height; output.format=PNG_FORMAT_RGB;
    if(!png_image_write_to_file(&output,path.c_str(),0,pixels.data(),0,nullptr)) {
        std::string message=output.message; png_image_free(&output); throw std::runtime_error("Cannot save PNG map: "+message);
    }
    png_image_free(&output);
#endif
}
}
