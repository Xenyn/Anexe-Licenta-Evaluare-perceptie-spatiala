/*
 ******************************************************************************
 *  CarMaker - Version 15.0.1
 *  Virtual Test Driving
 ******************************************************************************
 */

#ifndef NOMINMAX
#define NOMINMAX
#endif

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include "Client.h"

// std
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <thread>
#include <vector>
#include <limits>

bool volatile stopMainThread = false;

void
signalHandler([[maybe_unused]] int const signal)
{
    stopMainThread = true;
}

void
printRadarDetections(Deserializer::RadarData& radarData)
{
    auto header = radarData.header;
    auto data   = radarData.data;

    if (Deserializer::RadarOutputCartesian == header.OutputType
        || Deserializer::RadarOutputSpherical == header.OutputType) {
        auto detections = reinterpret_cast<Deserializer::RadarDataCoordinates*>(data);

        for (int i = 0; i < header.nDetPointC; ++i) {
            std::cout << "    Detection[" << i << "] - Received power: "
                      << detections[i].PowerdB << std::endl;
        }
    } else if (Deserializer::RadarOutputVrx == header.OutputType) {
        auto detections = reinterpret_cast<Deserializer::RadarDataVrx*>(data);

        for (int i = 0; i < header.nDetVRx; ++i) {
            std::cout << "    Detection[" << i << "] - Received range: "
                      << detections[i].range << std::endl;
        }
    }
}

static float
deg2rad(float deg)
{
    return deg * 3.14159265358979323846f / 180.0f;
}

/*
printLidarPointCloud(Deserializer::LidarData& lidarData)
{
    const auto& header = lidarData.Header;
    const auto* data   = lidarData.SP;

    constexpr int NH = 581;
    constexpr int NV = 30;

    constexpr float HMIN = -180.0f;
    constexpr float HMAX =  180.0f;
    constexpr float VMIN =  -15.0f;
    constexpr float VMAX =   15.0f;

    constexpr bool BEAM_ID_IS_ONE_BASED = false;
    constexpr bool LENGTH_OF_IS_ROUND_TRIP = false;

    std::cout << "    Sensor " << header.SensorID
              << " - Number of detections: "
              << header.nScanPoints << std::endl;

    int nPrint = header.nScanPoints;
    if (nPrint > 10) {
        nPrint = 10;
    }

    for (int j = 0; j < nPrint; ++j) {
        const auto& sp = data[j];

        int beamID = sp.BeamID;

        if (BEAM_ID_IS_ONE_BASED) {
            beamID -= 1;
        }

        if (beamID < 0 || beamID >= NH * NV) {
            std::cout << "    SP[" << j << "] invalid BeamID="
                      << sp.BeamID << std::endl;
            continue;
        }

        int hIdx = beamID % NV;
        int vIdx = beamID / NV;

        float azDeg = HMIN + (HMAX - HMIN) * static_cast<float>(hIdx) / static_cast<float>(NH - 1);
        float elDeg = VMIN + (VMAX - VMIN) * static_cast<float>(vIdx) / static_cast<float>(NV - 1);

        float az = deg2rad(azDeg);
        float el = deg2rad(elDeg);

        // LengthOF is the optical flight path length. For the LiDAR point
        // position we need the one-way distance from the ray origin to the
        // reflection point.
        constexpr bool LENGTH_OF_IS_ROUND_TRIP = true;
        float r = LENGTH_OF_IS_ROUND_TRIP ? 0.5f * sp.LengthOF : sp.LengthOF;

        if (LENGTH_OF_IS_ROUND_TRIP) {
            r *= 0.5f;
        }

        float x = sp.Origin[0] + r * std::cos(el) * std::cos(az);
        float y = sp.Origin[1] + r * std::cos(el) * std::sin(az);
        float z = sp.Origin[2] + r * std::sin(el);

        std::cout << "    SP[" << j << "]"
                  << " BeamID=" << sp.BeamID
                  << " EchoID=" << sp.EchoID
                  << " LengthOF=" << sp.LengthOF
                  << " Intensity=" << sp.Intensity
                  << " Origin=("
                  << sp.Origin[0] << ", "
                  << sp.Origin[1] << ", "
                  << sp.Origin[2] << ")"
                  << " XYZ=(" << x << ", " << y << ", " << z << ")"
                  << std::endl;
    }
}
*/
static SOCKET gSimulinkSocket = INVALID_SOCKET;

static bool SendAll(SOCKET s, const char* data, int bytes)
{
    int sentTotal = 0;

    while (sentTotal < bytes) {
        int sent = send(s, data + sentTotal, bytes - sentTotal, 0);
        if (sent <= 0) {
            return false;
        }
        sentTotal += sent;
    }

    return true;
}

static void InitSimulinkTcpServer(unsigned short port)
{
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    SOCKET listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1
    addr.sin_port = htons(port);

    bind(listenSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    listen(listenSocket, 1);

    std::cout << "Waiting for Simulink on TCP port " << port << "..." << std::endl;

    gSimulinkSocket = accept(listenSocket, nullptr, nullptr);

    std::cout << "Simulink connected." << std::endl;

    closesocket(listenSocket);
}
static void SendLidarToSimulink(Deserializer::LidarData& lidarData, float timestamp)
{
    constexpr int MAX_POINTS = 300300;
    constexpr int HEADER_FLOATS = 9;
    constexpr int VALUES_PER_POINT = 4;
    constexpr float PACKET_MAGIC = 1279345.0f;
    constexpr float PACKET_VERSION = 1.0f;
    constexpr int NH = 1001;
    constexpr int NV = 300;

    constexpr float HMIN = -60.0f;
    constexpr float HMAX =  60.0f;
    constexpr float VMIN =  -12.5f;
    constexpr float VMAX =   12.5;

    std::vector<float> packet(
        HEADER_FLOATS + MAX_POINTS * VALUES_PER_POINT,
        std::numeric_limits<float>::quiet_NaN());

    const int actualPointCount = lidarData.Header.nScanPoints;
    const int n = (actualPointCount < MAX_POINTS) ? actualPointCount : MAX_POINTS;
    static std::uint32_t frameNumber = 0;
    ++frameNumber;

    packet[0] = PACKET_MAGIC;
    packet[1] = PACKET_VERSION;
    packet[2] = timestamp;
    packet[3] = static_cast<float>(frameNumber);
    packet[4] = static_cast<float>(lidarData.Header.SensorID);
    packet[5] = static_cast<float>(actualPointCount);
    packet[6] = static_cast<float>(n);
    packet[7] = (actualPointCount > MAX_POINTS) ? 1.0f : 0.0f;
    packet[8] = static_cast<float>(VALUES_PER_POINT);
    int minBeam = NH * NV;
    int maxBeam = -1;
    float minR = 1e9f;
    float maxR = -1.0f;

    for (int k = 0; k < n; ++k) {
        const auto& sp = lidarData.SP[k];

        if (sp.BeamID < minBeam) minBeam = sp.BeamID;
        if (sp.BeamID > maxBeam) maxBeam = sp.BeamID;

        if (sp.LengthOF < minR) minR = sp.LengthOF;
        if (sp.LengthOF > maxR) maxR = sp.LengthOF;
    }

    std::cout << "BeamID range: " << minBeam << "..." << maxBeam
          << " | LengthOF range: " << minR << "..." << maxR
          << " | n=" << n << std::endl;
    for (int i = 0; i < n; ++i) {
        const auto& sp = lidarData.SP[i];

        int beamID = sp.BeamID;

        if (beamID < 0 || beamID >= NH * NV) {
            continue;
        }

        constexpr int BEAM_MAPPING_MODE = 0; // test 0, then 1

        int hIdx = 0;
        int vIdx = 0;

        if constexpr (BEAM_MAPPING_MODE == 0) {
            hIdx = beamID % NH;
            vIdx = beamID / NH;
        } else {
            vIdx = beamID % NV;
            hIdx = beamID / NV;
        }

        float azDeg = HMIN + (HMAX - HMIN)
            * (static_cast<float>(hIdx) + 0.5f) / static_cast<float>(NH);
        constexpr bool FLIP_ELEVATION = false;

        float elDeg = FLIP_ELEVATION
            ? VMAX - (VMAX - VMIN) * (static_cast<float>(vIdx) + 0.5f) / static_cast<float>(NV)
            : VMIN + (VMAX - VMIN) * (static_cast<float>(vIdx) + 0.5f) / static_cast<float>(NV);

        float az = azDeg * 3.14159265358979323846f / 180.0f;
        float el = elDeg * 3.14159265358979323846f / 180.0f;

        // LengthOF is the optical flight path length. For the point
        // coordinates we need the one-way distance from the ray origin to
        // the reflection point.
        constexpr bool LENGTH_OF_IS_ROUND_TRIP = true;
        float r = LENGTH_OF_IS_ROUND_TRIP ? 0.5f * sp.LengthOF : sp.LengthOF;

        // Point in LiDAR sensor frame
        float xs = sp.Origin[0] + r * std::cos(el) * std::cos(az);
        float ys = sp.Origin[1] + r * std::cos(el) * std::sin(az);
        float zs = sp.Origin[2] + r * std::sin(el);

        // LiDAR mounting from CarMaker screenshot:
        // Position x/y/z = 2.7 / 0 / 1.6 m
        // Orientation    = 0 / 0 / 0 deg
        float xv = xs + 2.7f;
        float yv = ys + 0.0f;
        float zv = zs + 1.6f;

        const int offset = HEADER_FLOATS + VALUES_PER_POINT * i;
        packet[offset + 0] = xv;
        packet[offset + 1] = yv;
        packet[offset + 2] = zv;
        packet[offset + 3] = sp.Intensity;
    }
    if (gSimulinkSocket != INVALID_SOCKET) {
        SendAll(gSimulinkSocket,
                reinterpret_cast<const char*>(packet.data()),
                static_cast<int>(packet.size() * sizeof(float)));
    }
}
void
userCallbackFunction(Deserializer::Parser& deserializer, std::string_view domain, std::string_view port)
{
    std::cout << "[DEBUG] Callback reached. SensorType="
              << static_cast<int>(deserializer.getSensorType())
              << " NumSensors="
              << deserializer.getNumSensors()
              << " Thread("
              << domain << ":" << port << ")"
              << std::endl;

    switch (deserializer.getSensorType()) {
        case Deserializer::SensorType::RadarRsi: {
            for (unsigned int i = 0; i < deserializer.getNumSensors(); ++i) {
                std::cout << std::fixed << std::setprecision(3);
                std::cout << "  Thread(" << domain << ":" << port << ") "
                          << " - RadarRSI[" << i << "]" << std::endl;

                printRadarDetections(deserializer.getRadarDetections(i));
            }
            break;
        }

        case Deserializer::SensorType::LidarRsi: {
             for (unsigned int i = 0; i < deserializer.getNumSensors(); ++i) {
                std::cout << std::fixed << std::setprecision(3);
                std::cout << "  Thread(" << domain << ":" << port << ") "
                          << " - LidarRSI[" << i << "]" << std::endl;

                auto& lidarData = deserializer.getLidarPointCloud(i);

                std::cout << "    Sensor " << lidarData.Header.SensorID
                          << " - Number of detections: "
                          << lidarData.Header.nScanPoints
                          << std::endl;
                SendLidarToSimulink(lidarData, deserializer.getTimestamp());         
                }

            break;
        }

        default:
            std::cout << "[DEBUG] Unhandled sensor type." << std::endl;
            break;
    }
}

int
main(int const argc, char* argv[])
{
    std::signal(SIGINT, signalHandler);

    InitSimulinkTcpServer(55000);

    Tcp::Client tcpClient(argc, argv, userCallbackFunction);

    tcpClient.start();

    while (!stopMainThread) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    tcpClient.stop();

    return 0;
}
