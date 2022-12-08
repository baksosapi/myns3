cmake_minimum_required(VERSION 3.20)
project(simulasi_ns3)

set(CMAKE_CXX_STANDARD 17)

file (STRINGS "ns3lib-loc.txt" BUILD_LOC)

set(NS3BUILDDIR ${BUILD_LOC}/ns-allinone-3.36.1/ns-3.36.1/build)

link_directories(
        ${NS3BUILDDIR}
        ${NS3BUILDDIR}/lib
)

add_executable(${PROJECT_NAME}
        aodv-energy-optim.cpp
#        aodv-energy.cpp
#        aodv-basic.cpp
#        raw/manet-routing-compare.cc
#        routinglib/aodv/aodv-routing-protocol.cc
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${NS3BUILDDIR}
        ${NS3BUILDDIR}/include
        )

target_link_libraries(${PROJECT_NAME}
        ns3.36.1-antenna-default
        ns3.36.1-antenna-test-default
        ns3.36.1-aodv-default
        ns3.36.1-aodv-test-default
        ns3.36.1-applications-default
        ns3.36.1-applications-test-default
        ns3.36.1-bridge-default
        ns3.36.1-buildings-default
        ns3.36.1-buildings-test-default
        ns3.36.1-config-store-default
        ns3.36.1-core-default
        ns3.36.1-core-test-default
        ns3.36.1-csma-default
        ns3.36.1-csma-layout-default
        ns3.36.1-dsdv-default
        ns3.36.1-dsdv-test-default
        ns3.36.1-dsr-default
        ns3.36.1-dsr-test-default
        ns3.36.1-energy-default
        ns3.36.1-energy-test-default
        ns3.36.1-flow-monitor-default
        ns3.36.1-internet-apps-default
        ns3.36.1-internet-apps-test-default
        ns3.36.1-internet-default
        ns3.36.1-internet-test-default
        ns3.36.1-lr-wpan-default
        ns3.36.1-lr-wpan-test-default
        ns3.36.1-lte-default
        ns3.36.1-lte-test-default
        ns3.36.1-mesh-default
        ns3.36.1-mesh-test-default
        ns3.36.1-mobility-default
        ns3.36.1-mobility-test-default
        ns3.36.1-netanim-default
        ns3.36.1-netanim-test-default
        ns3.36.1-network-default
        ns3.36.1-network-test-default
        ns3.36.1-nix-vector-routing-default
        ns3.36.1-nix-vector-routing-test-default
        ns3.36.1-olsr-default
        ns3.36.1-olsr-test-default
        ns3.36.1-point-to-point-default
        ns3.36.1-point-to-point-layout-default
        ns3.36.1-point-to-point-test-default
        ns3.36.1-propagation-default
        ns3.36.1-propagation-test-default
        ns3.36.1-sixlowpan-default
        ns3.36.1-sixlowpan-test-default
        ns3.36.1-spectrum-default
        ns3.36.1-spectrum-test-default
        ns3.36.1-stats-default
        ns3.36.1-stats-test-default
        ns3.36.1-topology-read-default
        ns3.36.1-topology-read-test-default
        ns3.36.1-traffic-control-default
        ns3.36.1-traffic-control-test-default
        ns3.36.1-uan-default
        ns3.36.1-uan-test-default
        ns3.36.1-virtual-net-device-default
        ns3.36.1-wave-default
        ns3.36.1-wave-test-default
        ns3.36.1-wifi-default
        ns3.36.1-wifi-test-default
        ns3.36.1-wimax-default
        ns3.36.1-wimax-test-default
        )

add_definitions(-DNS3_LOG_ENABLE)
