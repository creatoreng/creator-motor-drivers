# primary library
add_library(
    herkulex
    src/Herkulex.cpp
    src/HerkulexRegister.cpp
    src/HerkulexStatusRegister.cpp
    src/HerkulexModel.cpp
    src/HerkulexPacket.cpp
    src/HerkulexBus.cpp
    src/HerkulexStatus.cpp
    src/HerkulexScan.cpp )
target_link_libraries(
    herkulex
    libserial_static
    status
    ${Boost_TIMER_LIBRARY}
    ${Boost_CHRONO_LIBRARY} )

add_subdirectory(examples)
