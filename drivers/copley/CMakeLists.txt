# primary library
add_library(
    copley
    src/Copley.cpp
    src/CopleyConfiguration.cpp
    src/CopleyEventRegister.cpp
    src/CopleyParameter.cpp
    src/CopleyCommandBuilder.cpp
    src/CopleyCommandBuilderAscii.cpp
    src/CopleyCommandBuilderBinary.cpp
    src/CopleyResponse.cpp
    src/CopleyResponseAscii.cpp
    src/CopleyResponseBinary.cpp
    src/CopleyModel.cpp
    src/CopleyBus.cpp
    src/CopleyStatus.cpp
    src/CopleyScan.cpp)
target_link_libraries(
    copley
    status
    libserial_static
    ${Boost_TIMER_LIBRARY}
    ${Boost_CHRONO_LIBRARY} )

add_subdirectory(examples)
