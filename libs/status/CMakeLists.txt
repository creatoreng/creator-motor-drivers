# primary library
add_library(status src/GenericStatus.cpp)
target_link_libraries(
    status
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_CHRONO_LIBRARY}
    ${Boost_TIMER_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY})
