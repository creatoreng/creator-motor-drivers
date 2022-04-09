# communicate to a Herkulex motor
add_executable( herkulex-example herkulex-example.cpp )
target_link_libraries( herkulex-example herkulex )

# scan all Herkulex amplifiers
add_executable( herkulex-scan herkulex-scan.cpp )
target_link_libraries( herkulex-scan herkulex )
