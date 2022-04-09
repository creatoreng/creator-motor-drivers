# communicate to a Copley amplifier (no configuration)
add_executable( copley-example-amplifier copley-example-amplifier.cpp )
target_link_libraries( copley-example-amplifier copley )

# drive a supported motor + primary encoder
add_executable( copley-example-supported copley-example-supported.cpp )
target_link_libraries( copley-example-supported copley )

# drive a supported motor + primary encoder
add_executable( copley-example-custom copley-example-custom.cpp )
target_link_libraries( copley-example-custom copley )

# scan all Copley amplifiers
add_executable( copley-scan copley-scan.cpp )
target_link_libraries( copley-scan copley )
