/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
**/

#include <drivers/herkulex/HerkulexScan.hpp>

#include <iostream>

using namespace momentum;

int main(int argc, char *argv[])
{
	HerkulexScan scan;
	if(scan.connect_uart("/dev/tty0")){
		std::cout << "Herkulex IDs found:" << std::endl;
		std::cout << scan.scan_all() << std::endl;
	}

	return 0;
}
