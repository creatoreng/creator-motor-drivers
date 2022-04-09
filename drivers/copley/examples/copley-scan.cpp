/**
* @author Creator, Inc.
* @copyright Copyright (c) 2022, Creator, Inc. MIT license.
**/

#include <drivers/copley/CopleyScan.hpp>

#include <iostream>

using namespace momentum;

int main(int argc, char *argv[])
{
	CopleyScan scan;
	if(scan.connect_uart("/dev/tty0")){
		std::cout << "Copley IDs found:" << std::endl;
		std::cout << scan.scan_all() << std::endl;
	}

	return 0;
}
