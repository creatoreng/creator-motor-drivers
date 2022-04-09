/**
 * @author Creator, Inc.
 * @copyright Copyright (c) 2022, Creator, Inc. MIT license.
 *
 * See header file for documentation.
 **/

#include <drivers/copley/CopleyBus.hpp> // For kSerialNodeId
#include <drivers/copley/CopleyCommandBuilder.hpp>

namespace momentum
{

CopleyCommandBuilder::CopleyCommandBuilder(const uint8_t node_id, const uint8_t axis)
  : _op_code(OpCode::kNop), _node_id(node_id), _axis(axis)
{
}

bool CopleyCommandBuilder::expects_response(void) const
{
	// only reset commands are not acknowledged, with the exception of reset of CAN multi-drop nodes.
	// In the case of resetting a CAN multi-drop notes other than the master, an error response
	// is expected. See Copley ASCII Programmer's Guide.

	return _op_code != OpCode::kReset || (_op_code == OpCode::kReset && _node_id != CopleyBus::kSerialNodeId);
}

} // namespace momentum
