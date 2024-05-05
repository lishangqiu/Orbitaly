// Orbitaly serial protocol — the single source of truth for the firmware half.
//
// Every constant here also exists in orbitaly/motion/serial_protocol.py, and
// tests/test_serial_protocol.py parses this file and asserts the two agree.
// There is no code generation: drift is caught, not prevented, because a
// generator would be one more thing to get wrong and this file has to stay
// readable to somebody holding a soldering iron.
//
// Keep this file free of Arduino headers. framing.cpp and queue.cpp include it
// and are compiled on the host by the test suite.

#ifndef ORBITALY_PROTOCOL_H
#define ORBITALY_PROTOCOL_H

#include <stdint.h>

#define ORB_SOF 0xA5
#define ORB_PROTOCOL_VERSION 1
#define ORB_MAX_PAYLOAD 24
#define ORB_FRAME_OVERHEAD 5

// Sequence numbers wrap; 0xFF is reserved so ABORTED can report "nothing was
// in flight" without a second message type.
#define ORB_SEQ_NONE 0xFF
#define ORB_SEQ_MAX 0xFE

#define ORB_PIN_NONE 0xFF

#define ORB_AXIS_AZ 0
#define ORB_AXIS_EL 1
#define ORB_AXIS_ALL 0xFF
#define ORB_AXIS_COUNT 2

// -- message types ---------------------------------------------------------
// High bit set == sent by the firmware. On a loopback jumper, traffic coming
// back the way it went out is then recognisably wrong rather than plausible.

#define ORB_MSG_HELLO 0x01
#define ORB_MSG_SEG 0x02
#define ORB_MSG_ABORT 0x03
#define ORB_MSG_ENABLE 0x04
#define ORB_MSG_CLEAR_FAULT 0x05
#define ORB_MSG_PING 0x06
#define ORB_MSG_STATUS 0x07

#define ORB_MSG_IDENT 0x81
#define ORB_MSG_ACK 0x82
#define ORB_MSG_NACK 0x83
#define ORB_MSG_DONE 0x84
#define ORB_MSG_ABORTED 0x85
#define ORB_MSG_EVENT 0x86
#define ORB_MSG_STATE 0x87
#define ORB_MSG_PONG 0x88

// -- NACK reasons ----------------------------------------------------------

#define ORB_NACK_BAD_CRC 1
#define ORB_NACK_BAD_LENGTH 2
#define ORB_NACK_BAD_TYPE 3
#define ORB_NACK_BAD_AXIS 4
#define ORB_NACK_QUEUE_FULL 5
#define ORB_NACK_INHIBITED 6
#define ORB_NACK_BAD_VERSION 7
#define ORB_NACK_NOT_READY 8

// -- abort causes ----------------------------------------------------------

#define ORB_ABORT_HOST 1
#define ORB_ABORT_ENDSTOP 2
#define ORB_ABORT_ESTOP 3
#define ORB_ABORT_WATCHDOG 4

// -- event kinds -----------------------------------------------------------

#define ORB_EVENT_ENDSTOP_AZ 1
#define ORB_EVENT_ENDSTOP_EL 2
#define ORB_EVENT_ESTOP 3

// -- capability bits (IDENT) -----------------------------------------------

#define ORB_CAP_EXACT_ABORT 0x0001
#define ORB_CAP_ENDSTOP_NC 0x0002
#define ORB_CAP_ENABLE_ACTIVE_LOW 0x0004
#define ORB_CAP_ESTOP_FITTED 0x0008
#define ORB_CAP_AZ_ENDSTOP 0x0010
#define ORB_CAP_EL_ENDSTOP 0x0020

// -- per-axis STATE flags --------------------------------------------------

#define ORB_FLAG_INHIBIT_NEG 0x01
#define ORB_FLAG_INHIBIT_POS 0x02
#define ORB_FLAG_ENABLED 0x04
#define ORB_FLAG_FAULT 0x08

// -- payload sizes ---------------------------------------------------------
// Asserted against Python's struct sizes by the cross-check test, so a field
// added on one side and forgotten on the other fails immediately.

#define ORB_LEN_HELLO 1
#define ORB_LEN_SEG 9
#define ORB_LEN_ABORT 1
#define ORB_LEN_ENABLE 2
#define ORB_LEN_CLEAR_FAULT 1
#define ORB_LEN_PING 1
#define ORB_LEN_STATUS 0
#define ORB_LEN_IDENT 20
#define ORB_LEN_ACK 3
#define ORB_LEN_NACK 3
#define ORB_LEN_DONE 4
#define ORB_LEN_ABORTED 5
#define ORB_LEN_EVENT 2
#define ORB_LEN_PONG 1

#endif  // ORBITALY_PROTOCOL_H
