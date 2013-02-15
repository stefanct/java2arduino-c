/** \file
Arduino2java-wide constants.*/

#ifndef A2J_CONST_H
#define A2J_CONST_H

#define A2J_OVERHEAD 5 /**< Number of protocol (i.e. non-payload) bytes in arduino2j packets.*/
#define A2J_MAX_PAYLOAD 255 /**< Maximum number of bytes to be transmitted as payload in arduino2j packets.*/
#define A2J_BUFFER (A2J_OVERHEAD + 2 * A2J_MAX_PAYLOAD) /**< Number of bytes the buffer in a j2a_handle can hold.*/
#define A2J_TIMEOUT 10 /**< Timeout for single byte reads from the stream (in milliseconds). */

/** @addtogroup j2aframing java2arduino framing characters
\see \ref framing */
//@{
#define A2J_SOF 0x12 /**< Start of frame.*/
#define A2J_ESC 0x7D /**< Escape character.*/
//@}

/** @addtogroup j2acrc java2arduino crc constants
\see \ref crc */
//@{
#define A2J_CRC_CMD 11 /**< Constant to be added to the command offset byte. */
#define A2J_CRC_LEN 97 /**< Constant to be added to the length byte. */
//@}

/** @addtogroup j2amany*/
//@{
#define A2J_MANY_HEADER 6 /**< Number of bytes of the a2jMany header. Consisting of the function offset, isLast and 4B offset.*/
#define A2J_MANY_PAYLOAD (A2J_MAX_PAYLOAD - A2J_MANY_HEADER) /**< Maximum number of bytes to be transmitted as payload in a2jMany packets.*/
//@}

/** @addtogroup j2aerrors java2arduino error values */
//@{
#define A2J_RET_OOB 0xF0 /**< Out of bounds of the arduino2j jump table.*/
#define A2J_RET_TO 0xF2 /**< Timeout while waiting for a byte.*/
#define A2J_RET_CHKSUM 0xF3 /**< Checksum error.*/
//@}

#endif // A2J_CONST_H
