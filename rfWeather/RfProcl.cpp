/*****************************************************************************************
* FILENAME :        RfProcl.cpp
*
* DESCRIPTION :
*       Implementation of the RfDht sensor device
*           The messages expected from transmitter are expected in the following format:
*           expecting the max send payload per message to be 32Bit.
*           32 bit
*           -------------------------------------------------
*           | 31 - 25   | 24 - 16   | 15 - 8    | 7 - 0     |
*           -------------------------------------------------
*           |   Header  |    16 bit DataWord    | CheckSum  |
*           -------------------------------------------------
*
*           The Header includes node and payload identification:
*           -------------------------------------------------
*           | 7     | 6 - 5   | 4 - 3   | 2 - 1   | 0       |
*           -------------------------------------------------
*           | PreAmb| ToNode  | FrNode  | MsgId   | PostAmb |
*           -------------------------------------------------           
*           PreAmb: The message header starts with a 1 bit preamble set to 1.

*           ToNode: This is the receiver identifier to generate a pointed message. 
*               The system currently supports 4 different receivers within one network 
*               range: 00, 10, 01, 11.
*
*           FrNode: it is possible to connect up to 4 nodes to one base, the node id's 
*               have to be unique, therefore the identifier can be defined out of two
*               bits: 00, 01, 10, 11
*           MsgId: Is used to identify the payload of this node. 4 different
*               payloads are currently supported:
*                   00 : Msg 0
*                   01 : Msg 1
*                   10 : Msg 2
*                   11 : Msg 3
*           PostAmb: is the header end identifier and is always 1
*
*           CheckSum: A bytewise and of the three bytes message.   
*
* NOTES :
*
*
* Copyright (c) [2017] [Stephan Wink]
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
vAUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* AUTHOR :    Stephan Wink        START DATE :    01.10.2017
*
*****************************************************************************************/

/****************************************************************************************/
/* Include Interfaces */
#include <stdint.h>
#include <stddef.h>

#include "RfProcl.h"

/****************************************************************************************/
/* Local constant defines */
#define MSG_PREAMBLE                0x80u
#define MSG_POSTAMBLE               0x01u
#define MSG_TO_NODE_ID_MASK         0x60u
#define MSG_FROM_NODE_ID_MASK       0x18u
#define MSG_FROM_MSGTYPE_ID_MASK    0x06u
#define MAX_NODE_ID                 0x03u
#define MAX_MESSAGE_TYPE            0x03u 

/****************************************************************************************/
/* Local function like makros */

/****************************************************************************************/
/* Local type definitions (enum, struct, union) */

/****************************************************************************************/
/* Static Data instantiation */

/****************************************************************************************/
/* Public functions (unlimited visibility) */

/**---------------------------------------------------------------------------------------
 * @brief     Verifies the message by checking the pre/post amble and checksum
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    true, if message is compliant to checks
*//*-----------------------------------------------------------------------------------*/
bool RfProcl::VerifyMessage(msg_t *msg_p)
{
    bool msgCorrect_bol = true;
    uint8_t tmpChkSum_u8 = 0u;

    msgCorrect_bol &= (0 != (MSG_PREAMBLE & msg_p->header_u8));
    msgCorrect_bol &= (0 != (MSG_POSTAMBLE & msg_p->header_u8));
    tmpChkSum_u8 += msg_p->header_u8;
    tmpChkSum_u8 += (uint8_t)(msg_p->data_u16 >> 8);
    tmpChkSum_u8 += (uint8_t)(msg_p->data_u16 & 0x00FF);
    msgCorrect_bol &= (tmpChkSum_u8 == msg_p->chkSum_u8);

    return(msgCorrect_bol);
}

/**---------------------------------------------------------------------------------------
 * @brief     returns the receiver id of the message
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    receiver id 
*//*-----------------------------------------------------------------------------------*/
uint8_t RfProcl::GetToNodeId(msg_t *msg_p)
{
    uint8_t toNodeId_u8 = 0;

    toNodeId_u8 = msg_p->header_u8 & MSG_TO_NODE_ID_MASK;
    toNodeId_u8 = toNodeId_u8 >> 5; 

    return(toNodeId_u8);     
}

/**---------------------------------------------------------------------------------------
 * @brief     returns the sender id of the message
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    sender id 
*//*-----------------------------------------------------------------------------------*/
uint8_t RfProcl::GetFromNodeId(msg_t *msg_p)
{
    uint8_t fromNodeId_u8 = 0;

    fromNodeId_u8 = msg_p->header_u8 & MSG_FROM_NODE_ID_MASK;
    fromNodeId_u8 = fromNodeId_u8 >> 3; 

    return(fromNodeId_u8);     
}

/**---------------------------------------------------------------------------------------
 * @brief     returns the msg id of the message
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    message id 
*//*-----------------------------------------------------------------------------------*/
uint8_t RfProcl::GetMsgTypeId(msg_t *msg_p)
{
    uint8_t msgTypeId_u8 = 0;

    msgTypeId_u8 = msg_p->header_u8 & MSG_FROM_MSGTYPE_ID_MASK;
    msgTypeId_u8 = msgTypeId_u8 >> 1; 

    return(msgTypeId_u8);
}

/**---------------------------------------------------------------------------------------
 * @brief     returns the message data
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    message data 
*//*-----------------------------------------------------------------------------------*/
uint16_t RfProcl::GetMsgData(msg_t *msg_p)
{
    return(msg_p->data_u16);
}

/**---------------------------------------------------------------------------------------
 * @brief     reset the message to zero and set the default marker
 * @author    winkste
 * @date      11. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    - 
*//*-----------------------------------------------------------------------------------*/
void RfProcl::InitializeMessage(msg_t *msg_p)
{
    // reset the complete message to 0
    msg_p->header_u8 = 0;
    msg_p->data_u16 = 0u;
    msg_p->chkSum_u8 = 0;

    // set pre-and post ambel
    msg_p->header_u8 |= MSG_PREAMBLE;
    msg_p->header_u8 |= MSG_POSTAMBLE;
}

/**---------------------------------------------------------------------------------------
 * @brief     set the receiver address in message header
 * @author    winkste
 * @date      11. Mar. 2018
 * @param     msg_p         pointer to message buffer
 * @param     toNodeId_u8   receiver address
 * @return    true if input parameter are valid 
*//*-----------------------------------------------------------------------------------*/
bool RfProcl::SetToNodeId(msg_t *msg_p, uint8_t toNodeId_u8)
{
    bool fctAck_bol = false;

    if((NULL != msg_p) && (toNodeId_u8 <= MAX_NODE_ID))
    {
        msg_p->header_u8 |= (toNodeId_u8 << 5);
        fctAck_bol = true;
    }

    return(fctAck_bol);
}

/**---------------------------------------------------------------------------------------
 * @brief     set the sender address in message header
 * @author    winkste
 * @date      11. Mar. 2018
 * @param     msg_p             pointer to message buffer
 * @param     fromNodeId_u8     sender address
 * @return    true if input parameter are valid 
*//*-----------------------------------------------------------------------------------*/
bool RfProcl::SetFromNodeId(msg_t *msg_p, uint8_t fromNodeId_u8)
{
    bool fctAck_bol = false;

    if((NULL != msg_p) && (fromNodeId_u8 <= MAX_NODE_ID))
    {
        msg_p->header_u8 |= (fromNodeId_u8 << 3);
        fctAck_bol = true;       
    }

    return(fctAck_bol);
}

/**---------------------------------------------------------------------------------------
 * @brief     set the message type in message header
 * @author    winkste
 * @date      11. Mar. 2018
 * @param     msg_p             pointer to message buffer
 * @param     msgTypeId_u8      message type
 * @return    true if input parameter are valid 
*//*-----------------------------------------------------------------------------------*/
bool RfProcl::SetMsgTypeId(msg_t *msg_p, uint8_t msgTypeId_u8)
{
    bool fctAck_bol = false;

    if((NULL != msg_p) && (msgTypeId_u8 <= MAX_MESSAGE_TYPE))
    {
        msg_p->header_u8 |= (msgTypeId_u8 << 1);
        fctAck_bol = true;       
    }

    return(fctAck_bol);
}

/**---------------------------------------------------------------------------------------
 * @brief     set the message data in message header
 * @author    winkste
 * @date      11. Mar. 2018
 * @param     msg_p             pointer to message buffer
 * @param     data_u16          message data
 * @return    true if input parameter are valid 
*//*-----------------------------------------------------------------------------------*/
bool RfProcl::SetMsgData(msg_t *msg_p, uint16_t data_u16)
{
    bool fctAck_bol = false;

    if(NULL != msg_p)
    {
        msg_p->data_u16 = data_u16;
        fctAck_bol = true;       
    }

    return(fctAck_bol);
}

/**---------------------------------------------------------------------------------------
 * @brief     set the message data in message header
 * @author    winkste
 * @date      11. Mar. 2018
 * @param     msg_p             pointer to message buffer
 * @return    true if input parameter are valid 
*//*-----------------------------------------------------------------------------------*/
bool RfProcl::CalculateChkSum(msg_t *msg_p)
{
    bool fctAck_bol = false;

    if(NULL != msg_p)
    {
        msg_p->chkSum_u8 =  msg_p->header_u8;
        msg_p->chkSum_u8 += (uint8_t)(msg_p->data_u16 >> 8);
        msg_p->chkSum_u8 += (uint8_t)(msg_p->data_u16 & 0x00FF);
        fctAck_bol = true;       
    }

    return(fctAck_bol);
}

/**---------------------------------------------------------------------------------------
 * @brief     return 32Bit raw message data
 * @author    winkste
 * @date      11. Mar. 2018
 * @param     msg_p             pointer to message buffer
 * @return    32Bit raw message stream
*//*-----------------------------------------------------------------------------------*/
uint32_t RfProcl::GetRawData(msg_t *msg_p)
{
    uint32_t rawData_u32;
    /*uint16_t data_u16a[2];

    data_u16a[0] = (uint16_t)((((uint16_t)myMessage_s.header_u8) << 8) + ((myMessage_s.data_u16 & 0xFF00) >> 8));
    data_u16a[1] = (uint16_t)(((myMessage_s.data_u16 & 0x00FF) << 8) + myMessage_s.chkSum_u8);
    rawData_u32 = ((uint32_t)(data_u16a[0]) << 16) + data_u16a[1];*/
    
    rawData_u32 =  (msg_p->chkSum_u8);
    rawData_u32 += ((uint32_t)msg_p->data_u16 << 8);
    rawData_u32 += ((uint32_t)msg_p->header_u8 << 24);
    
    return(rawData_u32);
}

/**---------------------------------------------------------------------------------------
 * @brief     set raw message data
 * @author    winkste
 * @date      11. Mar. 2018
 * @param     msg_p             pointer to message buffer
 * @param     rawData_u32       raw message data
 * @return    -
*//*-----------------------------------------------------------------------------------*/
void RfProcl::SetRawData(msg_t *msg_p, uint32_t rawData_u32)
{
    msg_p->header_u8 = (uint8_t)((rawData_u32 & 0xFF000000) >> 24);
    msg_p->data_u16 = (uint16_t)((rawData_u32 & 0x00FFFF00) >> 8);
    msg_p->chkSum_u8 = (uint8_t)(rawData_u32 & 0x000000FF);
}

/****************************************************************************************/
/* Private functions: */


