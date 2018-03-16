/*****************************************************************************************
* FILENAME :        RfProcl.h
*
* DESCRIPTION :
*       Implementation of the RfProtocol
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
*               range: 00, 10, 01, 11
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
*****************************************************************************************/
#ifndef RFPROCL_H_
#define RFPROCL_H_

/****************************************************************************************/
/* Imported header files: */
#include <stdint.h>

/****************************************************************************************/
/* Global constant defines: */
#define TO_NODE_ID_00       0x00u
#define TO_NODE_ID_01       0x01u
#define TO_NODE_ID_02       0x02u
#define TO_NODE_ID_03       0x03u

#define FROM_NODE_ID_00     0x00u
#define FROM_NODE_ID_01     0x01u
#define FROM_NODE_ID_02     0x02u
#define FROM_NODE_ID_03     0x03u

#define MSG_ID_00           0x00u
#define MSG_ID_01           0x01u
#define MSG_ID_02           0x02u
#define MSG_ID_03           0x03u

/****************************************************************************************/
/* Global function like macro defines (to be avoided): */

/****************************************************************************************/
/* Global type definitions (enum, struct, union): */

typedef struct msg_tag
{
    uint8_t     header_u8;
    uint16_t    data_u16;
    uint8_t     chkSum_u8;
}msg_t;

/****************************************************************************************/
/* Class definition: */
class RfProcl
{
    public:
        /********************************************************************************/
        /* Public data definitions */
        /********************************************************************************/
        /* Public function definitions: */
        static bool VerifyMessage(msg_t *msg_p);
        static uint8_t GetToNodeId(msg_t *msg_p);
        static uint8_t GetFromNodeId(msg_t *msg_p);
        static uint8_t GetMsgTypeId(msg_t *msg_p);
        static uint16_t GetMsgData(msg_t *msg_p);

        static void InitializeMessage(msg_t *msg_p);
        static bool SetToNodeId(msg_t *msg_p, uint8_t toNodeId_u8);
        static bool SetFromNodeId(msg_t *msg_p, uint8_t fromNodeId_u8);
        static bool SetMsgTypeId(msg_t *msg_p, uint8_t msgTypeId_u8);
        static bool SetMsgData(msg_t *msg_p, uint16_t data_u16);
        static bool CalculateChkSum(msg_t *msg_p);
        static uint32_t GetRawData(msg_t *msg_p);
        static void SetRawData(msg_t *msg_p, uint32_t rawData_u32);

        //virtual ~RfProcl();
    private:
        /********************************************************************************/
        /* Private data definitions */

        /********************************************************************************/
        /* Private function definitions: */
    protected:
        /********************************************************************************/
        /* Protected data definitions */

        /********************************************************************************/
        /* Protected function definitions: */

};

#endif /* RFPROCL_H_ */
