/*****************************************************************************************
* FILENAME :        RfDevices.h
*
* DESCRIPTION :
*       RF Interface definitions
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
#ifndef RFDEVICES_H_
#define RFDEVICES_H_

/****************************************************************************************/
/* Imported header files: */
#include "RfProcl.h"

/****************************************************************************************/
/* Global constant defines: */
#define FROM_NODE_ID_BATH_1   0x00u
#define FROM_NODE_ID_CHILD    0x01u
#define FROM_NODE_ID_PARENTS  0x02u
#define FROM_NODE_ID_GUEST    0x03u

#define TO_NODE_ID_1FLOOR     0x00u
#define TO_NODE_ID_0FLOOR     0x01u

#define MSG_ID_TEMP           0x00u
#define MSG_ID_HUM            0x01u
#define MSG_ID_BAT            0x02u
#define MSG_ID_CNT            0x03u
#define MSG_ID_CONST          0x00u

/****************************************************************************************/
/* Global function like macro defines (to be avoided): */

/****************************************************************************************/
/* Global type definitions (enum, struct, union): */

/****************************************************************************************/
/* Class definition: */

#endif /* RFDEVICES_H_ */
