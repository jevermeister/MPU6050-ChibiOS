// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//     2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//     2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//     2011-10-03 - added automatic Arduino version detection for ease of use
//     2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//     2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//     2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//     2011-08-02 - added support for 16-bit registers
//                - fixed incorrect Doxygen comments on some methods
//                - added timeout value for read operations (thanks mem @ Arduino forums)
//     2011-07-30 - changed read/write function structures to return success or byte counts
//                - made all methods static for multi-device memory savings
//     2011-07-28 - initial release

/* ChibiOS I2Cdev Main I2C device class conversion 2/5/2013 by Jan Schlemminger - C conversion, ChibiOS compliance
 * First release. I just tested byte-based reading so this should be considered HIGHLY EXPERIMENTAL!!!
 * 
 * 
 * Feel free to test and report bugs. Updates at https://github.com/jevermeister/MPU6050-ChibiOS
*/

/* ============================================
ChibiOS I2Cdev Main I2C device class code is placed under the MIT license
Copyright (c) 2012 Jan Schlemminger

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _I2CDEV_CHIBI_H_
#define _I2CDEV_CHIBI_H_

/* currently only one I2C bus is possible */
#define I2C_MPU 			I2CD1
#define MPU_INT_PORT		GPIOB
#define MPU_INT_PIN			4


#define I2CDEV_DEFAULT_READ_TIMEOUT     1000
#define I2CDEV_BUFFER_LENGTH			64

int8_t I2CdevreadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout);
int8_t I2CdevreadBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout);
int8_t I2CdevreadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout);
int8_t I2CdevreadBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout);
int8_t I2CdevreadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout);
int8_t I2CdevreadWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout);
int8_t I2CdevreadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout);
int8_t I2CdevreadWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout);

bool_t I2CdevwriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
bool_t I2CdevwriteBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
bool_t I2CdevwriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
bool_t I2CdevwriteBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
bool_t I2CdevwriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
bool_t I2CdevwriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
bool_t I2CdevwriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
bool_t I2CdevwriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

#endif /* _I2CDEV_CHIBI_H_ */
