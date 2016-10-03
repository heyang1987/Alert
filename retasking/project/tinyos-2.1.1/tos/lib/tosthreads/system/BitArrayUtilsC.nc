/*
 * Copyright (c) 2008 Johns Hopkins University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * @author Chieh-Jan Mike Liang <cliang4@cs.jhu.edu>
 * @author Kevin Klues <klueska@cs.stanford.edu>
 */

module BitArrayUtilsC {
  provides interface BitArrayUtils;
}

implementation {
  uint16_t getByteIndex(uint8_t bitIndex) {
    return bitIndex / 8;
  }
  uint8_t getMask(uint8_t bitIndex) {
    return 1 << (bitIndex % 8);
  }
  async command void BitArrayUtils.clrArray(uint8_t* array, uint8_t size) {
    memset(array, 0, size);
  }
  async command bool BitArrayUtils.getBit(uint8_t* array, uint8_t bitIndex) {
    return (array[getByteIndex(bitIndex)] & getMask(bitIndex)) ? TRUE : FALSE;
  }
  async command void BitArrayUtils.setBit(uint8_t* array, uint8_t bitIndex) {
    array[getByteIndex(bitIndex)] |= getMask(bitIndex);
  }
  async command void BitArrayUtils.clrBit(uint8_t* array, uint8_t bitIndex) {
    array[getByteIndex(bitIndex)] &= ~getMask(bitIndex);
  }
}
