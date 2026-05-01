/*-------------------------------------------------------------------------
   _mullonglong.c - routine for multiplication of 64 bit int64_t

   Copyright (C) 2012, Philipp Klaus Krause . philipp@informatik.uni-frankfurt.de

   This library is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation; either version 2, or (at your option) any
   later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this library; see the file COPYING. If not, write to the
   Free Software Foundation, 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301, USA.

   As a special exception, if you link this library with other files,
   some of which are compiled with SDCC, to produce an executable,
   this library does not by itself cause the resulting executable to
   be covered by the GNU General Public License. This exception does
   not however invalidate any other reasons why the executable file
   might be covered by the GNU General Public License.
-------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

int64_t _mullonglong(int64_t ll, int64_t lr)
{
  uint64_t ret = 0ull;
  uint8_t i, j;

  for (i = 0; i < sizeof (int64_t); i++)
    {
      uint8_t l = ll >> (i * 8);
      for(j = 0; (i + j) < sizeof (int64_t); j++)
        {
          uint8_t r = lr >> (j * 8);
          ret += (uint64_t)((uint16_t)(l * r)) << ((i + j) * 8);
        }
    }

  return(ret);
}

uint32_t div64_32_fast(uint64_t n, uint32_t d) {
    uint64_t rem = 0;
    uint32_t quot = 0;

    // We only need to loop 32 times if we handle the high 32 bits 
    // of the numerator as the initial remainder.
    rem = n >> 32; 
    uint32_t low = (uint32_t)n;

    for (int8_t i = 31; i >= 0; i--) {
        rem = (rem << 1) | ((low >> i) & 1);
        if (rem >= d) {
            rem -= d;
            quot |= (1UL << i);
        }
    }
    return quot;
}

int64_t _divslonglong (int64_t numerator, int64_t denominator)
{
  bool numeratorneg = (numerator < 0);
  bool denominatorneg = (denominator < 0);
  int64_t d;

  if (numeratorneg)
    numerator = -numerator;
  if (denominatorneg)
    denominator = -denominator;

  d = div64_32_fast(numerator, denominator);

  return ((numeratorneg ^ denominatorneg) ? -d : d);
}