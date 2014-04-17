/*
 * The MIT License (MIT)
 * Copyright (c) 2014 Tamas Nepusz
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
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef YAK_UTIL_HPP
#define YAK_UTIL_HPP

namespace yak {

template <typename Type>
Type standard_normal(void) {
   static Type v, fac;
   static int phase = 0;
   Type S, Z, U1, U2, u;

   if (phase)
      Z = v * fac;
   else
   {
      do
      {
         U1 = static_cast<Type>(rand()) / RAND_MAX;
         U2 = static_cast<Type>(rand()) / RAND_MAX;

         u = 2. * U1 - 1.;
         v = 2. * U2 - 1.;
         S = u * u + v * v;
      } while(S >= 1);

      fac = sqrt (-2. * log(S) / S);
      Z = u * fac;
   }

   phase = 1 - phase;

   return Z;
}

}       // end of namespaces

#endif

