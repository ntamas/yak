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

