#ifndef __INT64_H__
#define __INT64_H__

#include <stdint.h>

#ifndef __INT64_T_TYPE__




typedef struct {
  uint32_t h;
  uint32_t l;
} U_64;

#define PAWN_CELL_SIZE  32
#define ucell uint32_t

#define WORDSHIFT   (PAWN_CELL_SIZE/2)

#if PAWN_CELL_SIZE==32
    typedef unsigned short word_t;
    #define LOWORD(v)   (word_t)((v) & 0xffffu)
    #define HIWORD(v)   (word_t)(((v) >> 16) & 0xffffu)
#elif PAWN_CELL_SIZE==64
    typedef uint32_t    word_t;
    #define LOWORD(v)   (word_t)((v) & 0xffffffffu)
    #define HIWORD(v)   (word_t)(((v) >> 32) & 0xffffffffu)
#else
    #error Unsupported cell size
#endif

#if 0
//https://github.com/nordlow/justcxx/blob/master/muldiv64.h
//https://github.com/esneider/div64/blob/master/div64.h
//https://github.com/GarethNelson/openpdk/blob/ff01af6be7dd69cb569e3775fb08d8e9c780d8ef/pawn/source/amx/fixed.c

/*(k*ah + al) * (k*bh + bl) = k*k*(ah*bh) + k*((ah*bl + al*bh)) + al*bl */
void mul32u(uint32_t a, uint32_t b, U_64 *pr) {
    uint32_t ah = a >> 16, al = a & 0xFFFFU;
    uint32_t bh = b >> 16, bl = b & 0xFFFFU;
    uint32_t rl = (al * bl);
    uint32_t rm1 = ah * bl;
    uint32_t rm2 = al * bh;
    uint32_t rh = (ah * bh);
    uint32_t rm1h = rm1 >> 16;
    uint32_t rm1l = rm1 & 0xFFFFU;
    uint32_t rm2h = rm2 >> 16;
    uint32_t rm2l = rm2 & 0xFFFFU;
    uint32_t rmh = (rm1h + rm2h);
    uint32_t rml = (rm1l + rm2l);

    rl += rml << 16;
    if (rml & 0xFFFF0000U)
    rmh++;
    rh += rmh;

    pr->l = rl;
    pr->h = rh;
}


ucell div64_32(ucell t[2], ucell divisor)
{
  /* This function was adapted from source code that appeared in
   * Dr. Dobb's Journal, August 1992, page 117.
   */
  ucell u, v;
  word_t rHigh, rLow, dHigh, dLow;

//  assert(divisor!=0);
//  /* if the divisor is smaller than t[1], the result will not fit in a cell */
//  assert(divisor>=t[1]);

  dHigh=HIWORD(divisor);
  dLow=LOWORD(divisor);

  /* Underestimate high half of quotient and subtract product
   * of estimate and divisor from dividend.
   */
  rHigh = (word_t)(t[1] / (dHigh + 1));
  u = (ucell)rHigh * (ucell)dLow;
  v = (ucell)rHigh * (ucell)dHigh;
  if ((t[0] -= (u << WORDSHIFT)) > ((ucell)-1L - (u << WORDSHIFT)))
    t[1]--;
  t[1] -= HIWORD(u);
  t[1] -= v;

  /* Correct estimate. */
  while ((t[1] > (ucell)dHigh) || ((t[1] == (ucell)dHigh) && (t[0] >= ((ucell)dLow << WORDSHIFT)))) {
    if ((t[0] -= ((ucell)dLow << WORDSHIFT)) > (ucell)-1L - ((ucell)dLow << WORDSHIFT))
      t[1]--;
    t[1] -= dHigh;
    rHigh++;
  } /* while */
  /* Underestimate low half of quotient and subtract product of
   * estimate and divisor from what remains of dividend.
   */
  rLow = (word_t) ((ucell)((t[1] << WORDSHIFT) + HIWORD(t[0])) / (dHigh + 1));
  u = (ucell)rLow * (ucell)dLow;
  v = (ucell)rLow * (ucell)dHigh;
  if ((t[0] -= u) > ((ucell)-1L - u))
    t[1]--;
  if ((t[0] -= (v << WORDSHIFT)) > ((ucell)-1L - (v << WORDSHIFT)))
    t[1]--;
  t[1] -= HIWORD(v);

  /* Correct estimate. */
  while ((t[1] > 0) || ((t[1] == 0) && t[0] >= divisor)) {
    if ((t[0] -= divisor) > ((ucell)-1L - divisor))
      t[1]--;
    rLow++;
  } /* while */

  return ((ucell)rHigh << WORDSHIFT) + rLow;
}    

#else
void mul32u(uint32_t a, uint32_t b, U_64 *pr);
ucell div64_32(ucell t[2], ucell divisor);
#endif

//(k*ah + al) / c = (k*ah/c + k*ah%c + al /c + al%c)
inline uint32_t muldiv32u(uint32_t a, uint32_t b, uint32_t c) {
  U_64 r;
  mul32u(a, b, &r);
#ifdef DEBUG_INT
  printf("muldiv32u1: %lu x %lu = %lu|%lu\n\r", a, b, r.h, r.l);
  uint32_t low      = 0xFFFF & (uint32_t)r.l;
  uint32_t mid_low  = 0xFFFF & (uint32_t)(r.l >> 16);
  uint32_t mid_high = 0xFFFF & (uint32_t)r.h;
  uint32_t high     =          (uint32_t)(r.h >> 16);
  printf("muldiv32u2: %04x|%04x|%04x|%04x\n\r", (uint16_t)high, (uint16_t)mid_high, (uint16_t)mid_low, (uint16_t)low);
#endif
  //uint32_t dividend  = high / c;  
  uint32_t arg[2] = {r.l, r.h};
  uint32_t res = div64_32(arg, c);
#ifdef DEBUG_INT
  printf("muldiv32u3: %lu\n\r", res);
#endif
  return res;
}

inline int32_t muldiv32s(int32_t a, int32_t b, int32_t c) {
  int32_t sign = 1;
  if (a<0) { 
    sign *= -1;
    a *= -1;
  }
  if (b<0) {
    sign *= -1;
    b *= -1;
  }
  if (c<0) {
    sign *= -1;
    c *= -1;
  }

  return sign*((int32_t)muldiv32u(a, b, c));
}


inline int32_t mulsh32s(int32_t a, int32_t b, uint8_t c) {
  int32_t sign = 1;
  if (a<0) { 
    sign *= -1;
    a *= -1;
  }
  if (b<0) {
    sign *= -1;
    b *= -1;
  }

  U_64 r;
  mul32u(a, b, &r);
  if(c > 31) {
    c -= 32;
    r.h >>= c;
    return sign * r.h;
  } else {
    r.l >>= c;
    r.h <<= (32-c);
    r.l |=r.h;
    return sign * r.l;
  }
}

inline int32_t muladddiv32u(int32_t a, int32_t b, int32_t c, int32_t d) {
  U_64 r;
  mul32u(a, b, &r);
  r.l += c;
  if(r.l < c) r.h++;
  uint32_t arg[2] = {r.l, r.h};
  return div64_32(arg, c);
}







//https://github.com/nordlow/justcxx/blob/master/muldiv64.h
//https://github.com/esneider/div64/blob/master/div64.h
//https://github.com/GarethNelson/openpdk/blob/ff01af6be7dd69cb569e3775fb08d8e9c780d8ef/pawn/source/amx/fixed.c
/*(k*ah + al) * (k*bh + bl) = k*k*(ah*bh) + k*((ah*bl + al*bh)) + al*bl */
//(k*ah + al) / c = (k*ah/c + k*ah%c + al /c + al%c)
uint32_t muldiv32u(uint32_t a, uint32_t b, uint32_t c);
int32_t muldiv32s(int32_t a, int32_t b, int32_t c);
int32_t mulsh32s(int32_t a, int32_t b, uint8_t c);
int32_t muladddiv32u(int32_t a, int32_t b, int32_t c, int32_t d);

#define muldiv32ss(a, b, c) ((a/c)*(b/c)*c + (a%c)*(b/c) + (a/c)*(b%c) + (a%c)*(b%c)/c)
//#define muldiv32u(a, b, c) ((a/c)*(b/c)*c + (a%c)*(b/c) + (a/c)*(b%c) + (a%c)*(b%c)/c)
//#define muldiv32s muldiv32u



#else
#define muldiv32u(a, b, c) (((uint64_t)a)*((uint64_t)b)/c)
#define muldiv32s(a, b, c) (((int64_t)a)*((int64_t)b)/c)
#define muladddiv32s(a, b, c, d) ((((int64_t)a)*((int64_t)b) + c)/d)
#define muldiv32ss muldiv32s
#endif



#endif //__INT64_H__