#ifndef _MACROS_QUADTREE_H_
#define _MACROS_QUADTREE_H_

// Distance calculations
#define DIST_EUCLIDEAN(x,y,x_,y_) ( ( (x)-(x_) )*( (x)-(x_) ) + ( (y)-(y_) )*( (y)-(y_) ) )
#define DIST_MANHATTAN(x,y,x_,y_) ( abs( (x)-(x_) ) + abs( (y)-(y_) ) )

// Bound check functions
#define INBOUND_RECT(x,y,rect) (((x) > ( (rect).tl.x) && (x) < ( (rect).br.x) && (y) > ( (rect).tl.y) && (y) < ( (rect).br.y)))
#define INBOUND_RECT_PTS(x,y,x_tl,y_tl,x_br,y_br) ( (x) > (x_tl) && (x) < (x_br) && (y) > (y_tl) && (y) < (y_br) )

// related to the quadtree node ID
// child idx (flag_sn, flag_ew) ( sn << 1 | ew )
// Z-order
// |-------|-------|
// |0 (0,0)|1 (0,1)|
// |-------|-------|
// |2 (1,0)|3 (1,1)|
// |-------|-------|
//
// 
//  id_node * 4  - 2 -> first child id
// (id_node + 2) / 4 -> parent id
// (id_node + 2) & 0b11 -> my quadrant index (0~3))

#define GET_PARENT_ID(id_node)                ( ((id_node) + 2) >> 2 )
#define GET_FIRST_CHILD_ID(id_node)           ( ((id_node) << 2) - 2 )
#define GET_CHILD_ID_FLAGS(id_node,sn,ew)     ( ((id_node) << 2) - 2 + ((sn) << 1) + (ew) )
#define GET_CHILD_ID_INDEX(id_node,idx_child) ( ((id_node) << 2) - 2 + (idx_child) )
#define GET_MY_QUADRANT(id_node)              ( ((id_node) + 2) & 0b11 )

#define FIND_QUADRANT(x_,y_,qrect,sn,ew) { (ew) = (x_) > (((qrect).tl.x+(qrect).br.x)>>1); (sn) = (y_) > (((qrect).tl.y+(qrect).br.y)>>1); }

#define GET_CHILD_INDEX(sn,ew) ( ((sn) << 1) | (ew) )

// 
#define DIV4_QUOTIENT(i)  ( (i) >> 2 )
#define DIV4_REMAINDER(i) ( (i) & 0b11 )

#endif

/*
    nonzero bit counting JAVA : int.bitCount(i);
    uint32_t bitCount3(uint32_t i){
        i = (i & 0x55555555) + ((i >> 1) & 0x55555555);  // ...01010101010101010101010101010101 (32 bits)
        i = (i & 0x33333333) + ((i >> 2) & 0x33333333);  // ...00110011001100110011001100110011 (32 bits)
        i = (i & 0x0F0F0F0F) + ((i >> 4) & 0x0F0F0F0F);  // ...00001111000011110000111100001111 (32 bits)
        i = (i & 0x00FF00FF) + ((i >> 8) & 0x00FF00FF);  // ...00000000111111110000000011111111 (32 bits)
        i = (i & 0x0000FFFF) + ((i >> 16) & 0x0000FFFF); // ...00000000000000001111111111111111 (32 bits)
        return i & 0x3F;
    };
*/


/*
ref : http://graphics.stanford.edu/~seander/bithacks.html#IntegerLogDeBruijn
- Find the log base 2 of an N-bit integer in O(lg(N)) operations
unsigned int v;  // 32-bit value to find the log2 of 
const unsigned int b[] = {0x2, 0xC, 0xF0, 0xFF00, 0xFFFF0000};
const unsigned int S[] = {1, 2, 4, 8, 16};
int i;
register unsigned int r = 0; // result of log2(v) will go here
for (i = 4; i >= 0; i--) // unroll for speed...
{
  if (v & b[i])
  {
    v >>= S[i];
    r |= S[i];
  } 
}
// OR (IF YOUR CPU BRANCHES SLOWLY):
unsigned int v;	         // 32-bit value to find the log2 of 
register unsigned int r; // result of log2(v) will go here
register unsigned int shift;
r =     (v > 0xFFFF) << 4; v >>= r;
shift = (v > 0xFF  ) << 3; v >>= shift; r |= shift;
shift = (v > 0xF   ) << 2; v >>= shift; r |= shift;
shift = (v > 0x3   ) << 1; v >>= shift; r |= shift;
                                        r |= (v >> 1);
// OR (IF YOU KNOW v IS A POWER OF 2):
unsigned int v;  // 32-bit value to find the log2 of 
static const unsigned int b[] = {0xAAAAAAAA, 0xCCCCCCCC, 0xF0F0F0F0, 
                                 0xFF00FF00, 0xFFFF0000};
register unsigned int r = (v & b[0]) != 0;
for (i = 4; i > 0; i--) // unroll for speed...
{
  r |= ((v & b[i]) != 0) << i;
}
Of course, to extend the code to find the log of a 33- to 64-bit number, we would append another element, 0xFFFFFFFF00000000, to b, append 32 to S, and loop from 5 to 0. This method is much slower than the earlier table-lookup version, but if you don't want big table or your architecture is slow to access memory, it's a good choice. The second variation involves slightly more operations, but it may be faster on machines with high branch costs (e.g. PowerPC).
The second version was sent to me by Eric Cole on January 7, 2006. Andrew Shapira subsequently trimmed a few operations off of it and sent me his variation (above) on Sept. 1, 2007. The third variation was suggested to me by John Owens on April 24, 2002; it's faster, but it is only suitable when the input is known to be a power of 2. On May 25, 2003, Ken Raeburn suggested improving the general case by using smaller numbers for b[], which load faster on some architectures (for instance if the word size is 16 bits, then only one load instruction may be needed). These values work for the general version, but not for the special-case version below it, where v is a power of 2; Glenn Slayden brought this oversight to my attention on December 12, 2003. 
OR
int __builtin_clz (unsigned int x)
 Returns the number of leading 0-bits in x, starting at the most significant bit position. If x is 0, the result is undefined.
 
const int tab64[64] = {
    63,  0, 58,  1, 59, 47, 53,  2,
    60, 39, 48, 27, 54, 33, 42,  3,
    61, 51, 37, 40, 49, 18, 28, 20,
    55, 30, 34, 11, 43, 14, 22,  4,
    62, 57, 46, 52, 38, 26, 32, 41,
    50, 36, 17, 19, 29, 10, 13, 21,
    56, 45, 25, 31, 35, 16,  9, 12,
    44, 24, 15,  8, 23,  7,  6,  5};
int log2_64 (uint64_t value)
{
    value |= value >> 1;
    value |= value >> 2;
    value |= value >> 4;
    value |= value >> 8;
    value |= value >> 16;
    value |= value >> 32;
    return tab64[((uint64_t)((value - (value >> 1))*0x07EDD5E59A4E28C2)) >> 58];
}
const int tab32[32] = {
     0,  9,  1, 10, 13, 21,  2, 29,
    11, 14, 16, 18, 22, 25,  3, 30,
     8, 12, 20, 28, 15, 17, 24,  7,
    19, 27, 23,  6, 26,  5,  4, 31};
int log2_32 (uint32_t value)
{
    value |= value >> 1;
    value |= value >> 2;
    value |= value >> 4;
    value |= value >> 8;
    value |= value >> 16;
    return tab32[(uint32_t)(value*0x07C4ACDD) >> 27];
}
*/


/*
- bit interleaving
int main() {
    uint8_t size = 4;
    uint8_t xx[size]; // 8 bits
    uint8_t yy[size]; // 8 bits
    uint16_t zz[size]; // interleaved data. (two times more bits)
    
    static const unsigned int B[] = {0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF};
    static const unsigned int S[] = {1, 2, 4, 8};
    unsigned int x = 0b1011; // Interleave lower 16 bits of x and y, so the bits of x
    unsigned int y = 0b0011; // are in the even positions and bits from y in the odd;
    unsigned int z; // z gets the resulting 32-bit Morton Number.  
                    // x and y must initially be less than 65536.
    cout << bitset<8>(x) <<"," <<bitset<8>(y) ;
    x = (x | (x << S[3])) & B[3];
    x = (x | (x << S[2])) & B[2];
    x = (x | (x << S[1])) & B[1];
    x = (x | (x << S[0])) & B[0];
    y = (y | (y << S[3])) & B[3];
    y = (y | (y << S[2])) & B[2];
    y = (y | (y << S[1])) & B[1];
    y = (y | (y << S[0])) & B[0];
    z = y | (x << 1);
/*
(z & 0b10101010) : y 추출 (0xAAAAAAAA) (0xA == 0b1010)
(z | 0b01010101) : (0x55555555) (0x5 == 0b0101)
// top    = ((( (z) & 0b10101010) − 1) & 0b10101010) | ( (z) & 0b01010101) // y 에서 1을 빼고, 다시 합쳐줌.
y 추출, -1 후 다시 y로 만들어줌.
// bottom = ((( (z) | 0b01010101) + 1) & 0b10101010) | ( (z) & 0b01010101) // y 에서 1을 더하고, 다시 합쳐줌.
// left   = ((( (z) & 0b01010101) − 1) & 0b01010101) | ( (z) & 0b10101010) // 
// right  = ((( (z) | 0b10101010) + 1) & 0b01010101) | ( (z) & 0b10101010)
*/