/* Generated by Nim Compiler v1.6.12 */
#define NIM_INTBITS 32

#include "nimbase.h"
#include <string.h>
#undef LANGUAGE_C
#undef MIPSEB
#undef MIPSEL
#undef PPC
#undef R3000
#undef R4000
#undef i386
#undef linux
#undef mips
#undef near
#undef far
#undef powerpc
#undef unix
  #  define nimfr_(proc, file) \
      TFrame FR_; \
      FR_.procname = proc; FR_.filename = file; FR_.line = 0; FR_.len = 0; nimFrame(&FR_);

  #  define nimfrs_(proc, file, slots, length) \
      struct {TFrame* prev;NCSTRING procname;NI line;NCSTRING filename; NI len; VarSlot s[slots];} FR_; \
      FR_.procname = proc; FR_.filename = file; FR_.line = 0; FR_.len = length; nimFrame((TFrame*)&FR_);

  #  define nimln_(n, file) \
      FR_.line = n; FR_.filename = file;
  typedef struct NimStrPayload NimStrPayload;
typedef struct NimStringV2 NimStringV2;
struct NimStrPayload {NI cap;
NIM_CHAR data[SEQ_DECL_SIZE];
};
struct NimStringV2 {NI len;
NimStrPayload* p;
};
typedef NF tyArray__B5fORh4Yn8C9bv7acOsFpVA[7];
typedef NU8 tyEnum_Fields__IBnxUOsZ4pbBazHA7whJ8A;
N_LIB_PRIVATE N_NIMCALL(void, addInt__stdZprivateZdigitsutils_174)(NimStringV2* result, NI64 x);
static N_INLINE(void, addInt__stdZprivateZdigitsutils_196)(NimStringV2* result, NI x);
static N_INLINE(void, nimFrame)(TFrame* s);
N_LIB_PRIVATE N_NOINLINE(void, callDepthLimitReached__system_5032)(void);
static N_INLINE(void, popFrame)(void);
N_LIB_PRIVATE N_NIMCALL(NimStringV2, collectionToString__main_81)(tyArray__B5fORh4Yn8C9bv7acOsFpVA x, NimStringV2 prefix, NimStringV2 separator, NimStringV2 suffix);
N_LIB_PRIVATE N_NIMCALL(void, eqcopy___system_3536)(NimStringV2* dest, NimStringV2 src);
static N_INLINE(void, appendString)(NimStringV2* dest, NimStringV2 src);
static N_INLINE(void, copyMem__system_1709)(void* dest, void* source_0, NI size);
static N_INLINE(void, nimCopyMem)(void* dest, void* source_0, NI size);
N_LIB_PRIVATE N_NOINLINE(void, raiseOverflow)(void);
N_LIB_PRIVATE N_NOINLINE(void, raiseRangeErrorI)(NI64 i, NI64 a, NI64 b);
N_LIB_PRIVATE N_NIMCALL(void, prepareAdd)(NimStringV2* s, NI addlen);
N_LIB_PRIVATE N_NIMCALL(void, addQuoted__main_122)(NimStringV2* s, NF x);
static N_INLINE(NIM_BOOL*, nimErrorFlag)(void);
static const struct {
  NI cap; NIM_CHAR data[1+1];
} TM__n49a9aYp5BrbXv9a6OCpJYm0g_4 = { 1 | NIM_STRLIT_FLAG, "[" };
static const NimStringV2 TM__n49a9aYp5BrbXv9a6OCpJYm0g_5 = {1, (NimStrPayload*)&TM__n49a9aYp5BrbXv9a6OCpJYm0g_4};
static const struct {
  NI cap; NIM_CHAR data[2+1];
} TM__n49a9aYp5BrbXv9a6OCpJYm0g_6 = { 2 | NIM_STRLIT_FLAG, ", " };
static const NimStringV2 TM__n49a9aYp5BrbXv9a6OCpJYm0g_7 = {2, (NimStrPayload*)&TM__n49a9aYp5BrbXv9a6OCpJYm0g_6};
static const struct {
  NI cap; NIM_CHAR data[1+1];
} TM__n49a9aYp5BrbXv9a6OCpJYm0g_8 = { 1 | NIM_STRLIT_FLAG, "]" };
static const NimStringV2 TM__n49a9aYp5BrbXv9a6OCpJYm0g_9 = {1, (NimStrPayload*)&TM__n49a9aYp5BrbXv9a6OCpJYm0g_8};
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR NIM_BOOL nimInErrorMode__system_4861;

#line 549 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
static N_INLINE(void, nimFrame)(TFrame* s) {
#line 550 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	{
#line 550 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		if (!(framePtr__system_4513 == ((TFrame*) NIM_NIL))) goto LA3_;

#line 551 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		(*s).calldepth = ((NI16) 0);	}
	goto LA1_;
	LA3_: ;
	{
#line 554 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"

#line 554 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		(*s).calldepth = (NI16)((*framePtr__system_4513).calldepth + ((NI16) 1));	}
	LA1_: ;

#line 556 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	(*s).prev = framePtr__system_4513;
#line 557 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	framePtr__system_4513 = s;
#line 558 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	{
#line 558 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		if (!((*s).calldepth == ((NI16) 2000))) goto LA8_;

#line 558 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"

#line 558 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		callDepthLimitReached__system_5032();
	}
	LA8_: ;
}

#line 99 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
static N_INLINE(void, popFrame)(void) {
#line 100 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	framePtr__system_4513 = (*framePtr__system_4513).prev;}

#line 116 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/std/private/digitsutils.nim"
static N_INLINE(void, addInt__stdZprivateZdigitsutils_196)(NimStringV2* result, NI x) {	nimfr_("addInt", "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/std/private/digitsutils.nim");
#line 117 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/std/private/digitsutils.nim"
	nimln_(117, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/std/private/digitsutils.nim");
#line 117 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/std/private/digitsutils.nim"
	addInt__stdZprivateZdigitsutils_174(result, ((NI64) (x)));
	popFrame();}

#line 10 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
N_LIB_PRIVATE N_NIMCALL(NimStringV2, dollar___systemZdollars_3)(NI x) {	NimStringV2 result;	nimfr_("$", "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");	result.len = 0; result.p = NIM_NIL;
#line 12 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
	nimln_(12, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");
#line 12 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
	addInt__stdZprivateZdigitsutils_196((&result), x);
	popFrame();	return result;}

#line 14 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
N_LIB_PRIVATE N_NIMCALL(NimStringV2, dollar___systemZdollars_6)(NI64 x) {	NimStringV2 result;	nimfr_("$", "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");	result.len = 0; result.p = NIM_NIL;
#line 16 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
	nimln_(16, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");
#line 16 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
	addInt__stdZprivateZdigitsutils_174((&result), x);
	popFrame();	return result;}

#line 8 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/memory.nim"
static N_INLINE(void, nimCopyMem)(void* dest, void* source_0, NI size) {	void* T1_;
#line 10 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/memory.nim"

#line 10 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/memory.nim"
	T1_ = (void*)0;	T1_ = memcpy(dest, source_0, ((size_t) (size)));}

#line 2220 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system.nim"
static N_INLINE(void, copyMem__system_1709)(void* dest, void* source_0, NI size) {
#line 2221 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system.nim"

#line 2221 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system.nim"
	nimCopyMem(dest, source_0, size);
}

#line 94 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/strs_v2.nim"
static N_INLINE(void, appendString)(NimStringV2* dest, NimStringV2 src) {{
#line 95 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/strs_v2.nim"
	{		NI TM__n49a9aYp5BrbXv9a6OCpJYm0g_2;		NI TM__n49a9aYp5BrbXv9a6OCpJYm0g_3;
#line 95 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/strs_v2.nim"
		if (!(((NI) 0) < src.len)) goto LA3_;

#line 97 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/strs_v2.nim"

#line 97 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/strs_v2.nim"
		if (nimAddInt(src.len, ((NI) 1), &TM__n49a9aYp5BrbXv9a6OCpJYm0g_2)) { raiseOverflow(); goto BeforeRet_;};		if (((NI)(TM__n49a9aYp5BrbXv9a6OCpJYm0g_2)) < ((NI) 0) || ((NI)(TM__n49a9aYp5BrbXv9a6OCpJYm0g_2)) > ((NI) 2147483647)){ raiseRangeErrorI((NI)(TM__n49a9aYp5BrbXv9a6OCpJYm0g_2), ((NI) 0), ((NI) 2147483647)); goto BeforeRet_;}
#line 97 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/strs_v2.nim"
		copyMem__system_1709(((void*) ((&(*(*dest).p).data[(*dest).len]))), ((void*) ((&(*src.p).data[((NI) 0)]))), ((NI) ((NI)(TM__n49a9aYp5BrbXv9a6OCpJYm0g_2))));

#line 98 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/strs_v2.nim"
		if (nimAddInt((*dest).len, src.len, &TM__n49a9aYp5BrbXv9a6OCpJYm0g_3)) { raiseOverflow(); goto BeforeRet_;};		(*dest).len = (NI)(TM__n49a9aYp5BrbXv9a6OCpJYm0g_3);	}
	LA3_: ;
	}BeforeRet_: ;
}

#line 120 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
N_LIB_PRIVATE N_NIMCALL(NimStringV2, collectionToString__main_81)(tyArray__B5fORh4Yn8C9bv7acOsFpVA x, NimStringV2 prefix, NimStringV2 separator, NimStringV2 suffix) {	NimStringV2 result;	NIM_BOOL firstElement;	nimfr_("collectionToString", "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");	result.len = 0; result.p = NIM_NIL;
#line 54 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/fatal.nim"
	nimln_(54, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/fatal.nim");
#line 54 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/fatal.nim"
	eqcopy___system_3536((&result), prefix);

#line 122 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
	nimln_(122, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");	firstElement = NIM_TRUE;	{		NF value;		tyEnum_Fields__IBnxUOsZ4pbBazHA7whJ8A i;		value = (NF)0;
#line 33 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim"
		nimln_(33, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim");		i = ((tyEnum_Fields__IBnxUOsZ4pbBazHA7whJ8A) 0);		{
#line 34 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim"
			nimln_(34, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim");			while (1) {
#line 123 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
				nimln_(123, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");				value = x[(i)- 0];
#line 123 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
				{					if (!firstElement) goto LA6_;

#line 125 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
					nimln_(125, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");					firstElement = NIM_FALSE;				}
				goto LA4_;
				LA6_: ;
				{
#line 127 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
					nimln_(127, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");					prepareAdd((&result), separator.len + 0);appendString((&result), separator);				}
				LA4_: ;

#line 123 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
				nimln_(123, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");
#line 123 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
				addQuoted__main_122((&result), value);

#line 36 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim"
				nimln_(36, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim");				{
#line 36 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim"
					if (!(((tyEnum_Fields__IBnxUOsZ4pbBazHA7whJ8A) 6) <= i)) goto LA11_;

#line 36 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim"
					goto LA2;
				}
				LA11_: ;

#line 37 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim"
				nimln_(37, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/iterators.nim");				i += ((NI) 1);			}
		} LA2: ;
	}

#line 137 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
	nimln_(137, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");	prepareAdd((&result), suffix.len + 0);appendString((&result), suffix);	popFrame();	return result;}

#line 423 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
static N_INLINE(NIM_BOOL*, nimErrorFlag)(void) {	NIM_BOOL* result;	result = (NIM_BOOL*)0;
#line 424 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	result = (&nimInErrorMode__system_4861);	return result;}

#line 167 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
N_LIB_PRIVATE N_NIMCALL(NimStringV2, dollar___main_76)(tyArray__B5fORh4Yn8C9bv7acOsFpVA x) {	NimStringV2 result;NIM_BOOL* nimErr_;	nimfr_("$", "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");{nimErr_ = nimErrorFlag();	result.len = 0; result.p = NIM_NIL;
#line 169 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
	nimln_(169, "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim");
#line 169 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/dollars.nim"
	result = collectionToString__main_81(x, TM__n49a9aYp5BrbXv9a6OCpJYm0g_5, TM__n49a9aYp5BrbXv9a6OCpJYm0g_7, TM__n49a9aYp5BrbXv9a6OCpJYm0g_9);	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;	}BeforeRet_: ;
	popFrame();	return result;}
